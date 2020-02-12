use crate::{
    robot::{map::Map2D, sensors::LimitedSensor, sensors::Sensor},
    utility::{Point, Pose},
};
use rand::{distributions::WeightedIndex, prelude::*};
use rayon::prelude::*;
use statrs::function::erf::erf;
use std::{f64::consts::*, sync::Arc};

struct PoseBelief;

impl PoseBelief {
    fn new(max_particle_count: usize, max_position: Point) -> Vec<Pose> {
        let mut belief = Vec::with_capacity(max_particle_count);
        for _ in 0..max_particle_count {
            belief.push(Pose::random(
                0.0..2. * PI,
                0.0..max_position.x,
                0.0..max_position.y,
            ));
        }
        belief
    }

    fn from_distributions<T, U>(max_particle_count: usize, distr: (T, (T, T))) -> Vec<Pose>
    where
        T: Distribution<U>,
        U: Into<f64>,
    {
        let (angle_distr, (x_distr, y_distr)) = distr;
        let mut belief = Vec::with_capacity(max_particle_count);
        for ((x, y), angle) in x_distr
            .sample_iter(&mut thread_rng())
            .zip(y_distr.sample_iter(&mut thread_rng()))
            .zip(angle_distr.sample_iter(&mut thread_rng()))
            .take(max_particle_count)
        {
            belief.push(Pose {
                angle: angle.into(),
                position: Point {
                    x: x.into(),
                    y: y.into(),
                },
            });
        }
        belief
    }
}

pub type WeightCalculator = Box<dyn Fn(&f64) -> f64 + Send + Sync>;
pub type ErrorCalculator<Z> = Box<dyn Fn(&Pose, &Z, Arc<Map2D>) -> f64 + Send + Sync>;
pub type ResampleNoiseCalculator = Box<dyn Fn(usize) -> Pose + Send + Sync>;

pub struct DeathCondition {
    pub particle_count_threshold: usize,
    pub particle_concentration_threshold: f64,
}

impl DeathCondition {
    /// Calculates whether or not the condition described by `self` has been met
    ///
    /// Currently is ignorant of `Pose`s' angles
    pub fn triggered(&self, belief: &[Pose]) -> bool {
        let mut average = Point::default();
        for sample in belief {
            average += sample.position;
        }
        average = average / belief.len() as f64;

        let mut standard_deviation = 0.;
        for sample in belief {
            standard_deviation += (average - sample.position).mag().powi(2);
        }
        standard_deviation = standard_deviation.sqrt();
        println!("\tσ = {}", standard_deviation);
        belief.len() >= self.particle_count_threshold
            && standard_deviation <= self.particle_concentration_threshold
    }
}

/// A localizer that uses likelyhood-based Monte Carlo Localization
/// and takes in motion sensor data and `Z` as sensor data
///
/// `map` is the map on which the filter is localization
///
/// `belief` is the set of particles
///
/// `max_particle_count` is the max number of particles and starting number
///
/// `weight_sum_threshold` is cumulative weight of the belief used for likelyhood-based resampling
///
/// `death_condition` is the condition for the belief after resampling required to "restart" the algorithm
///
/// `weight_from_error` calculates the weight of each particle from its error
///
/// `errors_from_sense` calculates the error of each particle from its sensor data
///
/// `resampling_noise` calculates the amount of noise to add to each particle during resampling
pub struct PoseMCL<Z: Sync + Send> {
    pub map: Arc<Map2D>,
    pub belief: Vec<Pose>,
    max_particle_count: usize,
    weight_sum_threshold: f64,
    death_condition: DeathCondition,
    weight_from_error: WeightCalculator,
    errors_from_sense: ErrorCalculator<Z>,
    resampling_noise: ResampleNoiseCalculator,
}

impl<Z: Sync + Send> PoseMCL<Z> {
    pub fn new(
        max_particle_count: usize,
        weight_sum_threshold: f64,
        death_condition: DeathCondition,
        map: Arc<Map2D>,
        weight_from_error: WeightCalculator,
        errors_from_sense: ErrorCalculator<Z>,
        resampling_noise: ResampleNoiseCalculator,
    ) -> Self {
        let belief = PoseBelief::new(max_particle_count, map.size);
        Self {
            max_particle_count,
            map,
            belief,
            death_condition,
            weight_sum_threshold,
            weight_from_error,
            errors_from_sense,
            resampling_noise,
        }
    }

    pub fn from_distributions<U, V>(
        belief_distr: (U, (U, U)),
        max_particle_count: usize,
        weight_sum_threshold: f64,
        death_condition: DeathCondition,
        map: Arc<Map2D>,
        weight_from_error: WeightCalculator,
        errors_from_sense: ErrorCalculator<Z>,
        resampling_noise: ResampleNoiseCalculator,
    ) -> Self
    where
        U: Distribution<V>,
        V: Into<f64>,
    {
        let belief = PoseBelief::from_distributions(max_particle_count, belief_distr);
        Self {
            max_particle_count,
            weight_sum_threshold,
            death_condition,
            map,
            weight_from_error,
            errors_from_sense,
            belief,
            resampling_noise,
        }
    }

    /// Takes in a sensor which senses the total change in pose sensed since the last update
    pub fn control_update<U: Sensor<Output = Pose>>(&mut self, u: &U) {
        let update = u.sense();
        self.belief.iter_mut().for_each(|p| *p += update);
    }

    /// Takes in a vector of distance finder sensors (e.g. laser range finder)
    pub fn observation_update(&mut self, z: &Z) {
        let errors: Vec<_> = self
            .belief
            .par_iter()
            .map(|sample| (&self.errors_from_sense)(sample, z, self.map.clone()))
            .collect();

        let mut new_particles = Vec::new();
        #[allow(clippy::float_cmp)]
        let weights: Vec<f64> = if errors.iter().all(|error| error == &0.) {
            errors
                .iter()
                .map(|_| 2. * self.weight_sum_threshold / self.belief.len() as f64) // TODO: fixed parameter
                .collect()
        } else {
            errors
                .iter()
                .map(|error| (self.weight_from_error)(error))
                .collect()
        };
        let distr = WeightedIndex::new(weights.clone()).unwrap();
        let mut sum_weights = 0.;
        let mut rng = thread_rng();
        // TODO: rather than have max particle count and weight sum threshold parameters,
        // it might be beneficial to use some dynamic combination of the two as the break condition.
        while sum_weights < self.weight_sum_threshold
            && new_particles.len() < self.max_particle_count
        {
            let idx = distr.sample(&mut rng);
            sum_weights += weights[idx];
            new_particles.push(self.belief[idx]);
        }
        println!("\tΣ = {}", sum_weights);
        self.belief = if self.death_condition.triggered(&new_particles) {
            PoseBelief::new(self.max_particle_count, self.map.size)
        } else {
            new_particles
                .iter()
                .map(|&p| p + (self.resampling_noise)(self.belief.len()))
                .collect()
        };
    }

    pub fn get_prediction(&self) -> Pose {
        let mut average_pose = Pose::default();
        let mut angle = 0.;
        for sample in &self.belief {
            average_pose += *sample;
            angle += sample.angle;
        }
        average_pose.with_angle(angle) / self.belief.len() as f64
    }
}

pub struct KLDPoseMCL<Z: Sync + Send> {
    pub map: Arc<Map2D>,
    pub belief: Vec<Pose>,
    max_particle_count: usize,
    min_particle_count: usize,
    weight_sum_threshold: f64,
    error_bound: f64,      // ε
    error_confidence: f64, // δ
    bin_size: Pose,        // ∆
    death_condition: DeathCondition,
    weight_from_error: WeightCalculator,
    errors_from_sense: ErrorCalculator<Z>,
    resampling_noise: ResampleNoiseCalculator,
}

impl<Z: Sync + Send> KLDPoseMCL<Z> {
        pub fn new(
        max_particle_count: usize,
        min_particle_count: usize,
        weight_sum_threshold: f64,
        error_bound: f64,      // ε
        error_confidence: f64, // δ
        bin_size: Pose,        // ∆
        death_condition: DeathCondition,
        map: Arc<Map2D>,
        weight_from_error: WeightCalculator,
        errors_from_sense: ErrorCalculator<Z>,
        resampling_noise: ResampleNoiseCalculator,
    ) -> Self {
        let belief = PoseBelief::new(max_particle_count, map.size);
        Self {
            max_particle_count,
            map,
            belief,
            min_particle_count,
            error_bound,
            bin_size,
            error_confidence,
            death_condition,
            weight_sum_threshold,
            weight_from_error,
            errors_from_sense,
            resampling_noise,
        }
    }

    pub fn from_distributions<U, V>(
        belief_distr: (U, (U, U)),
        max_particle_count: usize,
        min_particle_count: usize,
        weight_sum_threshold: f64,
        error_bound: f64,      // ε
        error_confidence: f64, // δ
        bin_size: Pose,        // ∆
        death_condition: DeathCondition,
        map: Arc<Map2D>,
        weight_from_error: WeightCalculator,
        errors_from_sense: ErrorCalculator<Z>,
        resampling_noise: ResampleNoiseCalculator,
    ) -> Self
    where
        U: Distribution<V>,
        V: Into<f64>,
    {
        let belief = PoseBelief::from_distributions(max_particle_count, belief_distr);
         Self {
            max_particle_count,
            map,
            belief,
            min_particle_count,
            error_bound,
            bin_size,
            error_confidence,
            death_condition,
            weight_sum_threshold,
            weight_from_error,
            errors_from_sense,
            resampling_noise,
        }
    }

    pub fn get_prediction(&self) -> Pose {
        let mut average_pose = Pose::default();
        let mut angle = 0.;
        for sample in &self.belief {
            average_pose += *sample;
            angle += sample.angle;
        }
        average_pose.with_angle(angle) / self.belief.len() as f64
    }

    pub fn observation_update(&mut self, z: &Z) {
        let errors: Vec<_> = self
            .belief
            .par_iter()
            .map(|sample| (&self.errors_from_sense)(sample, z, self.map.clone()))
            .collect();

        let weights: Vec<f64> = if errors.iter().all(|error| error == &0.) {
            errors
                .iter()
                .map(|_| 2. * self.weight_sum_threshold / self.belief.len() as f64) // TODO: fixed parameter
                .collect()
        } else {
            errors
                .iter()
                .map(|error| (self.weight_from_error)(error))
                .collect()
        };
        let particles = WeightedIndex::new(weights.clone()).unwrap();
        let mut rng = thread_rng();
        let mut new_particles = vec![];
        let mut desired_particles_count = 0.;
        let mut non_empty_bins = vec![];
        for n in 0.. {
            let particle = self.belief[particles.sample(&mut rng)];
            new_particles.push(particle);
            let bin = Pose {
                angle: (particle.angle / self.bin_size.angle).floor(),
                position: (
                    (particle.position.x / self.bin_size.position.x).floor(),
                    (particle.position.y / self.bin_size.position.y).floor(),
                )
                    .into(),
            };
            if !non_empty_bins.contains(&bin) {
                let k = non_empty_bins.len() as f64;
                non_empty_bins.push(bin);
                let normal_quantile =
                    (1. - erf(4. * (1. - self.error_confidence) / 2f64.sqrt())) / 2.; // Take the upper (1 - self.error_confidence)% of the normal distribution
                let a = 2. / (9. * k);
                desired_particles_count =
                    k / (2. * self.error_bound) * (1. - a + a.sqrt() * normal_quantile).powi(3);
            }
            if n as f64 >= desired_particles_count && n >= self.min_particle_count {
                break;
            }
        }
        self.belief = if self.death_condition.triggered(&new_particles) {
            PoseBelief::new(self.max_particle_count, self.map.size)
        } else {
            new_particles
                .iter()
                .map(|&p| p + (self.resampling_noise)(self.belief.len()))
                .collect()
        };
    }
}

/// A localizer that uses likelyhood-based Monte Carlo Localization
/// and takes in motion and range finder sensor data
///
/// `map` is the map on which the filter is localization
///
/// `belief` is the set of particles
///
/// `max_particle_count` is the max number of particles and starting number
///
/// `weight_sum_threshold` is cumulative weight of the belief used for likelyhood-based resampling
///
/// `death_condition` is the condition for the belief after resampling required to "restart" the algorithm
///
/// `weight_from_error` calculates the weight of each particle from its error
///
/// `resampling_noise` calculates the amount of noise to add to each particle during resampling
pub struct DistanceFinderMCL {
    pub map: Arc<Map2D>,
    pub belief: Vec<Pose>,
    max_particle_count: usize,
    weight_sum_threshold: f64,
    death_condition: DeathCondition,
    weight_from_error: WeightCalculator,
    resampling_noise: ResampleNoiseCalculator,
}

impl DistanceFinderMCL {
    /// Generates a new localizer with the given parameters.
    /// Every step, the localizer should recieve a control and observation update
    pub fn new(
        max_particle_count: usize,
        weight_sum_threshold: f64,
        death_condition: DeathCondition,
        map: Arc<Map2D>,
        weight_from_error: WeightCalculator,
        resampling_noise: ResampleNoiseCalculator,
    ) -> Self {
        let belief = PoseBelief::new(max_particle_count, map.size);
        Self {
            max_particle_count,
            weight_sum_threshold,
            map,
            weight_from_error,
            death_condition,
            belief,
            resampling_noise,
        }
    }

    /// Similar to new, but instead of generating `belief` based on a uniform distribution,
    /// generates it based on the given `pose_distr` which is in the form (angle distribution, (x distribution, y distribution))
    pub fn from_distributions<T, U>(
        distr: (T, (T, T)),
        max_particle_count: usize,
        weight_sum_threshold: f64,
        death_condition: DeathCondition,
        map: Arc<Map2D>,
        weight_from_error: WeightCalculator,
        resampling_noise: ResampleNoiseCalculator,
    ) -> Self
    where
        T: Distribution<U>,
        U: Into<f64>,
    {
        let belief = PoseBelief::from_distributions(max_particle_count, distr);
        Self {
            max_particle_count,
            weight_sum_threshold,
            death_condition,
            map,
            weight_from_error,
            belief,
            resampling_noise,
        }
    }

    /// Takes in a sensor which senses the total change in pose sensed since the last update
    pub fn control_update<U: Sensor<Output = Pose>>(&mut self, u: &U) {
        let update = u.sense();
        self.belief.iter_mut().for_each(|p| *p += update);
    }

    /// Takes in a vector of distance finder sensors (e.g. laser range finder)
    pub fn observation_update<Z>(&mut self, z: &[Z])
    where
        Z: Sensor<Output = Option<f64>> + LimitedSensor<f64>,
    {
        let mut errors: Vec<f64> = Vec::with_capacity(self.belief.len());
        for sample in &self.belief {
            let mut sum_error = 0.;
            for sensor in z.iter() {
                let pred_observation = self.map.raycast(*sample + sensor.relative_pose());
                sum_error += match sensor.sense() {
                    Some(real_dist) => match pred_observation {
                        Some(pred) => {
                            let pred_dist = pred.dist(sample.position);
                            if pred_dist <= sensor.range().unwrap_or(std::f64::MAX) {
                                (real_dist - pred_dist).abs() // powi(2) // TODO: fixed parameter
                            } else {
                                0.
                            }
                        }
                        None => 6., // TODO: fixed parameter
                    },
                    None => match pred_observation {
                        Some(_) => 6., // TODO: fixed parameter
                        None => 0.,
                    },
                };
            }
            errors.push(sum_error / z.len() as f64);
        }

        let mut new_particles = Vec::new();
        let weights: Vec<f64> = if errors.iter().all(|error| error == &0.) {
            errors
                .iter()
                .map(|_| 2. * self.weight_sum_threshold / self.belief.len() as f64) // TODO: fixed parameter
                .collect()
        } else {
            errors
                .iter()
                .map(|error| (self.weight_from_error)(error))
                .collect()
        };
        let distr = WeightedIndex::new(weights.clone()).unwrap();
        let mut sum_weights = 0.;
        let mut rng = thread_rng();
        // TODO: rather than have max particle count and weight sum threshold parameters,
        // it might be beneficial to use some dynamic combination of the two as the break condition.
        while sum_weights < self.weight_sum_threshold
            && new_particles.len() < self.max_particle_count
        {
            let idx = distr.sample(&mut rng);
            sum_weights += weights[idx];
            new_particles.push(self.belief[idx]);
        }
        println!("\tΣ = {}", sum_weights);
        self.belief = if self.death_condition.triggered(&new_particles) {
            PoseBelief::new(self.max_particle_count, self.map.size)
        } else {
            new_particles
                .iter()
                .map(|&p| p + (self.resampling_noise)(self.belief.len()))
                .collect()
        };
    }

    pub fn get_prediction(&self) -> Pose {
        let mut average_pose = Pose::default();
        let mut angle = 0.;
        for sample in &self.belief {
            average_pose += *sample;
            angle += sample.angle;
        }
        average_pose.with_angle(angle) / self.belief.len() as f64
    }
}

/// A localizer that uses likelyhood-based Monte Carlo Localization
/// and takes in motion and range finder sensor data
///
/// `map` is the map on which the filter is localization
///
/// `belief` is the set of particles
///
/// `max_particle_count` is the max number of particles and starting number
///
/// `weight_sum_threshold` is cumulative weight of the belief used for likelyhood-based resampling
///
/// `death_condition` is the condition for the belief after resampling required to "restart" the algorithm
///
/// `weight_from_error` calculates the weight of each particle from its error
///
/// `resampling_noise` calculates the amount of noise to add to each particle during resampling
pub struct ObjectDetectorMCL {
    pub map: Arc<Map2D>,
    pub belief: Vec<Pose>,
    max_particle_count: usize,
    weight_sum_threshold: f64,
    death_condition: DeathCondition,
    weight_from_error: WeightCalculator,
    resampling_noise: ResampleNoiseCalculator,
}

impl ObjectDetectorMCL {
    /// Generates a new localizer with the given parameters.
    /// Every step, the localizer should recieve a control and observation update
    pub fn new(
        max_particle_count: usize,
        weight_sum_threshold: f64,
        death_condition: DeathCondition,
        map: Arc<Map2D>,
        weight_from_error: WeightCalculator,
        resampling_noise: ResampleNoiseCalculator,
    ) -> Self {
        let belief = PoseBelief::new(max_particle_count, map.size);
        Self {
            max_particle_count,
            weight_sum_threshold,
            death_condition,
            map,
            weight_from_error,
            belief,
            resampling_noise,
        }
    }

    /// Similar to new, but instead of generating `belief` based on a uniform distribution,
    /// generates it based on the given `pose_distr` which is in the form (angle distribution, (x distribution, y distribution))
    pub fn from_distributions<T, U>(
        distr: (T, (T, T)),
        max_particle_count: usize,
        weight_sum_threshold: f64,
        death_condition: DeathCondition,
        map: Arc<Map2D>,
        weight_from_error: WeightCalculator,
        resampling_noise: ResampleNoiseCalculator,
    ) -> Self
    where
        T: Distribution<U>,
        U: Into<f64>,
    {
        let belief = PoseBelief::from_distributions(max_particle_count, distr);
        Self {
            max_particle_count,
            weight_sum_threshold,
            death_condition,
            map,
            weight_from_error,
            belief,
            resampling_noise,
        }
    }

    /// Takes in a sensor which senses the total change in pose since the last update
    pub fn control_update<U: Sensor<Output = Pose>>(&mut self, u: &U) {
        let update = u.sense();
        self.belief.iter_mut().for_each(|p| *p += update);
    }

    /// Takes in a sensor which senses all objects within a certain field of view
    pub fn observation_update<Z>(&mut self, z: &Z)
    where
        Z: Sensor<Output = Vec<Point>> + LimitedSensor<f64>,
    {
        let observation = {
            let mut observation = z.sense();
            observation.sort_by(|a, b| a.mag().partial_cmp(&b.mag()).unwrap());
            observation
        };
        let fov = if let Some(range) = z.range() {
            range
        } else {
            2. * PI
        };
        let mut errors: Vec<f64> = Vec::with_capacity(self.belief.len());
        for sample in &self.belief {
            let mut sum_error = 0.;
            let pred_observation = {
                let mut pred_observation = self.map.cull_points(*sample + z.relative_pose(), fov);
                pred_observation.sort_by(|a, b| a.mag().partial_cmp(&b.mag()).unwrap());
                pred_observation
            };
            // TODO: fixed parameter
            // This method of calculating error is not entirely sound
            for (real, pred) in observation.iter().zip(pred_observation.iter()) {
                sum_error += (*real - *pred).mag();
            }
            // TODO: fixed parameter
            // TODO: panics at Uniform::new called with `low >= high` when erorr is divided by total
            sum_error += 6. * (observation.len() as f64 - pred_observation.len() as f64).abs();
            errors.push(sum_error);
        }

        let mut new_particles = Vec::new();
        #[allow(clippy::float_cmp)]
        let weights: Vec<f64> = if errors
            .iter()
            .all(|error| error == &0. || error == &std::f64::NAN)
        {
            errors
                .iter()
                .map(|_| self.weight_sum_threshold / self.belief.len() as f64) // TODO: fixed parameter
                .collect()
        } else {
            errors
                .iter()
                .map(|error| (self.weight_from_error)(error))
                .collect()
        };
        let distr = WeightedIndex::new(weights.clone()).unwrap();
        let mut sum_weights = 0.;
        let mut rng = thread_rng();
        // TODO: rather than have max particle count and weight sum threshold parameters,
        // it might be beneficial to use some dynamic combination of the two as the break condition.
        while sum_weights < self.weight_sum_threshold
            && new_particles.len() < self.max_particle_count
        {
            let idx = distr.sample(&mut rng);
            sum_weights += weights[idx];
            new_particles.push(self.belief[idx]);
        }
        self.belief = if self.death_condition.triggered(&new_particles) {
            PoseBelief::new(self.max_particle_count, self.map.size)
        } else {
            new_particles
                .iter()
                .map(|&p| p + (self.resampling_noise)(self.belief.len()))
                .collect()
        };
    }

    pub fn get_prediction(&self) -> Pose {
        let mut average_pose = Pose::default();
        let mut angle = 0.;
        for sample in &self.belief {
            average_pose += *sample;
            angle += sample.angle;
        }
        average_pose.with_angle(angle) / self.belief.len() as f64
    }
}
