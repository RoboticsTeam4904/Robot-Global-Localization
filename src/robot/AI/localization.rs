use crate::{
    robot::{map::Map2D, sensors::Sensor},
    utility::{Point, Pose},
};
use rand::{distributions::WeightedIndex, prelude::*};
use rayon::prelude::*;
use statrs::function::erf::erf;
use std::{f64::consts::*, marker::PhantomData, sync::Arc};

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

pub trait WeightCalculator: Fn(&f64) -> f64 {}
impl<T: Fn(&f64) -> f64> WeightCalculator for T {}
pub trait ErrorCalculator<Z>: Fn(&Pose, &Z, Arc<Map2D>) -> f64 {}
impl<Z, T: Fn(&Pose, &Z, Arc<Map2D>) -> f64> ErrorCalculator<Z> for T {}
pub trait ResampleNoiseCalculator: Fn(usize) -> Pose {}
impl<T: Fn(usize) -> Pose> ResampleNoiseCalculator for T {}

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
pub struct PoseMCL<W, E, R, Z>
where
    W: WeightCalculator,
    E: ErrorCalculator<Z>,
    R: ResampleNoiseCalculator,
{
    pub map: Arc<Map2D>,
    pub belief: Vec<Pose>,
    max_particle_count: usize,
    weight_sum_threshold: f64,
    death_condition: DeathCondition,
    weight_from_error: W,
    errors_from_sense: E,
    resampling_noise: R,
    data_type: PhantomData<Z>,
}

impl<W, E, R, Z> PoseMCL<W, E, R, Z>
where
    W: WeightCalculator + Send + Sync,
    E: ErrorCalculator<Z> + Send + Sync,
    R: ResampleNoiseCalculator + Send + Sync,
    Z: Sync + Send,
{
    pub fn new(
        max_particle_count: usize,
        weight_sum_threshold: f64,
        death_condition: DeathCondition,
        map: Arc<Map2D>,
        weight_from_error: W,
        errors_from_sense: E,
        resampling_noise: R,
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
            data_type: PhantomData,
        }
    }

    pub fn from_distributions<U, V>(
        belief_distr: (U, (U, U)),
        max_particle_count: usize,
        weight_sum_threshold: f64,
        death_condition: DeathCondition,
        map: Arc<Map2D>,
        weight_from_error: W,
        errors_from_sense: E,
        resampling_noise: R,
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
            data_type: PhantomData,
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

pub struct KLDPoseMCL<W, E, R, Z>
where
    W: WeightCalculator,
    E: ErrorCalculator<Z>,
    R: ResampleNoiseCalculator,
{
    pub map: Arc<Map2D>,
    pub belief: Vec<Pose>,
    max_particle_count: usize,
    min_particle_count: usize,
    weight_sum_threshold: f64,
    error_bound: f64,      // ε
    error_confidence: f64, // δ
    bin_size: Pose,        // ∆
    death_condition: DeathCondition,
    weight_from_error: W,
    errors_from_sense: E,
    resampling_noise: R,
    data_type: PhantomData<Z>,
}

impl<W, E, R, Z> KLDPoseMCL<W, E, R, Z>
where
    W: WeightCalculator + Send + Sync,
    E: ErrorCalculator<Z> + Send + Sync,
    R: ResampleNoiseCalculator + Send + Sync,
    Z: Sync + Send,
{
    pub fn new(
        max_particle_count: usize,
        min_particle_count: usize,
        weight_sum_threshold: f64,
        error_bound: f64,      // ε
        error_confidence: f64, // δ
        bin_size: Pose,        // ∆
        death_condition: DeathCondition,
        map: Arc<Map2D>,
        weight_from_error: W,
        errors_from_sense: E,
        resampling_noise: R,
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
            data_type: PhantomData,
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
        weight_from_error: W,
        errors_from_sense: E,
        resampling_noise: R,
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
            data_type: PhantomData,
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
