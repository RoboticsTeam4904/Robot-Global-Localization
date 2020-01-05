use crate::robot::map::Map2D;
use crate::robot::sensors::LimitedSensor;
use crate::robot::sensors::Sensor;
use crate::utility::{Point, Pose};
use nalgebra::{ArrayStorage, ComplexField, Matrix, Matrix5, RowVector5, SymmetricEigen, U11, U5};
use rand::distributions::WeightedIndex;
use rand::prelude::*;
use std::f64::consts::PI;
use std::sync::Arc;

// TODO: c o d e d u p l i c a t i o n
type Matrix11x5 = Matrix<f64, U11, U5, ArrayStorage<f64, U11, U5>>;
struct PoseBelief {}

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

    fn from_distributions<T, U>(max_particle_count: usize, pose_distr: (T, (T, T))) -> Vec<Pose>
    where
        T: Distribution<U>,
        U: Into<f64>,
    {
        let (angle_distr, (x_distr, y_distr)) = pose_distr;
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

/// Uses Unscented Kalman Filter to approximate robot pose
pub struct DistanceKalmanFilter {
    pub map: Arc<Map2D>,
    pub sigma_matrix: Matrix<f64, U11, U5, ArrayStorage<f64, U11, U5>>,
    pub covariance_matrix: Matrix5<f64>,
    beta: f64,
    alpha: f64,
    kappa: f64,
}

impl DistanceKalmanFilter {
    pub fn new(
        covariance_matrix: Matrix5<f64>,
        init_state: RowVector5<f64>, // the components of this vector are x-pos, y-pos, theta, x-acceleration, y-acceleration
        alpha: f64,
        kappa: f64,
        beta: f64,
        map: Arc<Map2D>,
    ) -> Self {
        Self {
            map,
            sigma_matrix: DistanceKalmanFilter::gen_sigma_matrix(
                &covariance_matrix,
                alpha,
                beta,
                kappa,
                init_state,
            ),
            covariance_matrix,
            beta,
            alpha,
            kappa,
        }
    }

    fn gen_sigma_matrix(
        cov_matrix: &Matrix5<f64>,
        alpha: f64,
        beta: f64,
        kappa: f64,
        init_state: RowVector5<f64>,
    ) -> Matrix11x5 {
        let mut rows: Vec<RowVector5<f64>> = Vec::new();
        rows.push(init_state);
        let lambda = (alpha.powi(2)) * (5. + kappa) - 5.;
        let eigendecomp = (cov_matrix * (5. + lambda)).symmetric_eigen();
        let mut diagonalization = eigendecomp.eigenvalues;
        diagonalization.data.iter_mut().for_each(|e| {
            e.sqrt();
        });
        let square_root_cov = eigendecomp.eigenvectors.try_inverse().unwrap()
            * Matrix5::from_diagonal(&diagonalization)
            * eigendecomp.eigenvectors;
        for i in 1..6 {
            rows.push(init_state + square_root_cov.row(i));
        }
        for i in 1..6 {
            rows.push(init_state - square_root_cov.row(i));
        }

        Matrix11x5::from_rows(rows.as_slice())
    }
}

/// A pose localizer that uses likelyhood-based Monte Carlo Localization
/// and takes in motion and range finder sensor data
pub struct DistanceFinderMCL {
    pub map: Arc<Map2D>,
    pub belief: Vec<Pose>,
    max_particle_count: usize,
    weight_sum_threshold: f64,
    weight_from_error: Box<dyn FnMut(&f64) -> f64 + Send + Sync>,
    resampling_noise: Pose,
}

impl DistanceFinderMCL {
    /// Generates a new localizer with the given parameters.
    /// Every step, the localizer should recieve a control and observation update
    pub fn new(
        max_particle_count: usize,
        map: Arc<Map2D>,
        weight_from_error: Box<dyn FnMut(&f64) -> f64 + Send + Sync>,
        resampling_noise: Pose,
    ) -> Self {
        let max_position = (map.width, map.height);
        let belief = PoseBelief::new(max_particle_count, max_position.into());
        Self {
            max_particle_count,
            weight_sum_threshold: max_particle_count as f64 / 50., // TODO: fixed parameter
            map,
            weight_from_error,
            belief,
            resampling_noise,
        }
    }

    /// Similar to new, but instead of generating `belief` based on a uniform distribution,
    /// generates it based on the given `pose_distr` which is in the form (angle distribution, (x distribution, y distribution))
    pub fn from_distributions<T, U>(
        pose_distr: (T, (T, T)),
        max_particle_count: usize,
        map: Arc<Map2D>,
        weight_from_error: Box<dyn FnMut(&f64) -> f64 + Send + Sync>,
        resampling_noise: Pose,
    ) -> Self
    where
        T: Distribution<U>,
        U: Into<f64>,
    {
        let belief = PoseBelief::from_distributions(max_particle_count, pose_distr);
        Self {
            max_particle_count,
            weight_sum_threshold: max_particle_count as f64 / 50., // TODO: fixed parameter
            map,
            weight_from_error,
            belief,
            resampling_noise,
        }
    }

    /// Takes in a sensor which senses the total change in pose sensed since the last update
    pub fn control_update<U: Sensor<Pose>>(&mut self, u: &U) {
        let update = u.sense();
        self.belief.iter_mut().for_each(|p| *p += update);
    }

    /// Takes in a vector of distance finder sensors (e.g. laser range finder)
    pub fn observation_update<Z>(&mut self, z: &[Z])
    where
        Z: Sensor<Option<f64>> + LimitedSensor<f64, Option<f64>>,
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
                        None => 5., // TODO: fixed parameter
                    },
                    None => match pred_observation {
                        Some(_) => 5., // TODO: fixed parameter
                        None => 0.,
                    },
                };
            }
            errors.push(sum_error / z.len() as f64);
        }

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
            new_particles.push(self.belief[idx] + Pose::random_from_range(self.resampling_noise));
        }
        self.belief = new_particles;
    }

    pub fn get_prediction(&self) -> Pose {
        let mut average_pose = Pose::default();
        for sample in &self.belief {
            average_pose += *sample;
        }
        average_pose / (self.belief.len() as f64)
    }
}

/// A pose localizer that uses likelyhood-based Monte Carlo Localization
/// and takes in motion and range finder sensor data
pub struct ObjectDetectorMCL {
    pub map: Arc<Map2D>,
    pub belief: Vec<Pose>,
    max_particle_count: usize,
    weight_sum_threshold: f64,
    weight_from_error: Box<dyn FnMut(&f64) -> f64 + Send + Sync>,
    resampling_noise: Pose,
}

impl ObjectDetectorMCL {
    /// Generates a new localizer with the given parameters.
    /// Every step, the localizer should recieve a control and observation update
    pub fn new(
        max_particle_count: usize,
        map: Arc<Map2D>,
        weight_from_error: Box<dyn FnMut(&f64) -> f64 + Send + Sync>,
        resampling_noise: Pose,
    ) -> Self {
        let max_position = (map.width, map.height);
        let belief = PoseBelief::new(max_particle_count, max_position.into());
        Self {
            max_particle_count,
            weight_sum_threshold: max_particle_count as f64 / 50., // TODO: fixed parameter
            map,
            weight_from_error,
            belief,
            resampling_noise,
        }
    }

    /// Similar to new, but instead of generating `belief` based on a uniform distribution,
    /// generates it based on the given `pose_distr` which is in the form (angle distribution, (x distribution, y distribution))
    pub fn from_distributions<T, U>(
        pose_distr: (T, (T, T)),
        max_particle_count: usize,
        map: Arc<Map2D>,
        weight_from_error: Box<dyn FnMut(&f64) -> f64 + Send + Sync>,
        resampling_noise: Pose,
    ) -> Self
    where
        T: Distribution<U>,
        U: Into<f64>,
    {
        let belief = PoseBelief::from_distributions(max_particle_count, pose_distr);
        Self {
            max_particle_count,
            weight_sum_threshold: max_particle_count as f64 / 50., // TODO: fixed parameter
            map,
            weight_from_error,
            belief,
            resampling_noise,
        }
    }

    /// Takes in a sensor which senses the total change in pose since the last update
    pub fn control_update<U: Sensor<Pose>>(&mut self, u: &U) {
        let update = u.sense();
        self.belief.iter_mut().for_each(|p| *p += update);
    }

    /// Takes in a sensor which senses all objects within a certain field of view
    pub fn observation_update<Z>(&mut self, z: &Z)
    where
        Z: Sensor<Vec<Point>> + LimitedSensor<f64, Vec<Point>>,
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
            let mut total = 0;
            for (real, pred) in observation.iter().zip(pred_observation.iter()) {
                sum_error += (*real - *pred).mag();
                total += 1;
            }
            // TODO: fixed parameter
            // TODO: panics at Uniform::new called with `low >= high` when erorr is divided by total
            sum_error += 5. * (observation.len() as f64 - pred_observation.len() as f64).abs();
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
            new_particles.push(self.belief[idx] + Pose::random_from_range(self.resampling_noise));
        }
        self.belief = new_particles;
    }

    pub fn get_prediction(&self) -> Pose {
        let mut average_pose = Pose::default();
        for sample in &self.belief {
            average_pose += *sample;
        }
        average_pose / (self.belief.len() as f64)
    }
}
