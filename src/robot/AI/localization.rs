use std::f64::consts::{FRAC_PI_8, PI};
use rand::distributions::WeightedIndex;
use rand::prelude::*;
use crate::utility::{Point, Pose};
use crate::robot::map::Map2D;

/// A localizer that uses likelyhood-based Monte Carlo Localization
/// and takes in motion and range finder sensor data
pub struct DistanceFinderMCL {
    max_particle_count: usize,
    weight_sum_threshold: f64,
    pub map: Map2D,
    sensor_poses: Vec<Pose>,
    weight_from_error: Box<dyn FnMut(&f64) -> f64 + Send + Sync>,
    pub belief: Vec<Pose>,
}

impl DistanceFinderMCL {
    pub fn new(
        max_particle_count: usize,
        map: Map2D,
        sensor_poses: Vec<Pose>,
        weight_from_error: Box<dyn FnMut(&f64) -> f64 + Send + Sync>,
    ) -> Self {
        let mut belief = Vec::with_capacity(max_particle_count);
        for _ in 0..max_particle_count {
            belief.push(Pose::random(0.0..2. * PI, 0.0..map.width, 0.0..map.height));
        }
        Self {
            max_particle_count,
            weight_sum_threshold: max_particle_count as f64 / 50., // TODO: fixed parameter
            map,
            sensor_poses,
            weight_from_error,
            belief,
        }
    }

    pub fn from_distributions<T, U>(
        x_distr: T,
        y_distr: T,
        angle_distr: T,
        max_particle_count: usize,
        map: Map2D,
        sensor_poses: Vec<Pose>,
        weight_from_error: Box<dyn FnMut(&f64) -> f64 + Send + Sync>,
    ) -> Self
    where
        T: Distribution<U>,
        U: Into<f64>,
    {
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
        Self {
            max_particle_count,
            weight_sum_threshold: max_particle_count as f64 / 50., // TODO: fixed parameter
            map,
            sensor_poses,
            weight_from_error,
            belief,
        }
    }

    /// Takes in the total change in pose sensed by motion sensors since the last update
    pub fn control_update(&mut self, u: Pose) {
        for i in 0..self.belief.len() {
            self.belief[i] += u;
        }
    }

    /// Takes in a vector of ranges indexed synchronously with `self.sensor_poses`
    pub fn observation_update(&mut self, z: Vec<Option<f64>>) {
        // TODO: These weights need a lot more fixing
        // TODO: Don't just use a constant, c'mon
        const MAX_SENSOR_RANGE: f64 = 15.;
        let mut errors: Vec<f64> = Vec::with_capacity(self.belief.len());
        for sample in &self.belief {
            let mut sum_error = 0.;
            for (i, observation) in z.iter().enumerate() {
                let pred_observation = self.map.raycast(*sample + self.sensor_poses[i]);
                sum_error += match observation {
                    Some(real_dist) => match pred_observation {
                        Some(pred) => {
                            let pred_dist = pred.dist(sample.position);
                            if pred_dist <= MAX_SENSOR_RANGE {
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
        let mut rng = thread_rng();
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
        // TODO: rather than have max particle count and weight sum threshold parameters,
        // it might be beneficial to use some dynamic combination of the two as the break condition.
        while sum_weights < self.weight_sum_threshold
            && new_particles.len() < self.max_particle_count
        {
            let idx = distr.sample(&mut rng);
            sum_weights += weights[idx];
            new_particles.push(
                self.belief[idx]
                    + Pose::random(
                        (-FRAC_PI_8 / 8.)..(FRAC_PI_8 / 8.),
                        -0.05..0.05,
                        -0.05..0.05,
                    ), // TODO: fixed parameter
            );
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
