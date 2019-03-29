extern crate rand;
extern crate bitvec;

use rand::prelude::*;
use rand::distributions::{Distribution, WeightedIndex};
use bitvec::BitVec;
use crate::utility::clamp;

pub trait MCL<T, U> {
    fn motion_position_update(&mut self, sensor_data: T);
    fn sensor_weight_update(&mut self, sensor_update: U);
    fn resample(&mut self);
    fn get_average_position(&self) -> T;
}

pub struct BinaryMCL {
    pub particles: Vec<(usize, f64)>,
    pub map: BitVec,
}

impl BinaryMCL {
    pub fn new(map: BitVec, particle_count: usize) -> Self {
        let mut rng = thread_rng();
        let mut particles = Vec::with_capacity(particle_count);
        for _ in 0..particle_count {
            particles.push((rng.gen_range(0, map.len()), 1.));
        }
        BinaryMCL { particles, map }
    }

    pub fn from_distribution<U>(map: BitVec, particle_count: usize, distribution: U) -> Self
    where
        U: rand::distributions::Distribution<f64>,
    {
        let mut rng = thread_rng();
        let sampler = distribution.sample_iter(&mut rng);
        let mut particles = Vec::with_capacity(particle_count);
        for particle in sampler.take(particle_count) {
            particles.push((clamp(particle, 0., Some(map.len() as f64)) as usize, 1.));
        }
        BinaryMCL { particles, map }
    }
}

impl MCL<isize, BitVec> for BinaryMCL {
    fn motion_position_update(&mut self, sensor_data: isize) {
        let map = &self.map;
        self.particles.iter_mut().for_each(|p| {
            p.0 = clamp(p.0 as isize + sensor_data, 0isize, Some(map.len() as isize)) as usize
        });
    }

    fn sensor_weight_update(&mut self, sensor_data: BitVec) {
        let map = &self.map;
        self.particles.iter_mut().for_each(|p| {
            let mut total_count = 0.;
            let mut total_correct = 0.;
            let breadth = sensor_data.len() as isize / 2;
            for i in 0..sensor_data.len() {
                let position = (p.0 + i) as isize - breadth;
                if position < 0 || position as usize >= map.len() {
                    continue;
                }
                total_count += 1.;
                if map[position as usize] == sensor_data[i] {
                    total_correct += 1.;
                }
            }
            p.1 = clamp(total_correct / total_count, 0.01, None);
        });
    }

    fn resample(&mut self) {
        let particles = self.particles.as_slice();
        let mut new_particles = Vec::with_capacity(self.particles.len());
        let mut rng = thread_rng();
        let distr = WeightedIndex::new(particles.iter().map(|p| p.1)).unwrap();
        for _ in 0..self.particles.len() {
            new_particles.push(particles[distr.sample(&mut rng)]);
        }
        self.particles = new_particles;
    }

    fn get_average_position(&self) -> isize {
        let mut pos_sum = 0;
        for particle in &self.particles {
            pos_sum += particle.0;
        }
        (pos_sum / self.particles.len()) as isize
    }
}