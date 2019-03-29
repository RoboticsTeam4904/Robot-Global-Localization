extern crate rand;
extern crate bitvec;

use rand::prelude::*;
use bitvec::BitVec;
use super::environment::BinaryEnvironment;

/// The generic trait for any sensor
///
/// `T` is what is being sensed (true info from environment) &
/// `U` is the output of the sensor (percieved info from the environment)
pub trait Sensor<U, T> {
    /// Update the sensor
    fn update(&mut self, env: U);
    /// Gets the value that the sensor is currently sensing
    fn sense(&self) -> T;
}

pub struct MovementSensor {
    pub error_margin: usize,
    pub error_chance: f64,
    pub latest_movement: isize,
}

impl MovementSensor {
    pub fn new(error_margin: usize, error_chance: f64) -> Self {
        Self {
            error_margin,
            error_chance,
            latest_movement: 0,
        }
    }
}

impl Sensor<&BinaryEnvironment, isize> for MovementSensor {
    fn update(&mut self, input: &BinaryEnvironment) {
        let mut rng = thread_rng();
        self.latest_movement = input.robot_velocity as isize
            + if rng.gen_bool(self.error_chance) {
                rng.gen_range(0, self.error_margin + 1) as isize * rng.gen_range(-1, 1)
            } else {
                0
            };
    }

    fn sense(&self) -> isize {
        self.latest_movement
    }
}

pub struct BinarySensor {
    pub breadth: isize,
    pub error_chance: f64,
    pub triggers: BitVec,
}

impl BinarySensor {
    pub fn new(breadth: usize, error_chance: f64) -> Self {
        Self {
            breadth: breadth as isize,
            error_chance,
            triggers: BitVec::with_capacity(1 + breadth * 2),
        }
    }
}

impl Sensor<&BinaryEnvironment, BitVec> for BinarySensor {
    fn update(&mut self, input: &BinaryEnvironment) {
        self.triggers.clear();
        let mut rng = thread_rng();
        for i in -self.breadth..=self.breadth {
            let position = input.robot_position as isize + i;
            if position < 0 || position as usize >= input.map.len() {
                continue;
            }
            let triggered = input.map[position as usize];
            self.triggers.push(if rng.gen_bool(self.error_chance) {
                !triggered
            } else {
                triggered
            });
        }
    }

    fn sense(&self) -> BitVec {
        self.triggers.clone()
    }
}