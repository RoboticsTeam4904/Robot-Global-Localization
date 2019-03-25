extern crate rand;

use std::fmt;
use rand::prelude::*;

struct BinaryEnvironment {
    map: Vec<bool>,
    size: usize,
    robot_position: usize,
}

impl BinaryEnvironment {
    fn new(size: usize) -> Self {
        let mut map: Vec<bool> = Vec::with_capacity(size);
        for _ in 0..size {
            map.push(random());
        }
        Self {
            map,
            size,
            robot_position: rand::thread_rng().gen_range(0, size),
        }
    }

    fn move_robot(&mut self, dist: isize) -> isize {
        let previous_position = self.robot_position as isize;
        self.robot_position = if self.robot_position as isize + dist < 0 {
            0
        } else {
            self.size - 1
        };
        self.robot_position as isize - previous_position
    }
}

impl fmt::Display for BinaryEnvironment {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let a = self.map.iter().map(|&bit| if bit { "◽◽◽" } else { "◾◾◾" }).collect::<String>();
        let mut b = " ".repeat((self.size - 1) * 3);
        b.insert(self.robot_position * 3 + 1, '🤖');
        write!(f, "{}\n{}", a, b)
    }
}

trait Sensor<T, U> {
    fn update(&mut self, input: T);
    fn sense(&self) -> U;
}

struct MovementSensor {
    error_margin: usize,
    latest_movement: isize,
}

impl Sensor<isize, isize> for MovementSensor {
    fn update(&mut self, input: isize) {
        let mut rng = rand::thread_rng();
        self.latest_movement = input + rng.gen_range(0, self.error_margin) as isize * rng.gen_range(-1, 1);
    }

    fn sense(&self) -> isize {
        self.latest_movement
    }
}

struct BinarySensor {
    error_chance: f64,
    triggered: bool,
}

impl Sensor<&BinaryEnvironment, bool> for BinarySensor {
    fn update(&mut self, input: &BinaryEnvironment) {
        let triggered = input.map[input.robot_position];
        self.triggered = if rand::thread_rng().gen_bool(self.error_chance) {
                !triggered
            } else {
                triggered
            };
    }

    fn sense(&self) -> bool {
        self.triggered
    }
}

struct BinarySensingRobot {
    environment: BinaryEnvironment,
    binary_sensor: BinarySensor,
    movement_sensor: MovementSensor,
}

impl BinarySensingRobot {
    fn make_move(&mut self, dist: isize) {
        self.movement_sensor.update(
            self.environment.move_robot(dist)
        );
        self.binary_sensor.update(&self.environment);
    }
}

fn main() {
}
