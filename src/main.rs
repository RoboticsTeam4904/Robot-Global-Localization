#[macro_use]
extern crate bitvec;
extern crate rand;

use bitvec::BitVec;
use rand::prelude::*;
use rand::seq::SliceRandom;
use std::fmt;
use std::time::{SystemTime, UNIX_EPOCH};

/// Clamps the `num` to the range `[lower, upper)`
///
/// If `T` is unsigned, do not use an `upper` of `0` because `upper` is tested exclusively
fn clamp<T>(num: T, lower: T, upper: Option<T>) -> T
where
    T: std::ops::Sub<Output = T> + From<i8> + PartialOrd,
{
    if num < lower {
        return lower;
    }
    if let Some(u) = upper {
        if num >= u {
            return u - 1i8.into();
        }
    }
    num
}

// fn clamp_to_range<T, U>(num: T, range: U) -> T
// where U: std::ops::RangeBounds<isize>, T: Into<isize> + From<isize>,
// {
//     let num = num.into();
//     let num = match range.start_bound() {
//         std::ops::Bound::Excluded(lower) if num <= *lower => *lower + 1isize,
//         std::ops::Bound::Included(lower) if num < *lower => *lower,
//         _ => num
//     };
//     match range.end_bound() {
//         std::ops::Bound::Excluded(upper) if num >= *upper => *upper - 1isize,
//         std::ops::Bound::Included(upper) if num > *upper => *upper,
//         _ => num
//     }.into()
// }

/// A one dimensional environment that is filled simply with either a true or false in every space
struct BinaryEnvironment {
    map: BitVec,
    size: usize,
    robot_position: usize,
}

impl BinaryEnvironment {
    fn new(size: usize, map_percentage_true: f64) -> Self {
        let mut map = BitVec::with_capacity(size);
        let mut rng = rand::thread_rng();
        for _ in 0..size {
            map.push(rng.gen_bool(map_percentage_true));
        }
        Self {
            map,
            size,
            robot_position: rand::thread_rng().gen_range(0, size),
        }
    }

    fn move_robot(&mut self, dist: isize) -> isize {
        let previous_position = self.robot_position as isize;
        let new_position = clamp(
            self.robot_position as isize + dist,
            0isize,
            Some(self.size as isize),
        );
        self.robot_position = new_position as usize;
        new_position - previous_position
    }
}

impl fmt::Display for BinaryEnvironment {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        const TILE_SIZE: usize = 1;
        let true_tile = "◾".repeat(TILE_SIZE);
        let false_tile = "◽".repeat(TILE_SIZE);
        let map = self
            .map
            .iter()
            .map(|bit| {
                if bit {
                    true_tile.clone()
                } else {
                    false_tile.clone()
                }
            })
            .collect::<String>();
        let mut robot = " ".repeat((self.size - 1) * TILE_SIZE);
        robot.insert(self.robot_position * TILE_SIZE + TILE_SIZE / 2, '🤖');
        write!(f, "{}\n{}", map, robot)
    }
}

/// The generic trait for any sensor
///
/// `T` is what is being sensed (true info from environment) &
/// `U` is the output of the sensor (percieved info from the environment)
trait Sensor<T, U> {
    /// Update the sensor
    fn update(&mut self, input: T) -> &Self;
    /// Gets the value that the sensor is currently sensing
    fn sense(&self) -> U;
}

struct MovementSensor {
    error_margin: usize,
    error_chance: f64,
    latest_movement: isize,
}

impl MovementSensor {
    fn new(error_margin: usize, error_chance: f64) -> Self {
        Self {
            error_margin,
            error_chance,
            latest_movement: 0,
        }
    }
}

impl Sensor<isize, isize> for MovementSensor {
    fn update(&mut self, input: isize) -> &Self {
        let mut rng = rand::thread_rng();
        self.latest_movement = input
            + if rng.gen_bool(self.error_chance) {
                rng.gen_range(0, self.error_margin + 1) as isize * rng.gen_range(-1, 1)
            } else {
                0
            };
        self
    }

    fn sense(&self) -> isize {
        self.latest_movement
    }
}

struct BinarySensor {
    breadth: isize,
    error_chance: f64,
    triggers: BitVec,
}

impl BinarySensor {
    fn new(breadth: usize, error_chance: f64) -> Self {
        Self {
            breadth: breadth as isize,
            error_chance,
            triggers: BitVec::with_capacity(1 + breadth * 2),
        }
    }
}

impl Sensor<&BinaryEnvironment, BitVec> for BinarySensor {
    fn update(&mut self, input: &BinaryEnvironment) -> &Self {
        self.triggers.clear();
        let mut rng = rand::thread_rng();
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
        self
    }

    fn sense(&self) -> BitVec {
        self.triggers.clone()
    }
}

/// A robot for a binary environment
///
/// Has `BinarySensor`, `MovementSensor`, and `BinaryEnvironment`.
/// The `BinaryEnvironment` is owned by the robot intentionally.
struct BinarySensingRobot {
    environment: BinaryEnvironment,
    binary_sensor: BinarySensor,
    movement_sensor: MovementSensor,
    ai: BinaryMCL,
}

impl BinarySensingRobot {
    fn make_move(&mut self, dist: isize) {
        self.movement_sensor
            .update(self.environment.move_robot(dist));
        self.binary_sensor.update(&self.environment);
    }
}

trait MCL<T, U> {
    fn motion_position_update(&mut self, sensor_data: T);
    fn sensor_weight_update(&mut self, sensor_update: U);
    fn resample(&mut self);
    fn get_average_position(&self) -> T;
}

struct BinaryMCL {
    particles: Vec<(usize, f64)>,
    map: BitVec,
}

impl BinaryMCL {
    fn new(map: BitVec, particle_count: usize) -> Self {
        let mut rng = rand::thread_rng();
        let mut particles = Vec::with_capacity(particle_count);
        for _ in 0..particle_count {
            particles.push((rng.gen_range(0, map.len()), 1.));
        }
        BinaryMCL { particles, map }
    }

    fn from_distribution<U>(map: BitVec, particle_count: usize, distribution: U) -> Self
    where
        U: rand::distributions::Distribution<f64>,
    {
        let mut rng = rand::thread_rng();
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
        let mut rng = rand::thread_rng();
        for _ in 0..self.particles.len() {
            new_particles.push(*particles.choose_weighted(&mut rng, |p| p.1).unwrap());
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

fn main() {
    let mut robot = {
        let env = BinaryEnvironment::new(3000, 0.5);
        let map = env.map.clone();
        BinarySensingRobot {
            environment: env,
            binary_sensor: BinarySensor::new(3, 0.005),
            movement_sensor: MovementSensor::new(3, 0.05),
            ai: BinaryMCL::new(map, 1000),
        }
    };
    let mut rng = thread_rng();
    let mut step_count = 0;
    let mut start_time: SystemTime;
    loop {
        start_time = SystemTime::now();
        step_count += 1;
        robot.make_move(rng.gen_range(
            -(robot.environment.size as isize) / 10,
            robot.environment.size as isize / 10,
        ));
        robot
            .ai
            .motion_position_update(robot.movement_sensor.sense());
        robot.ai.sensor_weight_update(robot.binary_sensor.sense());
        robot.ai.resample();
        let pose = robot.ai.get_average_position();
        let true_pose = robot.environment.robot_position;
        let diff = (pose as isize - true_pose as isize).abs();
        println!(
            "{} - Percieved: {}, Real: {}, Disconnect: {}, Time Taken: {:?}",
            step_count, pose, true_pose, diff, SystemTime::now().duration_since(start_time)
        );
        // println!("{}", robot.environment);
        // let mut pose_rep = " ".repeat(pose);
        // pose_rep.insert(pose, '🤖');
        // println!("{}", pose_rep);

        if diff == 0 {
            break;
        }
    }
}
