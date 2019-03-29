extern crate bitvec;
extern crate rand;

mod utility;
mod simulation;

use rand::prelude::*;
use std::time::SystemTime;
use simulation::environment::BinaryEnvironment;
use simulation::sensor::{Sensor, BinarySensor, MovementSensor};
use simulation::robot::BinarySensingRobot;
use simulation::ai::{MCL, BinaryMCL};

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
