use super::sensor::Sensor;
use super::ai::BinaryMCL;
use super::environment::BinaryEnvironment;
use super::sensor::{BinarySensor, MovementSensor};

/// A robot for a binary environment
///
/// Has `BinarySensor`, `MovementSensor`, and `BinaryEnvironment`.
/// The `BinaryEnvironment` is owned by the robot intentionally.
pub struct BinarySensingRobot {
    pub environment: BinaryEnvironment,
    pub binary_sensor: BinarySensor,
    pub movement_sensor: MovementSensor,
    pub ai: BinaryMCL,
}

impl BinarySensingRobot {
    pub fn make_move(&mut self, dist: isize) {
        self.environment.move_robot(dist);
        self.movement_sensor.update(&self.environment);
        self.binary_sensor.update(&self.environment);
    }
}
