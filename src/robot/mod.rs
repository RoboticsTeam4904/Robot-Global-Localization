use crate::utility::Pose;

pub mod map;
pub mod sensors;
pub mod AI;
pub mod simulation; // TODO

/// The generic robot trait
pub trait Robot {
    fn run(&mut self);
}

/// The generic trait for any sensor.
/// Only `sense` is required.
///
/// `T` is the output of the sensor (percieved info from the environment)
pub trait Sensor<T> {
    /// Update the sensor
    fn update(&mut self) {}
    /// Gets the value that the sensor is currently sensing
    fn sense(&self) -> T;
    /// Get the pose of the sensor relative to the pose of the robot
    fn get_relative_pose(&self) -> Pose {
        Pose::default()
    }
}

/// The generic trait for any actuator
///
/// `T` is the input for the actuator
pub trait Actuator<T> {
    fn set(&mut self, value: T);
}
