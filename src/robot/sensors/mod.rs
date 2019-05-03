use crate::{utility::Pose, Sensor};

// pub mod gpio;
pub mod dummy;

pub trait DistanceSensor<T>: Sensor<T> {
    /// Get the pose of the sensor relative to the pose of the robot
    fn get_relative_pose(&self) -> Pose;
}