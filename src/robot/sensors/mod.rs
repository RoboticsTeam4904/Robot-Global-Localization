use crate::utility::Pose;

// pub mod gpio;
pub mod dummy;

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
    fn relative_pose(&self) -> Pose {
        Pose::default()
    }
}

/// General trait for sensors that can have limitations (e.g. a distance sensor has a maximum range)
pub trait LimitedSensor<T, U>: Sensor<U> {
    /// Returns the maximum range of this sensor.
    /// Returns `None` by default if the sensor has no maximum.
    fn range(&self) -> Option<T> {
        None
    }
}
