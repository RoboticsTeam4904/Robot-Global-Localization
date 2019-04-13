pub mod simulation;

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
