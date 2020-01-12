pub mod map;
pub mod sensors;
pub mod ai;

/// The generic robot trait
pub trait Robot {
    fn run(&mut self);
}

/// The generic trait for any actuator
///
/// `T` is the input for the actuator
pub trait Actuator<T> {
    fn set(&mut self, value: T);
}
