use crate::utility::Pose;
use std::ops::Sub;

// pub mod gpio;
pub mod dummy;
pub mod network;
pub mod rplidar;

/// The generic trait for any sensor.
/// Only `sense` is required, but relative pose is highly recommended.
///
/// `Output` is the output of the sensor (percieved info from the environment)
pub trait Sensor {
    type Output;

    /// Update the sensor
    fn update(&mut self) {}
    /// Gets the value that the sensor is currently sensing
    fn sense(&self) -> Self::Output;
    /// Get the pose of the sensor relative to the pose of the robot
    fn relative_pose(&self) -> Pose {
        Pose::default()
    }
}

/// General trait for sensors that can have limitations (e.g. a distance sensor has a maximum range)
pub trait LimitedSensor<T>: Sensor {
    /// Returns the maximum range of this sensor.
    /// Returns `None` by default if the sensor has no maximum.
    fn range(&self) -> Option<T> {
        None
    }
}

/// Helper trait for providing easy usage of wrapped sensors.
pub trait WrappableSensor: Sensor + Sized {
    /// Overrides range of `self` to be `range`
    fn override_limit<L: Clone>(self, range: Option<L>) -> OverridenLimitedSensor<Self, L> {
        OverridenLimitedSensor::new(self, range)
    }
    /// Maps `self` to a `Sensor<Output = M>`
    /// by calling `map` on `self.sense` in the returned sensor's `sense` function.
    fn map<M, O>(self, map: M) -> MappedSensor<Self, <Self as Sensor>::Output, M, O>
    where
        M: Fn(<Self as Sensor>::Output) -> O,
    {
        MappedSensor::new(self, map)
    }
}

impl<S: Sensor + Sized> WrappableSensor for S {}

/// A wrapper sensor that applies that applies the the `map`
/// function to the output of `internal_sensor.sense()` in `MappedSensor::sense`.
///
/// The rest of the implementation of `Sense` and `LimitedSensor` is reflected upward
/// from `internal_sensor`.
pub struct MappedSensor<S, O, Map, MappedOut>
where
    S: Sensor<Output = O>,
    Map: Fn(O) -> MappedOut,
{
    pub internal_sensor: S,
    pub map: Map,
}

impl<S, O, Map, MappedOut> MappedSensor<S, O, Map, MappedOut>
where
    S: Sensor<Output = O>,
    Map: Fn(O) -> MappedOut,
{
    /// Creates a new `MappedSensor` around `internal_sensor` using `map`.
    pub fn new(internal_sensor: S, map: Map) -> Self {
        Self {
            internal_sensor,
            map,
        }
    }
}

impl<S, O, Map, MappedOut> Sensor for MappedSensor<S, O, Map, MappedOut>
where
    S: Sensor<Output = O>,
    Map: Fn(O) -> MappedOut,
{
    type Output = MappedOut;

    fn update(&mut self) {
        self.internal_sensor.update()
    }

    fn sense(&self) -> Self::Output {
        (self.map)(self.internal_sensor.sense())
    }

    fn relative_pose(&self) -> Pose {
        self.internal_sensor.relative_pose()
    }
}

impl<S, O, Map, MappedOut, R> LimitedSensor<R> for MappedSensor<S, O, Map, MappedOut>
where
    S: Sensor<Output = O> + LimitedSensor<R>,
    Map: Fn(O) -> MappedOut,
{
    fn range(&self) -> Option<R> {
        self.internal_sensor.range()
    }
}

/// A wrapper sensor that imposes `limit` as the sensors `LimitedSensor::range`
/// output rather than what it had previously.
///
/// Everything except for the `range` is reflected from the `internal_sensor`.
pub struct OverridenLimitedSensor<S, L>
where
    S: Sensor,
    L: Clone,
{
    internal_sensor: S,
    range: Option<L>,
}

impl<S, L> OverridenLimitedSensor<S, L>
where
    S: Sensor,
    L: Clone,
{
    pub fn new(internal_sensor: S, range: Option<L>) -> Self {
        Self {
            internal_sensor,
            range,
        }
    }
}

impl<S, L> Sensor for OverridenLimitedSensor<S, L>
where
    S: Sensor,
    L: Clone,
{
    type Output = <S as Sensor>::Output;

    fn update(&mut self) {
        self.internal_sensor.update();
    }

    fn sense(&self) -> Self::Output {
        self.internal_sensor.sense()
    }

    fn relative_pose(&self) -> Pose {
        self.internal_sensor.relative_pose()
    }
}

impl<S, L> LimitedSensor<L> for OverridenLimitedSensor<S, L>
where
    S: Sensor,
    L: Clone,
{
    fn range(&self) -> Option<L> {
        self.range.clone()
    }
}

/// A wrapper sensor that senses the difference in `absolute_sensor`'s outputs.
/// The difference is set in `update`, so updating multiple times between each call of `sense` could result in lost data.
pub struct DeltaSensor<S, O>
where
    S: Sensor<Output = O>,
    O: Sub + Clone,
{
    pub absolute_sensor: S,
    last: O,
    current: O,
}

impl<S, O> DeltaSensor<S, O>
where
    S: Sensor<Output = O>,
    O: Sub + Clone,
{
    pub fn new(absolute_sensor: S) -> Self {
        let sense_data = absolute_sensor.sense();
        Self {
            absolute_sensor,
            last: sense_data.clone(),
            current: sense_data,
        }
    }
}

impl<S, O, U> Sensor for DeltaSensor<S, O>
where
    S: Sensor<Output = O>,
    O: Sub<Output = U> + Clone,
{
    type Output = U;

    fn update(&mut self) {
        self.absolute_sensor.update();
        self.last = self.current.clone();
        self.current = self.absolute_sensor.sense();
    }

    fn sense(&self) -> Self::Output {
        self.current.clone() - self.last.clone()
    }

    fn relative_pose(&self) -> Pose {
        self.absolute_sensor.relative_pose()
    }
}

impl<S, O, R> LimitedSensor<R> for DeltaSensor<S, O>
where
    S: Sensor<Output = O> + LimitedSensor<R>,
    O: Sub + Clone,
{
    fn range(&self) -> Option<R> {
        self.absolute_sensor.range()
    }
}
