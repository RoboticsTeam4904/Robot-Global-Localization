use crate::utility::Pose;
use std::{marker::PhantomData, ops::Sub};

// pub mod gpio;
pub mod dummy;
pub mod io;
#[cfg(feature = "network")]
pub mod network;
#[cfg(feature = "rplidar")]
pub mod rplidar;

/// The generic trait for any sensor.
/// Only `sense` is required, but relative pose is highly recommended.
///
/// `Output` is the output of the sensor (percieved info from the environment).
///
/// By using the trait `WrappableSensor`, `map` and `override_limit` helper functions are provided
/// to map a sensor's `output` and impose a new limit on the sensor.
pub trait Sensor {
    type Output;

    /// Update the sensor (usually has side effects)
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

/// The general sink trait which emulates the "other" side of the sensor trait
/// wherein data is pushed rather than recieved.
///
/// `Input` is the input of the sensor.
///
/// By using the trait `WrappableSensorSink`, the helper functions `map_sink` and `zip`
/// are provided to map the input given to a sink before sinking it and reflect to the input
/// of the sink to multiple sinks.
pub trait SensorSink {
    type Input;

    /// Update the sensor-sink (similar to `Sensor::update` in that it does not take in data)
    fn update(&mut self) {}
    /// Push data to the sensor-sink (the inverse of `Sensor::sense`)
    fn push(&mut self, input: Self::Input);
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

/// Helper trait for providing easy usage of wrapped sensor-sinks.
pub trait WrappableSensorSink: SensorSink + Sized {
    /// Maps `self` to a `SensorSink<Input = M>`
    /// by applying `map` to inputs of `push`
    fn map_sink<M, I>(self, map: M) -> MappedSensorSink<Self, <Self as SensorSink>::Input, M, I>
    where
        M: Fn(I) -> <Self as SensorSink>::Input,
    {
        MappedSensorSink::new(self, map)
    }
    // Zips `self` with another `SensorSink` of the same input type.
    //
    // Whenevr a datum is pushed to `self`, it will also be cloned and pushed to `other` as well.
    fn zip<S, I>(self, other: S) -> ZippedSensorSink<Self, S, <Self as SensorSink>::Input>
    where
        S: SensorSink<Input = I>,
        Self: SensorSink<Input = I>,
        I: Clone,
    {
        ZippedSensorSink::new(self, other)
    }
}

impl<S: SensorSink + Sized> WrappableSensorSink for S {}

pub struct ZippedSensorSink<S1, S2, I>
where
    S1: SensorSink<Input = I>,
    S2: SensorSink<Input = I>,
{
    sink_1: S1,
    sink_2: S2,
}

impl<S1, S2, I> ZippedSensorSink<S1, S2, I>
where
    S1: SensorSink<Input = I>,
    S2: SensorSink<Input = I>,
{
    pub fn new(sink_1: S1, sink_2: S2) -> Self {
        Self { sink_1, sink_2 }
    }
}

impl<S1, S2, I> SensorSink for ZippedSensorSink<S1, S2, I>
where
    S1: SensorSink<Input = I>,
    S2: SensorSink<Input = I>,
    I: Clone,
{
    type Input = I;

    fn update(&mut self) {
        self.sink_1.update();
        self.sink_2.update();
    }

    fn push(&mut self, input: Self::Input) {
        self.sink_1.push(input.clone());
        self.sink_2.push(input);
    }
}

/// A wrapper sensor-sink that applies that applies the the `map`
/// function to the input of `internal_sensor_sink.push in `MappedSensorSink::push`.
///
/// The implementation of `update` is reflected upward from the internal sensor-sink.
pub struct MappedSensorSink<S, I, Map, MappedIn>
where
    S: SensorSink<Input = I>,
    Map: Fn(MappedIn) -> I,
{
    pub internal_sensor_sink: S,
    pub map: Map,
    mapped_input: PhantomData<MappedIn>,
}

impl<S, I, Map, MappedIn> MappedSensorSink<S, I, Map, MappedIn>
where
    S: SensorSink<Input = I>,
    Map: Fn(MappedIn) -> I,
{
    pub fn new(internal_sensor_sink: S, map: Map) -> Self {
        Self {
            internal_sensor_sink,
            map,
            mapped_input: PhantomData,
        }
    }
}

impl<S, I, Map, MappedIn> SensorSink for MappedSensorSink<S, I, Map, MappedIn>
where
    S: SensorSink<Input = I>,
    Map: Fn(MappedIn) -> I,
{
    type Input = MappedIn;

    fn update(&mut self) {
        self.internal_sensor_sink.update()
    }

    fn push(&mut self, input: Self::Input) {
        self.internal_sensor_sink.push((self.map)(input))
    }
}

/// A wrapper sensor that applies the `map`
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
