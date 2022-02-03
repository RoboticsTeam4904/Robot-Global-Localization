use crate::sensors::{LimitedSensor, Sensor};
use std::io::Write;

pub struct LoggingSensor<S, O, T, M>
where
    S: Sensor<Output = T>,
    O: Write,
    M: Fn(T) -> String,
    T: Clone,
{
    internal_sensor: S,
    latest_data: T,
    output: O,
    to_string: M,
}

impl<S, O, T, M> LoggingSensor<S, O, T, M>
where
    S: Sensor<Output = T>,
    O: Write,
    M: Fn(T) -> String,
    T: Clone,
{
    pub fn new(internal_sensor: S, output: O, to_string: M) -> Self {
        let latest_data = internal_sensor.sense();
        Self {
            internal_sensor,
            latest_data,
            output,
            to_string,
        }
    }
}

impl<'a, S, O, T, M> Sensor for LoggingSensor<S, O, T, M>
where
    S: Sensor<Output = T>,
    O: Write,
    M: Fn(T) -> String,
    T: Clone,
{
    type Output = T;

    fn update(&mut self) {
        self.internal_sensor.update();

        self.latest_data = self.internal_sensor.sense();
        self.output
            .write_all((self.to_string)(self.latest_data.clone()).as_bytes())
            .unwrap();
    }

    fn sense(&self) -> Self::Output {
        self.latest_data.clone()
    }
}

impl<'a, S, O, T, M, R> LimitedSensor<R> for LoggingSensor<S, O, T, M>
where
    S: Sensor<Output = T> + LimitedSensor<R>,
    O: Write,
    M: Fn(T) -> String,
    T: Clone,
{
    fn range(&self) -> Option<R> {
        self.internal_sensor.range()
    }
}
