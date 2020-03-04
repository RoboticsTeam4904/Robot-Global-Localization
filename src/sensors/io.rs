use super::{Sensor, SensorSink};
use std::io::{Read, Write};

/// An IO Sensor is a simple wrapper of something which implements
/// `std::io::Read` and/or `std::io::Write`.
/// 
/// If the wrapped source implements `std::io::Read`, then `IOSensor` implements `Sensor<Output = Vec<u8>>`.
/// The output is attained by calling `read_to_end`.
/// 
/// If the wrapped source implements `std::io::Write`, then `IOSensor` implements `SensorSink<Input = Vec<u8>>`.
/// The input is pushed by calling `write_all`.
/// 
/// It is recomended to use this sensor in conjuction with the `map` and `override_limit`
/// methods to build a sensor which fully meets your needs.
/// 
/// Although an `IOSensor` can technically wrap any object, even if it does not implement read or write,
/// it is not recommended as it will not accomplish anything.
pub struct IOSensor<I> {
    io: I,
    latest_data: Vec<u8>,
}

impl<I> IOSensor<I> {
    pub fn new(io: I) -> Self {
        Self {
            io,
            latest_data: vec![],
        }
    }

    pub fn with_starting_data(io: I, starting_data: Vec<u8>) -> Self {
        Self {
            io,
            latest_data: starting_data
        }
    }
}

impl<I> Sensor for IOSensor<I>
    where I: Read {
        type Output = Vec<u8>;

        fn update(&mut self) {
            let mut buf = vec![];
            self.io.read_to_end(&mut buf).unwrap();
            self.latest_data = buf;
        }

        fn sense(&self) -> Self::Output {
            self.latest_data.clone()
        }
}

impl<O> SensorSink for IOSensor<O>
    where O: Write {
        type Input = Vec<u8>;

        fn push(&mut self, input: Self::Input) {
            self.io.write_all(&input).unwrap();
        }
}
