use super::{Sensor, SensorSink};
use rmp_serde::{self, Serializer};
use serde::{de::DeserializeOwned, Serialize};
use std::fs::File;
use std::io::{BufRead, Read, Write};

pub struct LogSensor<T: DeserializeOwned, R: Read> {
    pub reader: R,
    pub latest_data: T,
}

impl<T: DeserializeOwned, R: Read> LogSensor<T, R> {
    pub fn new(mut reader: R) -> Self {
        let initial_data = rmp_serde::decode::from_read::<&mut R, T>(&mut reader)
            .expect("No deserializable data found in logging file");
        LogSensor {
            reader,
            latest_data: initial_data,
        }
    }
}

impl<T: DeserializeOwned> LogSensor<T, File> {
    pub fn new_from_file(filename: &str) -> Self {
        let f = File::open(filename).expect(&format!("No file found with filename {}.", filename));
        Self::new(f)
    }
}

impl<T: DeserializeOwned + Clone, R: Read> Sensor for LogSensor<T, R> {
    type Output = T;
    fn update(&mut self) {
        let latest_data = rmp_serde::decode::from_read::<&mut R, T>(&mut self.reader)
            .expect("Error: all data read from logging file.");
        self.latest_data = latest_data;
    }

    fn sense(&self) -> Self::Output {
        self.latest_data.clone()
    }
}

pub struct LogSensorSink<T: Serialize, W: Write> {
    pub writer: W,
    pub latest_data: T,
}

impl<T: Serialize + Default, W: Write> LogSensorSink<T, W> {
    pub fn new(writer: W) -> Self {
        LogSensorSink {
            writer,
            latest_data: T::default(),
        }
    }
}

impl<T: Serialize + Default> LogSensorSink<T, File> {
    pub fn new_from_file(filename: &str) -> Self {
        let f = File::open(filename).expect(&format!("No file found with filename {}.", filename));
        Self::new(f)
    }
}

impl<T: Serialize + Clone, W: Write> SensorSink for LogSensorSink<T, W> {
    type Input = T;
    fn update_sink(&mut self) {
        let mut send_buf = Vec::new();
        self.latest_data
            .serialize(&mut Serializer::new(&mut send_buf))
            .expect("Serialization of sensor data failed.");
        self.writer
            .write(&send_buf)
            .expect("Writing to file failed.");
    }

    fn push(&mut self, input: Self::Input) {
        self.latest_data = input;
    }
}
