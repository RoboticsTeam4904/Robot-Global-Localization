use super::{Sensor, SensorSink};
use rmp_serde::{self, Serializer};
use serde::{de::DeserializeOwned, Serialize};
use std::fs::File;
use std::io::{BufRead, Read, Write};

pub struct LogUDPSensor<T: DeserializeOwned, R: Read> {
    pub reader: R,
    pub latest_data: T,
    pub timestamp: f64,
}

impl<T: DeserializeOwned, R: Read> LogUDPSensor<T, R> {
    pub fn new(mut reader: R) -> Self {
        let (initial_data, timestamp) =
            rmp_serde::decode::from_read::<&mut R, (T, f64)>(&mut reader)
                .expect("No deserializable data found in logging file");
        LogUDPSensor {
            reader,
            latest_data: initial_data,
            timestamp,
        }
    }
}

impl<T: DeserializeOwned> LogUDPSensor<T, File> {
    pub fn new_from_file(filename: &str) -> Self {
        let f = File::open(filename).expect(&format!("No file found with filename {}.", filename));
        Self::new(f)
    }
}

impl<T: DeserializeOwned + Clone, R: Read> Sensor for LogUDPSensor<T, R> {
    type Output = T;
    fn update(&mut self) {
        let (latest_data, timestamp) =
            rmp_serde::decode::from_read::<&mut R, (T, f64)>(&mut self.reader)
                .expect("Error: all data read from logging file.");
        self.latest_data = latest_data;
        self.timestamp = timestamp;
    }

    fn sense(&self) -> Self::Output {
        self.latest_data.clone()
    }
}

pub struct LogUDPSensorSink<T: Serialize, W: Write> {
    pub writer: W,
    pub latest_data: T,
    pub timestamp: f64,
}

impl<T: Serialize + Default, W: Write> LogUDPSensorSink<T, W> {
    pub fn new(writer: W) -> Self {
        LogUDPSensorSink {
            writer,
            latest_data: T::default(),
            timestamp: 0.,
        }
    }
}

impl<T: Serialize + Default> LogUDPSensorSink<T, File> {
    pub fn new_from_file(filename: &str) -> Self {
        let f = File::open(filename).expect(&format!("No file found with filename {}.", filename));
        Self::new(f)
    }
}

impl<T: Serialize, W: Write> SensorSink for LogUDPSensorSink<T, W> {
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
