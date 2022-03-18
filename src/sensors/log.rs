use super::{Sensor, SensorSink};
use rmp_serde::{self, Serializer};
use serde::{de::DeserializeOwned, Serialize};
use std::cmp::PartialEq;
use std::fs::File;
use std::io::{BufRead, Read, Write};

pub struct LogSensor<T: DeserializeOwned, R: Read> {
    pub reader: R,
    pub latest_data: T,
    pub counter: usize,
}

impl<T: DeserializeOwned, R: Read> LogSensor<T, R> {
    pub fn new(mut reader: R) -> Self {
        let initial_data = rmp_serde::decode::from_read::<&mut R, T>(&mut reader)
            .expect("No deserializable data found in logging file");
        LogSensor {
            reader,
            latest_data: initial_data,
            counter: 0,
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
        if self.counter == 0 {
            let (latest_data, counter) =
                rmp_serde::decode::from_read::<&mut R, (T, usize)>(&mut self.reader)
                    .expect("Error: all data read from logging file.");
            self.latest_data = latest_data;
            self.counter = counter;
        } else {
            self.counter -= 1;
        }
    }

    fn sense(&self) -> Self::Output {
        self.latest_data.clone()
    }
}

pub struct LogSensorSink<T: Serialize, W: Write> {
    pub writer: W,
    pub latest_data: T,
    pub past_data: T,
    pub counter: usize,
}

impl<T: Serialize + Default + PartialEq, W: Write> LogSensorSink<T, W> {
    pub fn new(writer: W) -> Self {
        LogSensorSink {
            writer,
            latest_data: T::default(),
            past_data: T::default(),
            counter: 0,
        }
    }
}

impl<T: Serialize + Default + PartialEq> LogSensorSink<T, File> {
    pub fn new_from_file(filename: &str) -> Self {
        let f =
            File::create(filename).expect(&format!("No file found with filename {}.", filename));
        Self::new(f)
    }
}

impl<T: Serialize + Clone + PartialEq, W: Write> SensorSink for LogSensorSink<T, W> {
    type Input = T;
    fn update_sink(&mut self) {
        let mut send_buf = Vec::new();
        (self.past_data.clone(), self.counter + 1)
            .serialize(&mut Serializer::new(&mut send_buf))
            .expect("Serialization of sensor data failed.");
        self.writer
            .write(&send_buf)
            .expect("Writing to file failed.");
        self.counter = 0;
    }

    fn push(&mut self, input: Self::Input) {
        if self.latest_data == input {
            self.counter += 1;
        } else {
            self.past_data = self.latest_data.clone();
            self.latest_data = input;
        }
    }
}
