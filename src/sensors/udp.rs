use crate::{
    sensors::{
        log::{LogSensor, LogSensorSink},
        Sensor, SensorSink,
    },
    utility::{Point, Pose},
};
use failure::bail;
use rmp_serde::{self, Serializer};
use serde::{de::DeserializeOwned, Serialize};
use std::fmt::Debug;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::thread;

/// General UDP Sensor has as output some Deserializable type
pub struct UDPSensor<T: DeserializeOwned> {
    pub socket: UdpSocket,
    pub latest_data: T,
    pub timestamp: u64,
}

impl<T: DeserializeOwned> UDPSensor<T> {
    pub fn new(socket: UdpSocket) -> Result<Self, failure::Error>
    where
        T: Debug,
    {
        socket.set_read_timeout(None)?;
        socket.set_nonblocking(false)?;

        let mut buf = [0; 1024];
        let received = socket.recv(&mut buf)?;

        socket.set_nonblocking(true)?;
        let deserialized: Result<(T, u64), rmp_serde::decode::Error> =
            rmp_serde::from_slice(&buf[..received]);

        match deserialized {
            Ok(deserial) => Ok(UDPSensor {
                socket,
                latest_data: deserial.0,
                timestamp: deserial.1,
            }),
            _ => {
                bail!(deserialized.unwrap_err())
            }
        }
    }

    pub fn new_with_port(port: u16) -> Result<Self, failure::Error>
    where
        T: Debug + Default,
    {
        let socket = UdpSocket::bind(SocketAddr::new(IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0)), port))?;
        Self::new(socket)
    }
}

impl<T: DeserializeOwned + Clone> Sensor for UDPSensor<T> {
    type Output = T;

    fn update(&mut self) {
        let mut buf = [0; 1024];
        loop {
            let received = self.socket.recv(&mut buf);
            match received {
                Ok(received) => {
                    let deserialized: Result<(T, u64), rmp_serde::decode::Error> =
                        rmp_serde::from_slice(&buf[..received]);
                    if let Ok(de) = deserialized {
                        if de.1 > self.timestamp {
                            self.latest_data = de.0;
                            self.timestamp = de.1;
                        }
                    }
                }
                _ => {
                    break;
                }
            }
        }
    }

    fn sense(&self) -> Self::Output {
        self.latest_data.clone()
    }
}

/// General UDP sensor sink for some serializable type
pub struct UDPSensorSink<T: Serialize> {
    pub socket: UdpSocket,
    pub dst: String,
    pub latest_data: T,
    pub sending: bool,
}

impl<T: Serialize> UDPSensorSink<T> {
    pub fn new(socket: UdpSocket, dst: String) -> Result<Self, failure::Error>
    where
        T: Debug + Default,
    {
        socket.set_read_timeout(None)?;

        socket.set_nonblocking(true)?;

        Ok(UDPSensorSink {
            socket,
            dst,
            latest_data: T::default(),
            sending: true,
        })
    }

    pub fn new_with_port(port: u16, dst: String) -> Result<Self, failure::Error>
    where
        T: Debug + Default,
    {
        let socket = UdpSocket::bind(SocketAddr::new(IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0)), port))?;
        Self::new(socket, dst)
    }
}

impl<T: Serialize + Copy> SensorSink for UDPSensorSink<T> {
    type Input = T;

    fn update_sink(&mut self) {
        let mut send_buf = Vec::new();
        self.latest_data
            .serialize(&mut Serializer::new(&mut send_buf))
            .expect("Serialization of sensor data failed.");
        self.sending = self.socket.send_to(&send_buf, self.dst.clone()).is_ok();
    }

    fn push(&mut self, input: Self::Input) {
        self.latest_data = input;
    }
}
