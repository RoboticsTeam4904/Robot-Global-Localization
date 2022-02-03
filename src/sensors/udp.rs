use crate::{
    sensors::{Sensor, SensorSink},
    utility::{Point, Pose},
};
use failure::bail;
use rmp_serde::{self, Serializer};
use serde::{de::DeserializeOwned, Serialize};
use std::fmt::Debug;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};

/// General UDP Sensor has as output some Deserializable type
pub struct UDPSensor<T: DeserializeOwned> {
    pub socket: UdpSocket,
    pub latest_data: T,
}

impl<T: DeserializeOwned> UDPSensor<T> {
    pub fn new(port: u16, dst: SocketAddr) -> Result<Self, failure::Error>
    where
        T: Debug,
    {
        let socket = UdpSocket::bind(SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)),
            port,
        ))?;
        socket.connect(&dst)?;
        socket.set_read_timeout(None)?;
        socket.set_nonblocking(false)?;

        let mut buf = [0; 512];
        socket.recv(&mut buf)?;

        socket.set_nonblocking(true)?;
        let deserialized: Result<T, rmp_serde::decode::Error> = rmp_serde::from_slice(&buf);

        match deserialized {
            Ok(deserial) => Ok(UDPSensor {
                socket,
                latest_data: deserial,
            }),
            _ => {
                bail!(deserialized.unwrap_err())
            }
        }
    }
}

impl<T: DeserializeOwned + Copy> Sensor for UDPSensor<T> {
    type Output = T;

    fn update(&mut self) {
        loop {
            let mut buf = [0; 512];
            let received = self.socket.recv(&mut buf);
            match received {
                Ok(_) => {
                    let deserialized: Result<T, rmp_serde::decode::Error> =
                        rmp_serde::from_slice(&buf);
                    if deserialized.is_ok() {
                        self.latest_data = deserialized.unwrap();
                    }
                }
                _ => {
                    break;
                }
            }
        }
    }

    fn sense(&self) -> Self::Output {
        self.latest_data
    }
}

/// General UDP sensor sink for some serializable type
pub struct UDPSensorSink<T: Serialize> {
    pub socket: UdpSocket,
    pub latest_data: T,
}

impl<T: Serialize> UDPSensorSink<T> {
    pub fn new(port: u16, dst: SocketAddr) -> Result<Self, failure::Error>
    where
        T: Debug + Default,
    {
        let socket = UdpSocket::bind(SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)),
            port,
        ))?;
        socket.connect(&dst)?;
        socket.set_read_timeout(None)?;

        socket.set_nonblocking(true)?;

        Ok(UDPSensorSink {
            socket,
            latest_data: T::default(),
        })
    }
}

impl<T: Serialize + Copy> SensorSink for UDPSensorSink<T> {
    type Input = T;

    fn update_sink(&mut self) {
        let mut send_buf = Vec::new();
        let _ = self
            .latest_data
            .serialize(&mut Serializer::new(&mut send_buf));
        let _ = self.socket.send(&send_buf);
    }

    fn push(&mut self, input: Self::Input) {
        self.latest_data = input;
    }
}
