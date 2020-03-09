pub mod ai;
pub mod map;
#[cfg(feature = "network")]
pub mod networktables;
pub mod replay;
pub mod sensors;
pub mod utility;

#[cfg(test)]
mod tests {
    #[test]
    fn test_dummy_sensor() {
        use super::sensors::{dummy::DummySensor, *};
        let mut dummy = DummySensor::new("Hello");
        assert_eq!("Hello", dummy.sense());
        dummy.push("Hi");
        assert_eq!("Hi", dummy.sense());
        dummy.push("Why are you copying me?");
        assert_eq!("Why are you copying me?", dummy.sense());
        dummy.push("Echo!");
        assert_eq!("Echo!", dummy.sense());
    }

    #[test]
    fn test_mapped_sensor() {
        use super::sensors::{dummy::DummySensor, *};
        let mut mapped = DummySensor::new(9).map(|i: i32| 2 * i);
        assert_eq!(18, mapped.sense());
        mapped.push(7);
        assert_eq!(14, mapped.sense());
    }

    #[test]
    fn test_many_layered_sensor() {
        use super::{
            sensors::{dummy::DummySensor, *},
            utility::*,
        };
        let mut wrapped = DummySensor::new(1)
            .map(|i: i32| 2 * i)
            .override_limit(Some(3))
            .map_relative_pose(|_| Pose {
                position: (3., 3.).into(),
                ..Pose::default()
            })
            .map(|i: i32| 4 + i)
            .map_relative_pose(|pose| Pose {
                angle: pose.angle,
                position: pose.position * 3.,
            })
            .map_sink(|i: i32| (i - 4) / 2)
            .map_whole(|sensor| sensor.sense());
        wrapped.update();
        assert_eq!(6, wrapped.sense());
        assert_eq!(Some(3), wrapped.range());
        assert_eq!(
            Pose {
                angle: 0.,
                position: (9., 9.).into()
            },
            wrapped.relative_pose()
        );
        wrapped.push(10);
        wrapped.update_sink();
        wrapped.update();
        assert_eq!(10, wrapped.sense());
    }

    #[test]
    fn test_file_sensor_sink() {
        use super::sensors::{io::IOSensor, *};
        use std::fs::{copy, remove_file, OpenOptions};
        const PATH: &'static str = "test_resources/hello_mut.txt";
        const RESET_PATH: &'static str = "test_resources/hello.txt";
        remove_file(PATH).unwrap();
        copy(RESET_PATH, PATH).unwrap();
        {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .open(PATH)
                .unwrap();
            let mut sensor_sink = IOSensor::new(file)
                .map(|data: Vec<u8>| String::from_utf8(data).unwrap())
                .map_sink(|data: String| data.as_bytes().to_vec());
            sensor_sink.update();
            sensor_sink.update_sink();
            assert_eq!("hello".to_owned(), sensor_sink.sense());
            sensor_sink.push("hi".to_owned());
        }

        {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .open(PATH)
                .unwrap();
            let mut sensor_sink = IOSensor::new(file)
                .map(|data: Vec<u8>| String::from_utf8(data).unwrap())
                .map_sink(|data: String| data.as_bytes().to_vec());
            sensor_sink.update();
            sensor_sink.update_sink();
            assert_eq!("hellohi".to_owned(), sensor_sink.sense());
        }
        remove_file(PATH).unwrap();
        copy(RESET_PATH, PATH).unwrap();
    }

    #[test]
    fn test_map_creation() {
        use super::{map::*, utility::*};
        let map = Map2D::new(vec![
            Object2D::Rectangle(((0., 0.).into(), (1., 1.).into())),
            Object2D::Triangle(((0., 0.).into(), (1., 1.).into(), (1., 0.).into())),
            Object2D::Line(((0., 0.).into(), (1., 1.).into())),
        ]);
        assert_eq!(map.size, Point { x: 1., y: 1. });
        assert_eq!(map.vertices.len(), 4);
    }

    #[test]
    fn test_map_raycast() {
        use super::{map::*, utility::*};
        use std::f64::consts::*;
        let map = Map2D::new(vec![
            Object2D::Rectangle(((0., 0.).into(), (1., 1.).into())),
            Object2D::Triangle(((0., 0.).into(), (1., 1.).into(), (1., 0.).into())),
            Object2D::Line(((0., 0.).into(), (1., 1.).into())),
        ]);
        assert_eq!(
            map.raycast(Pose {
                angle: 0.,
                position: (0.1, 0.5).into()
            }),
            Some((0.5, 0.5).into())
        );
        assert_eq!(
            map.raycast(Pose {
                angle: FRAC_PI_2,
                position: (0.5, 0.24324).into()
            }),
            Some((0.5, 0.5).into())
        );
        let pred = map
            .raycast(Pose {
                angle: FRAC_PI_4,
                position: (0.34, 0.827).into(),
            })
            .expect("Failed to intersect");
        let actual: Point = (0.513, 1.).into();
        assert_eq!(actual.y, pred.y);
        assert!((actual.x - pred.x).abs() <= 0.05);
    }

    #[cfg(feature = "asyncio")]
    #[test]
    fn test_mincodec_tcp_sensor_sink() {
        use super::sensors::{io::AsyncIOSensor, *};
        use async_std::net::{TcpListener, TcpStream};
        use core_futures_io::FuturesCompat;
        use futures::{executor::block_on, io::AsyncReadExt, StreamExt};
        use std::{
            thread::{sleep, spawn},
            time::Duration,
        };
        // Setup the server thread (for testing only)
        spawn(|| {
            block_on(async {
                let listener = TcpListener::bind("127.0.0.1:6969")
                    .await
                    .expect("could not bind listener");
                let mut incoming = listener.incoming();
                if let Some(stream) = incoming.next().await {
                    let stream = stream.expect("server died");
                    // echo back whatever the server recieved
                    let (reader, writer) = &mut (&stream, &stream);
                    async_std::io::copy(reader, writer)
                        .await
                        .expect("Faield to echo");
                }
            })
        });
        // Test the client serversink
        block_on(async {
            let stream = TcpStream::connect("127.0.0.1:6969")
                .await
                .expect("could not connect");
            let (i, o) = stream.split();
            let mut tcp_sensor_sink = AsyncIOSensor::with_starting_data(
                FuturesCompat::new(i),
                FuturesCompat::new(o),
                "".to_string(),
                "".to_string(),
            );

            let one = "Echo!".to_string();
            tcp_sensor_sink.push(one.clone());
            sleep(Duration::from_millis(50));
            assert_eq!(tcp_sensor_sink.sense(), one);

            let two = "Can you hear me".to_string();
            tcp_sensor_sink.push(two.clone());
            sleep(Duration::from_millis(50));
            assert_eq!(tcp_sensor_sink.sense(), two);

            let three = "Yes I can".to_string();
            let four = "😎 😎 😎 😎 😎 ".to_string();
            tcp_sensor_sink.push(three);
            tcp_sensor_sink.push(four.clone());
            sleep(Duration::from_millis(50));
            assert_eq!(tcp_sensor_sink.sense(), four);
        });
    }
}
