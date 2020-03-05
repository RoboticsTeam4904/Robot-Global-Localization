pub mod ai;
pub mod map;
#[cfg(feature = "network")]
pub mod networktables;
pub mod replay;
pub mod sensors;
pub mod utility;

#[cfg(test)]
mod tests {
    use super::{
        map::*,
        sensors::{dummy::DummySensor, io::IOSensor, *},
        utility::{Point, Pose},
    };
    use std::{
        f64::consts::*,
        fs::{copy, remove_file, OpenOptions},
    };

    #[test]
    fn test_dummy_sensor() {
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
        let mut mapped = DummySensor::new(9).map(|i: i32| 2 * i);
        assert_eq!(18, mapped.sense());
        mapped.push(7);
        assert_eq!(14, mapped.sense());
    }

    #[test]
    fn test_many_layered_sensor() {
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
        let pred = map.raycast(Pose {
                angle: FRAC_PI_4,
                position: (0.34, 0.827).into()
        }).expect("Failed to intersect");
        let actual: Point = (0.513, 1.).into();
        assert_eq!(actual.y, pred.y);
        assert!((actual.x - pred.x).abs() <= 0.05);
    }
}
