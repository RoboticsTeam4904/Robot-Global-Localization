#![allow(dead_code)] // TODO: This will be removed after organizational overhall and a lib.rs
mod robot;
mod utility;

use robot::ai::localization::{DistanceFinderMCL, ObjectDetectorMCL};
use robot::map::{Map2D, Object2D};
use robot::sensors::dummy::{DummyDistanceSensor, DummyPositionSensor, DummyObjectSensor};
use std::f64::consts::{FRAC_PI_8, PI};
use std::sync::Arc;
use utility::{Point, Pose};
use piston_window::*;

// TODO: Multithread everything

struct DistanceSensorRobot {
    mcl: DistanceFinderMCL,
    motion_sensor: DummyPositionSensor,
    distance_sensors: Vec<DummyDistanceSensor>,
}

impl DistanceSensorRobot {
    fn repeat(&mut self) {
        self.mcl.control_update(&self.motion_sensor); // TODO: dummy motion sensor and its usage are currently oversimplified
        self.mcl.observation_update(&self.distance_sensors);
    }
}

struct ObjectSensorRobot {
    mcl: ObjectDetectorMCL,
    motion_sensor: DummyPositionSensor,
    object_sensor: DummyObjectSensor,
}

impl ObjectSensorRobot {
    fn repeat(&mut self) {
        self.mcl.control_update(&self.motion_sensor); // TODO: dummy motion sensor and its usage are currently oversimplified
        self.mcl.observation_update(&self.object_sensor);
    }
}

fn main() {
    let mut tick = 0;
    // Make a robot
    let mut robot = {
        let map = Arc::new(Map2D::new(
            10.,
            10.,
            vec![
                Object2D::Point((213.57, 17.93).into()),
                Object2D::Point((228.28, 27.44).into()),
                Object2D::Point((242.99, 17.93).into()),
                Object2D::Point((259.8, 133.13).into()),
                Object2D::Point((259.8, 133.13).into()),
                Object2D::Point((281.55, 133.13).into()),
                Object2D::Point((-303.3, 133.13).into()),
                Object2D::Point((220.25, 151.13).into()),
                Object2D::Point((0., 25.72).into()),
                Object2D::Line(((220.25, 133.13).into(), (323.81, 133.13).into())),
                Object2D::Line(((220.25, 133.13).into(), (220.25, 188.13).into())),
                Object2D::Line(((218.56, 27.44).into(), (238., 27.44).into())),
                Object2D::Line(((218.56, 27.44).into(), (207.75, 7.82).into())),
                Object2D::Line(((207.75, 7.82).into(), (248.7, 7.82).into())),
                Object2D::Line(((248.7, 7.82).into(), (238., 27.44).into())),
            ].iter().map(|o| match o { // temp
                Object2D::Point(p) => Object2D::Point(*p / 50.),
                Object2D::Line((p1, p2)) => Object2D::Line((*p1 / 50., *p2 / 50.)),
                Object2D::Rectangle(r) => Object2D::Rectangle(*r),
                Object2D::Triangle(t) => Object2D::Triangle(*t),
            }),
        ));
        let starting_robot_pose = Pose {
            angle: PI,
            position: Point { x: 8., y: 8. },
        };
        let object_sensor = DummyObjectSensor::new(
            2. * PI,
            map.clone(),
            Pose::default(),
            starting_robot_pose,
            Point { x: 0.05, y: 0.05 },
        );
        ObjectSensorRobot {
            mcl: ObjectDetectorMCL::new(
                20_000,
                map.clone(),
                Box::new(|error| 1.05f64.powf(-error)),
                Pose {
                    angle: FRAC_PI_8 / 4.,
                    position: Point { x: 0.05, y: 0.05 },
                },
            ),
            object_sensor,
            motion_sensor: DummyPositionSensor::new(
                starting_robot_pose,
                Pose {
                    angle: FRAC_PI_8 / 4.,
                    position: Point { x: 0.03, y: 0.03 },
                },
            ),
        }
    };
    // Setup visuals
    let mut window: PistonWindow = 
        WindowSettings::new("Akshar", [750, 750])
        .exit_on_esc(true).build().unwrap();
    // Start up the window
    while let Some(e) = window.next() {
        robot.repeat();
        
        tick += 1;
    }
}
