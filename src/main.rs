#![allow(dead_code)] // TODO: This will be removed after organizational overhall and a lib.rs
mod robot;
mod utility;

use nalgebra::{Matrix1, Matrix6, RowVector1, Vector1, Vector6};
use rand::distributions::{Distribution, Normal};
use rand::thread_rng;
use robot::ai::localization::{KalmanFilter, ObjectDetectorMCL};
use robot::map::{Map2D, Object2D};
use robot::sensors::dummy::{
    DummyAccelerationSensor, DummyDistanceSensor, DummyMotionSensor, DummyObjectSensor,
};
use robot::sensors::Sensor;
use std::f64::consts::{FRAC_PI_8, PI};
use std::sync::Arc;
use utility::{NewPose, Point, Pose};

fn main() {
    let time_scale = 1. / 1000.;
    let q: Matrix6<f64> = Matrix6::from_diagonal(&Vector6::new(
        0.,
        0.,
        0.,
        (0.1f64 * time_scale).powi(2),
        (0.2f64 * time_scale).powi(2),
        (0.2f64 * time_scale).powi(2),
    ));
    let r: Matrix1<f64> = Matrix1::from_vec(vec![9.]);
    let mut rng = thread_rng();
    let noise = Normal::new(0., 3.);
    let map = Arc::new(Map2D::new(
        10.,
        10.,
        vec![
            Object2D::Point((0., 200.).into()),
            Object2D::Point((200., 200.).into()),
            Object2D::Point((200., 0.).into()),
            Object2D::Point((0., 0.).into()),
            Object2D::Line(((0., 0.).into(), (200., 0.).into())),
            Object2D::Line(((0., 0.).into(), (0., 200.).into())),
            Object2D::Line(((200., 0.).into(), (200., 200.).into())),
            Object2D::Line(((0., 200.).into(), (200., 200.).into())),
        ]
        .iter()
        .map(|o| match o {
            // temp
            Object2D::Point(p) => Object2D::Point(*p / 50.),
            Object2D::Line((p1, p2)) => Object2D::Line((*p1 / 50., *p2 / 50.)),
            Object2D::Rectangle(r) => Object2D::Rectangle(*r),
            Object2D::Triangle(t) => Object2D::Triangle(*t),
        }),
    ));
    let init_pose = NewPose {
        angle: PI,
        position: Point {
            x: 8. + noise.sample(&mut rng),
            y: 8. + noise.sample(&mut rng),
        },
        vel_angle: 0.,
        velocity: Point { x: 0., y: 0. },
    };
    let robot_pose = NewPose {
        angle: PI,
        position: Point { x: 8., y: 8. },
        vel_angle: 0.,
        velocity: Point { x: 0., y: 0. },
    };
    let mut distance_sensor =
        DummyDistanceSensor::new(9., NewPose::default(), map.clone(), robot_pose, None);

    let motion_sensor = DummyAccelerationSensor::new(Pose {
        angle: 0.1,
        position: Point { x: 0.2, y: 0.2 },
    });

    let mut filter = KalmanFilter::new(
        Matrix6::from_diagonal(&Vector6::new(9., 9., 0., 0., 0., 0.)),
        init_pose.into(),
        robot_pose.into(),
        1e-3,
        0.,
        2.,
        q,
        r,
        distance_sensor.clone(),
        motion_sensor,
    );
    use piston_window::*;
    let map_visual_margins: Point = (25., 25.).into();
    let mut window: PistonWindow = WindowSettings::new("😎", [1000, 1000])
        .exit_on_esc(true)
        .build()
        .unwrap();
    let mut tick = 0;
    while let Some(e) = window.next() {
        window.draw_2d(&e, |c, g, _device| {
            clear([1.0; 4], g);
            for line in map.lines.clone() {
                line_from_to(
                    [0., 0., 0., 1.],
                    1.,
                    map.vertices[line.0] * 100. + map_visual_margins,
                    map.vertices[line.1] * 100. + map_visual_margins,
                    c.transform,
                    g,
                );
            }
            for point in map.points.clone() {
                let v: Point = map.vertices[point] * 100. + map_visual_margins;
                let size: Point = (5., 5.).into();
                ellipse_from_to([0.3, 0.3, 0.3, 1.], v + size, v - size, c.transform, g);
            }
        });
        distance_sensor.update_pose(filter.known_state.into());
        filter.prediction_update(time_scale);
        filter.measurement_update(RowVector1::new(distance_sensor.sense()));
        if tick % 1000 == 0 {
            let diff: NewPose = (filter.known_state - filter.real_state).into();
            println!("{:?}", diff);
        }
        tick += 1;
    }
}
