#![allow(dead_code)] // TODO: This will be removed after organizational overhall and a lib.rs
mod robot;
mod utility;

use nalgebra::{Matrix4, Matrix6, RowVector4, Vector4, Vector6};
use rand::distributions::{Distribution, Normal};
use rand::thread_rng;
use robot::ai::localization::KalmanFilter;
use robot::map::{Map2D, Object2D};
use robot::sensors::dummy::{DummyAccelerationSensor, DummyDistanceSensor};
use robot::sensors::Sensor;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI};
use std::sync::Arc;
use utility::{KinematicState, Point, Pose};

fn main() {
    let q: Matrix6<f64> = Matrix6::from_diagonal(&Vector6::new(
        0.,
        0.,
        0.,
        (0.01f64).powi(2),
        (0.01f64).powi(2),
        (0.01f64).powi(2),
    ));
    let r: Matrix4<f64> = Matrix4::from_diagonal(&Vector4::from_vec(vec![0.04; 4]));
    let mut rng = thread_rng();
    let noise = Normal::new(0., 1.);
    let map = Arc::new(Map2D::new(
        200.,
        200.,
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
            Object2D::Point(p) => Object2D::Point(*p),
            Object2D::Line((p1, p2)) => Object2D::Line((*p1, *p2)),
            Object2D::Rectangle(r) => Object2D::Rectangle(*r),
            Object2D::Triangle(t) => Object2D::Triangle(*t),
        }),
    ));
    let init_pose = KinematicState {
        angle: FRAC_PI_2,
        position: Point {
            x: 8. + noise.sample(&mut rng),
            y: 8. + noise.sample(&mut rng),
        },
        vel_angle: 0.,
        velocity: Point { x: 0., y: 0. },
    };
    let robot_pose = KinematicState {
        angle: FRAC_PI_2,
        position: Point { x: 8., y: 8. },
        vel_angle: 0.,
        velocity: Point { x: 0., y: 0. },
    };
    let distance_points = vec![
        Pose {
            angle: FRAC_PI_2,
            position: Point { x: 0., y: 10. / 3. },
        },
        Pose {
            angle: -FRAC_PI_2,
            position: Point {
                x: 0.,
                y: -10. / 3.,
            },
        },
        Pose {
            angle: 0.,
            position: Point { x: 10. / 3., y: 0. },
        },
        Pose {
            angle: PI,
            position: Point {
                x: -10. / 3.,
                y: 0.,
            },
        },
    ];
    let mut distance_sensors: Vec<DummyDistanceSensor> = distance_points
        .iter()
        .map(|e| {
            DummyDistanceSensor::new(
                0.2,
                KinematicState {
                    angle: e.angle,
                    position: e.position,
                    vel_angle: 0.,
                    velocity: Point::default(),
                },
                map.clone(),
                robot_pose.clone(),
                Some(100.),
            )
        })
        .collect();

    let motion_sensor = DummyAccelerationSensor::new(Pose {
        angle: 0.1 * TIME_SCALE as f64,
        position: Point {
            x: 0.1 * TIME_SCALE as f64,
            y: 0.1 * TIME_SCALE as f64,
        },
    });

    let mut filter = KalmanFilter::new(
        Matrix6::from_diagonal(&Vector6::new(0., 9., 9., 0., 0., 0.)),
        init_pose.into(),
        robot_pose.into(),
        1e-3,
        0.,
        2.,
        q,
        r,
        distance_sensors.clone(),
        motion_sensor,
    );
    const TIME_SCALE: u32 = 400;
    const MAP_SCALE: f64 = 2.;
    const ROBOT_ACCEL: f64 = 3. * TIME_SCALE as f64;
    const ROBOT_ANGLE_ACCEL: f64 = 0.1 * TIME_SCALE as f64;
    use piston_window::*;
    let map_visual_margins: Point = (25., 25.).into();
    let mut window: PistonWindow = WindowSettings::new("😎", [1000, 1000])
        .exit_on_esc(true)
        .build()
        .unwrap();
    let mut tick: u32 = 0;
    let scaler = 10.;

    while let Some(e) = window.next() {
        let mut control_update = Pose::default();
        // User input
        if let Some(Button::Keyboard(key)) = e.press_args() {
            let robot_angle = filter.real_state[0];
            match key {
                keyboard::Key::W => {
                    control_update.position.x += ROBOT_ACCEL * robot_angle.cos();
                    control_update.position.y += ROBOT_ACCEL * robot_angle.sin();
                }
                keyboard::Key::S => {
                    control_update.position.x -= ROBOT_ACCEL * robot_angle.cos();
                    control_update.position.y -= ROBOT_ACCEL * robot_angle.sin();
                }
                keyboard::Key::A => control_update.angle -= ROBOT_ANGLE_ACCEL,
                keyboard::Key::D => control_update.angle += ROBOT_ANGLE_ACCEL,

                _ => (),
            }
        }

        // Rendering
        window.draw_2d(&e, |c, g, _device| {
            clear([1.0; 4], g);
            for line in map.lines.clone() {
                line_from_to(
                    [0., 0., 0., 1.],
                    1.,
                    map.vertices[line.0] * MAP_SCALE + map_visual_margins,
                    map.vertices[line.1] * MAP_SCALE + map_visual_margins,
                    c.transform,
                    g,
                );
            }
            for point in map.points.clone() {
                let v: Point = map.vertices[point] * MAP_SCALE + map_visual_margins;
                let size: Point = (5., 5.).into();
                ellipse_from_to([0.7, 0.3, 0.3, 1.], v + size, v - size, c.transform, g);
            }
            polygon(
                [0., 0., 0., 1.],
                &[
                    [
                        filter.real_state[1] * MAP_SCALE
                            + map_visual_margins.x
                            + scaler * 1.5 * filter.real_state[0].cos(),
                        filter.real_state[2] * MAP_SCALE
                            + map_visual_margins.y
                            + scaler * 1.5 * filter.real_state[0].sin(),
                    ],
                    [
                        filter.real_state[1] * MAP_SCALE
                            + map_visual_margins.x
                            + scaler * (filter.real_state[0] + 2. * FRAC_PI_3).cos(),
                        filter.real_state[2] * MAP_SCALE
                            + map_visual_margins.y
                            + scaler * (filter.real_state[0] + 2. * FRAC_PI_3).sin(),
                    ],
                    [
                        filter.real_state[1] * MAP_SCALE
                            + map_visual_margins.x
                            + scaler * (filter.real_state[0] + 4. * FRAC_PI_3).cos(),
                        filter.real_state[2] * MAP_SCALE
                            + map_visual_margins.y
                            + scaler * (filter.real_state[0] + 4. * FRAC_PI_3).sin(),
                    ],
                ],
                c.transform,
                g,
            );
            polygon(
                [0.3, 0.3, 0.3, 1.],
                &[
                    [
                        filter.known_state[1] * MAP_SCALE
                            + map_visual_margins.x
                            + scaler * 1.5 * filter.known_state[0].cos(),
                        filter.known_state[2] * MAP_SCALE
                            + map_visual_margins.y
                            + scaler * 1.5 * filter.known_state[0].sin(),
                    ],
                    [
                        filter.known_state[1] * MAP_SCALE
                            + map_visual_margins.x
                            + scaler * (filter.known_state[0] + 2. * FRAC_PI_3).cos(),
                        filter.known_state[2] * MAP_SCALE
                            + map_visual_margins.y
                            + scaler * (filter.known_state[0] + 2. * FRAC_PI_3).sin(),
                    ],
                    [
                        filter.known_state[1] * MAP_SCALE
                            + map_visual_margins.x
                            + scaler * (filter.known_state[0] + 4. * FRAC_PI_3).cos(),
                        filter.known_state[2] * MAP_SCALE
                            + map_visual_margins.y
                            + scaler * (filter.known_state[0] + 4. * FRAC_PI_3).sin(),
                    ],
                ],
                c.transform,
                g,
            );
        });
        if tick % TIME_SCALE == 0 {
            let diff: KinematicState = (filter.real_state - filter.known_state).into();
            println!(
                "The difference between predicted pose and real pose is {:?} at time {}.",
                diff,
                tick as f64 / TIME_SCALE as f64
            )
        }

        // Update the filter and sensors
        distance_sensors.iter_mut().for_each(|distance_sensor| {
            distance_sensor.update_pose(filter.real_state.into());
        });
        filter.prediction_update(1. / TIME_SCALE as f64, control_update);
        filter.measurement_update(RowVector4::from_vec(
            distance_sensors
                .iter()
                .map(|distance_sensor| distance_sensor.sense())
                .collect(),
        ));
        tick += 1;
    }
}
