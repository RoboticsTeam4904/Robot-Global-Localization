#![allow(dead_code)] // TODO: This will be removed after organizational overhall and a lib.rs
mod robot;
mod utility;
use nalgebra::{ArrayStorage, Matrix, Matrix5, Matrix6, RowVector5, Vector5, Vector6, U1, U7};
use rand::{
    distributions::{Distribution, Normal},
    thread_rng,
};
use robot::{
    ai::localization::KalmanFilter,
    map::{Map2D, Object2D},
    sensors::{
        dummy::{DummyDistanceSensor, DummyVelocitySensor},
        LimitedSensor, Sensor,
    },
};
use std::{
    f64::consts::{FRAC_PI_2, FRAC_PI_3, PI},
    ops::Range,
    sync::Arc,
};
use utility::{KinematicState, Point, Pose};

const ANGLE_NOISE: f64 = 0.;
const X_NOISE: f64 = 25.;
const Y_NOISE: f64 = 3.;
const X_MCL_NOISE: f64 = 0.5;
const Y_MCL_NOISE: f64 = 0.5;
const ANGLE_MCL_NOISE: f64 = 0.5;
const CONTROL_X_NOISE: f64 = 0.02;
const CONTROL_Y_NOISE: f64 = 0.02;
const CONTROL_ANGLE_NOISE: f64 = 0.002;
const VELOCITY_X_SENSOR_NOISE: f64 = 0.05;
const VELOCITY_Y_SENSOR_NOISE: f64 = 0.05;
const ROTATIONAL_VELOCITY_SENSOR_NOISE: f64 = 0.05;
const TIME_SCALE: u32 = 400;
const MAP_SCALE: f64 = 2.;
const ROBOT_ACCEL: f64 = 3. * TIME_SCALE as f64;
const ROBOT_ANGLE_ACCEL: f64 = 0.1 * TIME_SCALE as f64;

fn main() {
    let q: Matrix6<f64> = Matrix6::from_diagonal(&Vector6::new(
        0.00000,
        0.00000,
        0.00000,
        CONTROL_ANGLE_NOISE.powi(2),
        CONTROL_X_NOISE.powi(2),
        CONTROL_Y_NOISE.powi(2),
    ));

    let r: Matrix6<f64> = Matrix6::from_diagonal(&Vector6::from_vec(vec![
        ANGLE_MCL_NOISE.powi(2),
        X_MCL_NOISE.powi(2),
        Y_MCL_NOISE.powi(2),
        ROTATIONAL_VELOCITY_SENSOR_NOISE.powi(2),
        VELOCITY_X_SENSOR_NOISE.powi(2),
        VELOCITY_Y_SENSOR_NOISE.powi(2),
    ]));
    let mut rng = thread_rng();

    let noise_x = Normal::new(0., X_NOISE);
    let noise_angle = Normal::new(0., ANGLE_NOISE);
    let noise_y = Normal::new(0., Y_NOISE);
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
            // Object2D::Line(((20., 60.).into(), (50., 100.).into())),
            // Object2D::Line(((50., 100.).into(), (80., 50.).into())),
            // Object2D::Line(((80., 50.).into(), (20., 60.).into())),
            // Object2D::Line(((140., 100.).into(), (180., 120.).into())),
            // Object2D::Line(((180., 120.).into(), (160., 90.).into())),
            // Object2D::Line(((160., 90.).into(), (140., 100.).into())),
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
    let init_state = KinematicState {
        angle: FRAC_PI_2 + noise_angle.sample(&mut rng),
        position: Point {
            x: (100. + noise_x.sample(&mut rng)),
            y: (8. + noise_y.sample(&mut rng)),
        },
        vel_angle: 0.,
        velocity: Point { x: 0., y: 0. },
    };
    println!("{:?}", init_state);
    let mut robot_state = KinematicState {
        angle: FRAC_PI_2,
        position: Point { x: 100., y: 8. },
        vel_angle: 0.,
        velocity: Point { x: 0., y: 0. },
    };
    let distance_points = vec![
        Pose {
            angle: FRAC_PI_2,
            position: Point { x: 0., y: 0. },
        },
        Pose {
            angle: 0.,
            position: Point { x: 0., y: 0. },
        },
        Pose {
            angle: -FRAC_PI_2,
            position: Point { x: 0., y: 0. },
        },
        Pose {
            angle: PI,
            position: Point { x: 0., y: 0. },
        },
    ];

    let motion_sensor = DummyVelocitySensor::new(
        Pose {
            angle: ROTATIONAL_VELOCITY_SENSOR_NOISE,
            position: Point {
                x: VELOCITY_X_SENSOR_NOISE,
                y: VELOCITY_Y_SENSOR_NOISE,
            },
        },
        Pose {
            angle: robot_state.vel_angle,
            position: robot_state.velocity,
        },
    );

    let mut filter = KalmanFilter::new(
        Matrix6::from_diagonal(&Vector6::new(
            ANGLE_NOISE.powi(2),
            X_NOISE.powi(2),
            Y_NOISE.powi(2),
            0.,
            0.,
            0.,
        )),
        init_state.into(),
        1e-5,
        0.,
        2.,
        q,
        r,
    );

    use piston_window::*;
    let map_visual_margins: Point = (25., 25.).into();
    let mut window: PistonWindow = WindowSettings::new("😎", [1000, 1000])
        .exit_on_esc(true)
        .build()
        .unwrap();
    let mut tick: u32 = 0;
    let scaler = 10.;
    let control_noise_angle = Normal::new(0., CONTROL_ANGLE_NOISE * TIME_SCALE as f64);
    let control_noise_x = Normal::new(0., CONTROL_X_NOISE * TIME_SCALE as f64);
    let control_noise_y = Normal::new(0., CONTROL_Y_NOISE * TIME_SCALE as f64);

    while let Some(e) = window.next() {
        let mut control = Pose::default();
        // User input
        if let Some(Button::Keyboard(key)) = e.press_args() {
            let robot_angle = robot_state.angle;
            match key {
                keyboard::Key::W => {
                    control.position.x += ROBOT_ACCEL;
                }
                keyboard::Key::S => {
                    control.position.x -= ROBOT_ACCEL;
                }
                keyboard::Key::A => control.angle -= ROBOT_ANGLE_ACCEL,
                keyboard::Key::D => control.angle += ROBOT_ANGLE_ACCEL,

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
                        robot_state.position.x * MAP_SCALE
                            + map_visual_margins.x
                            + scaler * 1.5 * 2. * robot_state.angle.cos(),
                        robot_state.position.y * MAP_SCALE
                            + map_visual_margins.y
                            + scaler * 1.5 * 2. * robot_state.angle.sin(),
                    ],
                    [
                        robot_state.position.x * MAP_SCALE
                            + map_visual_margins.x
                            + scaler * 2. * (robot_state.angle + 2. * FRAC_PI_3).cos(),
                        robot_state.position.y * MAP_SCALE
                            + map_visual_margins.y
                            + scaler * 2. * (robot_state.angle + 2. * FRAC_PI_3).sin(),
                    ],
                    [
                        robot_state.position.x * MAP_SCALE
                            + map_visual_margins.x
                            + scaler * 2. * (robot_state.angle + 4. * FRAC_PI_3).cos(),
                        robot_state.position.y * MAP_SCALE
                            + map_visual_margins.y
                            + scaler * 2. * (robot_state.angle + 4. * FRAC_PI_3).sin(),
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
            let diff2: KinematicState = robot_state;
            let diff: KinematicState = (filter.known_state).into();
            println!(
                "The difference between predicted pose and real pose is {:?}{:?} at time {}.",
                filter.known_state,
                filter.covariance_matrix,
                tick as f64 / TIME_SCALE as f64
            )
        }

        // Update the filter and sensors
        let mut rng = thread_rng();
        let control_noise = Pose {
            angle: control_noise_angle.sample(&mut rng),
            position: (
                control_noise_x.sample(&mut rng),
                control_noise_y.sample(&mut rng),
            )
                .into(),
        };

        robot_state.position.x += robot_state.velocity.x / TIME_SCALE as f64;
        robot_state.position.y += robot_state.velocity.y / TIME_SCALE as f64;
        robot_state.velocity.x += (control.position.x * robot_state.angle.cos()
            + control.position.y * (robot_state.angle - FRAC_PI_2).cos())
            / TIME_SCALE as f64;
        robot_state.velocity.y += (control.position.x * robot_state.angle.sin()
            + control.position.y * (robot_state.angle - FRAC_PI_2).sin())
            / TIME_SCALE as f64;
        robot_state.angle =
            (robot_state.angle + robot_state.vel_angle / TIME_SCALE as f64) % (2. * PI);
        robot_state.vel_angle += control.angle / TIME_SCALE as f64;
        let temp = robot_state.clone();
        robot_state = temp
            .clamp_control_update(Range {
                start: Point { x: 0.1, y: 0.1 },
                end: Point { x: 199.9, y: 199. },
            })
            .into();

        let mut control = control;
        control.angle -= (robot_state.vel_angle - temp.vel_angle) * TIME_SCALE as f64;
        control.position.x -= (robot_state.velocity.x - temp.velocity.x) * TIME_SCALE as f64;
        control.position.y -= (robot_state.velocity.y - temp.velocity.y) * TIME_SCALE as f64;
        control += control_noise;

        motion_sensor.update_pose(Pose {
            angle: robot_state.vel_angle,
            position: (robot_state.velocity.x, robot_state.velocity.y).into(),
        });

        filter.prediction_update(1. / TIME_SCALE as f64, control);
        filter.measurement_update(
            motion_sensor.sense(),
            "POSE THAT LEO BUBBALU WILL BE FEEDING IN",
        );

        tick += 1;
        // println!(
        //     "{:?}",
        //     distance_sensors
        //         .iter()
        //         .map(|distance_sensor| distance_sensor.sense())
        //         .collect::<Vec<f64>>()
        // );
    }
}
