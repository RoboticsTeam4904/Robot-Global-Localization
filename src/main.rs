#![allow(dead_code)] // TODO: This will be removed after organizational overhall and a lib.rs
mod replay;
mod robot;
mod utility;
use nalgebra::{Matrix, Matrix5, Matrix6, RowVector5, Vector5, Vector6, U1, U7};
use piston_window::*;
use rand::{
    distributions::{Distribution, Normal},
    thread_rng,
};
use robot::{
    ai::localization::{DistanceFinderMCL, KalmanFilter, DeathCondition},
    map::{Map2D, Object2D},
    sensors::{
        dummy::{DummyDistanceSensor, DummyPositionSensor, DummyVelocitySensor},
        Sensor,
    },
};
use std::{
    f64::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_8, PI},
    ops::Range,
    sync::Arc,
    thread::sleep,
    time::Duration,
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
const TIME_SCALE: f64 = 400.;
const MAP_SCALE: f64 = 2.;
const ROBOT_ACCEL: f64 = 3. * TIME_SCALE;
const ROBOT_ANGLE_ACCEL: f64 = 0.1 * TIME_SCALE;

fn main() {
    let mut rng = thread_rng();

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
            Object2D::Line(((20., 60.).into(), (50., 100.).into())),
            Object2D::Line(((50., 100.).into(), (80., 50.).into())),
            Object2D::Line(((80., 50.).into(), (20., 60.).into())),
            Object2D::Line(((140., 100.).into(), (180., 120.).into())),
            Object2D::Line(((180., 120.).into(), (160., 90.).into())),
            Object2D::Line(((160., 90.).into(), (140., 100.).into())),
            Object2D::Line(((100., 40.).into(), (160., 80.).into())),
        ],
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
    let mut robot_state = KinematicState {
        angle: FRAC_PI_2,
        position: Point { x: 100., y: 8. },
        vel_angle: 0.,
        velocity: Point { x: 0., y: 0. },
    };

    let mut motion_sensor = DummyVelocitySensor::new(
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
    let mut position_sensor = DummyPositionSensor::new(
        Pose {
            angle: robot_state.angle,
            position: robot_state.position,
        },
        Pose {
            angle: FRAC_PI_8 / 4.,
            position: Point { x: 1., y: 1. },
        },
    );
    let mut distance_sensors: Vec<DummyDistanceSensor> = (0..8)
        .map(|i| {
            DummyDistanceSensor::new(
                0.,
                Pose {
                    angle: FRAC_PI_2 * i as f64,
                    ..Pose::default()
                },
                map.clone(),
                robot_state.pose(),
                None,
            )
        })
        .collect();

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
    let mut mcl = {
        let particle_count = 40_000;
        let weight_sum_threshold = 400.;
        let death_threshold = DeathCondition {
            particle_count_threshold: 2000,
            particle_concentration_threshold: 300.,
        };
        DistanceFinderMCL::new(
            particle_count,
            weight_sum_threshold,
            death_threshold,
            map.clone(),
            Box::new(|e| 1.05f64.powf(-e)),
            Box::new(move |_| {
                Pose::random_from_range(Pose {
                    angle: 0.0001,
                    position: (0.5, 0.5).into(),
                })
            }),
        )
    };

    let map_visual_margins: Point = (25., 25.).into();
    let mut window: PistonWindow = WindowSettings::new("😎", [1000, 1000])
        .exit_on_esc(true)
        .build()
        .unwrap();
    let mut tick: u32 = 0;
    let control_noise_angle = Normal::new(0., CONTROL_ANGLE_NOISE * TIME_SCALE);
    let control_noise_x = Normal::new(0., CONTROL_X_NOISE * TIME_SCALE);
    let control_noise_y = Normal::new(0., CONTROL_Y_NOISE * TIME_SCALE);

    while let Some(e) = window.next() {
        // User input
        let mut control = Pose::default();
        if let Some(Button::Keyboard(key)) = e.press_args() {
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
            for particle in &mcl.belief {
                isoceles_triangle(
                    [1., 0., 0., 1.],
                    map_visual_margins,
                    MAP_SCALE,
                    0.2,
                    *particle,
                    c.transform,
                    g,
                )
            }
            isoceles_triangle(
                [0., 0., 0., 1.],
                map_visual_margins,
                MAP_SCALE,
                0.75,
                Pose {
                    angle: robot_state.angle,
                    position: robot_state.position,
                },
                c.transform,
                g,
            );
            isoceles_triangle(
                [0., 0., 1., 1.],
                map_visual_margins,
                MAP_SCALE,
                0.5,
                mcl.get_prediction(),
                c.transform,
                g,
            );
            let filter_prediction: KinematicState = filter.known_state.into();
            isoceles_triangle(
                [0., 1., 0., 1.],
                map_visual_margins,
                MAP_SCALE,
                0.5,
                filter_prediction.pose(),
                c.transform,
                g,
            );
        });

        // Update the physics simulation
        robot_state.position.x += robot_state.velocity.x / TIME_SCALE;
        robot_state.position.y += robot_state.velocity.y / TIME_SCALE;
        robot_state.velocity.x += (control.position.x * robot_state.angle.cos()
            + control.position.y * (robot_state.angle - FRAC_PI_2).cos())
            / TIME_SCALE;
        robot_state.velocity.y += (control.position.x * robot_state.angle.sin()
            + control.position.y * (robot_state.angle - FRAC_PI_2).sin())
            / TIME_SCALE;
        robot_state.angle = (robot_state.angle + robot_state.vel_angle / TIME_SCALE) % (2. * PI);
        robot_state.vel_angle += control.angle / TIME_SCALE;
        let temp = robot_state.clone();
        robot_state = temp
            .clamp_control_update(Range {
                start: Point { x: 0.1, y: 0.1 },
                end: Point { x: 199.9, y: 199. },
            })
            .into();

        let mut control = control;
        control.angle -= (robot_state.vel_angle - temp.vel_angle) * TIME_SCALE;
        control.position.x -= (robot_state.velocity.x - temp.velocity.x) * TIME_SCALE;
        control.position.y -= (robot_state.velocity.y - temp.velocity.y) * TIME_SCALE;

        let control_noise = Pose {
            angle: control_noise_angle.sample(&mut rng),
            position: (
                control_noise_x.sample(&mut rng),
                control_noise_y.sample(&mut rng),
            )
                .into(),
        };
        control += control_noise;

        // update sensors
        motion_sensor.update_pose(Pose {
            angle: robot_state.vel_angle,
            position: robot_state.velocity,
        });
        position_sensor.update_pose(robot_state.pose());
        distance_sensors
            .iter_mut()
            .for_each(|d| d.update_pose(robot_state.pose()));

        println!("P = {}", mcl.belief.len());
        // update localization
        mcl.control_update(&position_sensor);
        mcl.observation_update(&distance_sensors);
        filter.prediction_update(TIME_SCALE.recip(), control);
        filter.measurement_update(motion_sensor.sense(), mcl.get_prediction());

        tick += 1;
        // sleep(Duration::from_secs_f64(0.5));
    }
}

fn isoceles_triangle<G: Graphics>(
    color: [f32; 4],
    margin: Point,
    pose_scale: f64,
    triangle_scale: f64,
    pose: Pose,
    transform: math::Matrix2d,
    g: &mut G,
) {
    polygon(
        color,
        &[
            [
                pose.position.x * pose_scale + margin.x + triangle_scale * 15. * pose.angle.cos(),
                pose.position.y * pose_scale + margin.y + triangle_scale * 15. * pose.angle.sin(),
            ],
            [
                pose.position.x * pose_scale
                    + margin.x
                    + triangle_scale * 10. * (pose.angle + 2. * FRAC_PI_3).cos(),
                pose.position.y * pose_scale
                    + margin.y
                    + triangle_scale * 10. * (pose.angle + 2. * FRAC_PI_3).sin(),
            ],
            [
                pose.position.x * pose_scale
                    + margin.x
                    + triangle_scale * 10. * (pose.angle + 4. * FRAC_PI_3).cos(),
                pose.position.y * pose_scale
                    + margin.y
                    + triangle_scale * 10. * (pose.angle + 4. * FRAC_PI_3).sin(),
            ],
        ],
        transform,
        g,
    );
}
