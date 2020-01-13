#![allow(dead_code)] // TODO: This will be removed after organizational overhall and a lib.rs
mod robot;
mod utility;

use piston_window::*;
use rand::distributions::{Distribution, Normal};
use rand::thread_rng;
use robot::ai::localization::DistanceFinderMCL;
use robot::map::{Map2D, Object2D};
use robot::sensors::dummy::{DummyDistanceSensor, DummyPositionSensor};
use robot::sensors::Sensor;
use std::{
    f64::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_8, PI},
    sync::Arc,
};
use utility::{KinematicState, Point, Pose};

fn main() {
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
    ));
    let mut robot_pose = KinematicState {
        angle: FRAC_PI_2,
        position: Point { x: 8., y: 8. },
        vel_angle: 0.,
        velocity: Point { x: 0., y: 0. },
    };

    let distance_sensor_count = 4;
    let mut distance_sensors: Vec<DummyDistanceSensor> = (0..distance_sensor_count)
        .into_iter()
        .map(|i| {
            DummyDistanceSensor::new(
                0.5,
                KinematicState {
                    angle: PI / distance_sensor_count as f64,
                    ..KinematicState::default()
                },
                map.clone(),
                robot_pose.clone(),
                Some(100.),
            )
        })
        .collect();

    let mut motion_sensor = DummyPositionSensor::new(
        robot_pose,
        KinematicState {
            position: (0.5, 0.5).into(),
            angle: FRAC_PI_8 / 4.,
            ..KinematicState::default()
        },
    );

    let mut mcl = DistanceFinderMCL::new(
        20_000,
        map.clone(),
        Box::new(|error| 1.05f64.powf(-error)),
        KinematicState {
            angle: FRAC_PI_8 / 4.,
            position: Point { x: 0.5, y: 0.5 },
            ..KinematicState::default()
        },
    );

    const MAP_SCALE: f64 = 2.;
    const ROBOT_ACCEL: f64 = 3.;
    const ROBOT_ANGLE_ACCEL: f64 = 0.1;
    let map_visual_margins: Point = (25., 25.).into();
    let mut window: PistonWindow = WindowSettings::new("😎", [1000, 1000])
        .exit_on_esc(true)
        .build()
        .unwrap();
    let mut tick: u32 = 0;

    while let Some(e) = window.next() {
        // User input
        if let Some(Button::Keyboard(key)) = e.press_args() {
            match key {
                keyboard::Key::W => {
                    robot_pose.position.x += ROBOT_ACCEL * robot_pose.angle.cos();
                    robot_pose.position.y += ROBOT_ACCEL * robot_pose.angle.sin();
                }
                keyboard::Key::S => {
                    robot_pose.position.x -= ROBOT_ACCEL * robot_pose.angle.cos();
                    robot_pose.position.y -= ROBOT_ACCEL * robot_pose.angle.sin();
                }
                keyboard::Key::A => robot_pose.angle -= ROBOT_ANGLE_ACCEL,
                keyboard::Key::D => robot_pose.angle += ROBOT_ANGLE_ACCEL,

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
            println!("Particle count: {}", mcl.belief.len());
            for particle in &mcl.belief {
                isoceles_triangle(
                    [0., 0., 0., 1.],
                    map_visual_margins,
                    MAP_SCALE,
                    0.5,
                    *particle,
                    c.transform,
                    g
                );
            }
            isoceles_triangle(
                [0., 1., 0., 1.],
                map_visual_margins,
                MAP_SCALE,
                1.,
                robot_pose,
                c.transform,
                g,
            );
            isoceles_triangle(
                [0., 0., 1., 1.],
                map_visual_margins,
                MAP_SCALE,
                1.,
                mcl.get_prediction(),
                c.transform,
                g,
            );
        });

        // Update the filter and sensors
        motion_sensor.update_pose(robot_pose);
        distance_sensors.iter_mut().for_each(|distance_sensor| {
            distance_sensor.update_pose(robot_pose);
        });
        mcl.control_update(&motion_sensor);
        mcl.observation_update(&distance_sensors);
        tick += 1;
    }
}

fn isoceles_triangle<G: Graphics>(color: [f32; 4], margin: Point, pose_scale: f64, triangle_scale: f64, pose: KinematicState, transform: math::Matrix2d, g: &mut G) {
    polygon(
        color,
        &[
            [
                pose.position.x * pose_scale
                    + margin.x
                    + triangle_scale * 15. * pose.angle.cos(),
                pose.position.y * pose_scale
                    + margin.y
                    + triangle_scale * 15. * pose.angle.sin(),
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
