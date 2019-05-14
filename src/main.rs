#![allow(dead_code)] // TODO: This will be removed after organizational overhall and a lib.rs
mod robot;
mod utility;

use robot::map::{Map2D, Object2D};
use robot::Sensor;
use robot::sensors::dummy::{DummyDistanceSensor, DummyMotionSensor};
use robot::AI::localization::DistanceFinderMCL;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_8, PI};
use utility::{isoceles_triangle, Point, Pose};
use vitruvia::{
    graphics_2d,
    graphics_2d::{Color, Transform, Content},
    interaction::keyboard::{Arrow, Key},
    text::Text,
};

// TODO: Multithread everything

struct Robot {
    mcl: DistanceFinderMCL,
    motion_sensor: DummyMotionSensor,
    distance_sensors: Vec<DummyDistanceSensor>,
}

impl Robot {
    fn repeat(&mut self) {
        self.mcl.control_update(self.motion_sensor.sense()); // TODO: dummy motion sensor and its usage are currently oversimplified
        self.mcl.observation_update(
            self.distance_sensors
                .iter()
                .map(|sensor| sensor.sense())
                .collect(),
        );
    }
}

fn main() {
    // Setup visuals
    let gfx = graphics_2d::new();
    let mut _root = gfx.frame();
    // Start up the window
    let mut root = _root.clone();
    let ctx = gfx.start(_root);
    ctx.run_with(Box::new(move |mut context| {
        // Make a robot
        let mut robot = {
            let map = Map2D::new(
                10.,
                10.,
                vec![
                    Object2D::Rectangle((Point::default(), Point { x: 10., y: 10. })),
                    Object2D::Rectangle((Point { x: 10., y: 10. }, Point { x: 9., y: 7. })),
                    Object2D::Rectangle((Point { x: 2.5, y: 2.5 }, Point { x: 7.5, y: 3. })),
                    Object2D::Triangle((
                        Point { x: 1., y: 8. },
                        Point { x: 3., y: 8. },
                        Point { x: 2., y: 7. },
                    )),
                    Object2D::Line((Point { x: 5., y: 5. }, Point { x: 5., y: 10. })),
                ],
            );
            let starting_robot_pose = Pose {
                angle: 0.,
                position: Point { x: 8., y: 8. },
            };
            let distance_sensor_noise = 0.1;
            let distance_sensors = vec![
                DummyDistanceSensor::new(
                    distance_sensor_noise,
                    Pose::default(),
                    map.clone(),
                    starting_robot_pose,
                    None,
                ),
                DummyDistanceSensor::new(
                    distance_sensor_noise,
                    Pose::default()
                        + Pose {
                            angle: FRAC_PI_2,
                            position: Point::default(),
                        },
                    map.clone(),
                    starting_robot_pose,
                    None,
                ),
                DummyDistanceSensor::new(
                    distance_sensor_noise,
                    Pose::default()
                        + Pose {
                            angle: PI,
                            position: Point::default(),
                        },
                    map.clone(),
                    starting_robot_pose,
                    None,
                ),
                DummyDistanceSensor::new(
                    distance_sensor_noise,
                    Pose::default()
                        + Pose {
                            angle: PI + FRAC_PI_2,
                            position: Point::default(),
                        },
                    map.clone(),
                    starting_robot_pose,
                    None,
                ),
            ];
            Robot {
                mcl: DistanceFinderMCL::new(
                    20_000,
                    map.clone(),
                    distance_sensors
                        .iter()
                        .map(|sensor| sensor.get_relative_pose())
                        .collect(),
                    Box::new(|error| 2f64.powf(-error)),
                ),
                distance_sensors,
                motion_sensor: DummyMotionSensor::new(
                    starting_robot_pose,
                    Pose {
                        angle: FRAC_PI_8 / 4.,
                        position: Point { x: 0.01, y: 0.01 },
                    },
                ),
            }
        };
        // Make map visual
        root.add(
            robot
                .mcl
                .map
                .make_visual(50.)
                .with_transform(Transform::default().with_position((50., 50.))),
        );
        // Make tick, time, and particle counters' visuals
        let mut tick_visual = root.add(
            Content::from(Text::new("0t").with_size(30.))
                .with_transform(Transform::default().with_position((500., 20.))),
        );
        let start_time = std::time::Instant::now();
        let mut time_visual = root.add(
            Content::from(Text::new(format!("{:?}", start_time.elapsed()).as_str())
                .with_size(30.)).with_transform(Transform::default().with_position((50., 20.)))
        );
        let mut particle_count_visual = root.add(
            Content::from(Text::new(format!("{}p", robot.mcl.belief.len()).as_str())
                .with_size(30.)).with_transform(Transform::default().with_position((50., 570.))),
        );
        // Make particle visuals
        let mut particle_visuals = {
            let mut visuals = Vec::with_capacity(robot.mcl.belief.len());
            let path: vitruvia::graphics_2d::Content = isoceles_triangle(5., 8.)
                .fill(Color::black().into())
                .finalize()
                .into();
            for particle in &robot.mcl.belief {
                let mut particle_visual = root.add(path.clone());
                particle_visual.apply_transform(
                    Transform::default()
                        .with_position(particle.position * 50. + 50.)
                        .with_rotation(-particle.angle)
                        .with_scale(0.5),
                );
                visuals.push(particle_visual);
            }
            visuals
        };
        // Make position visuals
        let mut predicted_pose_visual = {
            let path = isoceles_triangle(10., 16.)
                .fill(Color::rgb(255, 0, 0).into())
                .finalize();
            let mut visual = root.add(path.into());
            let estimation = robot.mcl.get_prediction();
            visual.apply_transform(
                Transform::default()
                    .with_position(estimation.position * 50. + 50.)
                    .with_rotation(-estimation.angle),
            );
            visual
        };
        let mut real_pose_visual = {
            let path = isoceles_triangle(10., 16.)
                .fill(Color::rgb(0, 0, 255).into())
                .finalize();
            let mut visual = root.add(path.into());
            let pose = robot.motion_sensor.robot_pose;
            visual.apply_transform(
                Transform::default()
                    .with_position(pose.position * 50. + 50.)
                    .with_rotation(-pose.angle),
            );
            visual
        };
        let mut root = root.clone();
        let ctx = context.clone();
        context.bind(Box::new(move |t| {
            // Get user input to move the robot
            let mut inp = String::new();
            std::io::stdin().read_line(&mut inp).unwrap();
            let movement_cmd = robot.motion_sensor.robot_pose
                + Pose {
                    angle: PI / 8.
                        * if inp == "q\n" {
                            1.
                        } else if inp == "e\n" {
                            -1.
                        } else {
                            0.
                        },
                    position: if inp == "w\n" {
                        Point { x: 0., y: -0.5 }
                    } else if inp == "s\n" {
                        Point { x: 0., y: 0.5 }
                    } else if inp == "a\n" {
                        Point { x: -0.5, y: 0. }
                    } else if inp == "d\n" {
                        Point { x: 0.5, y: 0. }
                    } else {
                        Point::default()
                    },
                };
            // Move the robot and tick the mcl algorithm
            robot.motion_sensor.update_pose(movement_cmd);
            robot
                .distance_sensors
                .iter_mut()
                .for_each(|s| s.update_pose(movement_cmd));
            robot.repeat();
            let real_pose = robot.motion_sensor.robot_pose;
            let predicted_pose = robot.mcl.get_prediction();
            // Update visuals
            // Update particle visuals
            if robot.mcl.belief.len() < particle_visuals.len() {
                for i in robot.mcl.belief.len()..particle_visuals.len() {
                    particle_visuals[i]
                        .set_transform(Transform::default().with_scale(0.1e-100_f64));
                }
            } else if robot.mcl.belief.len() != particle_visuals.len() {
                let path: vitruvia::graphics_2d::Content = isoceles_triangle(5., 8.)
                    .fill(Color::black().into())
                    .finalize()
                    .into();
                for _ in particle_visuals.len()..robot.mcl.belief.len() {
                    particle_visuals.push(root.add(path.clone()));
                }
            }
            for i in 0..robot.mcl.belief.len() {
                let particle = robot.mcl.belief[i];
                particle_visuals[i].set_transform(
                    Transform::default()
                        .with_position(particle.position * 50. + 50.)
                        .with_rotation(-particle.angle)
                        .with_scale(0.5),
                );
            }
            // Update tick, time, particle counters' visuals
            tick_visual.update(Text::new(format!("{}t", t).as_str()).with_size(30.).into());
            time_visual.update(
                Text::new(format!("{:?}", start_time.elapsed()).as_str())
                    .with_size(30.)
                    .into(),
            );
            particle_count_visual.update(
                Text::new(format!("{}p", robot.mcl.belief.len()).as_str())
                    .with_size(30.)
                    .into(),
            );
            // Update position visuals
            predicted_pose_visual.set_transform(
                Transform::default()
                    .with_position(predicted_pose.position * 50. + 50.)
                    .with_rotation(-predicted_pose.angle),
            );
            real_pose_visual.set_transform(
                Transform::default()
                    .with_position(real_pose.position * 50. + 50.)
                    .with_rotation(-real_pose.angle),
            );
        }));
    }));
}
