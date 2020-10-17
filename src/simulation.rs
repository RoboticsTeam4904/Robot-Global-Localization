use global_robot_localization::{
    ai::{
        kalman_filter::{Config, KalmanFilter, LocalizationFilter},
        localization::{DeathCondition, PoseMCL},
        presets,
    },
    map::{Map2D, Object2D},
    replay::render::{draw_map, isoceles_triangle, point_cloud},
    sensors::{
        dummy::{DummyLidar, DummyObjectSensor3D, DummyPositionSensor, DummyVelocitySensor},
        Sensor,
    },
    utility::{
        variance_poses, DifferentialDriveState, KinematicState, Point, Point3D, Pose, Pose3D,
    },
};
use nalgebra::{Matrix6, RowVector6, Vector6};
use piston_window::*;
use rand::{
    distributions::{uniform::Uniform, Distribution, Normal},
    seq::IteratorRandom,
    thread_rng,
};
use std::{
    f64::consts::{FRAC_PI_2, FRAC_PI_8, PI},
    sync::Arc,
    time::Instant,
};

const ANGLE_NOISE: f64 = 0.;
const X_NOISE: f64 = 3.;
const Y_NOISE: f64 = 274.;
const CONTROL_X_NOISE: f64 = 0.01; // 3 cm / s^2 of noise
const CONTROL_Y_NOISE: f64 = 0.01; // 3 cm / s^2 of noise

const CONTROL_ANGLE_NOISE: f64 = 0.007; // 2 degrees / s^2 of noise
const VELOCITY_X_SENSOR_NOISE: f64 = 2.;
const VELOCITY_Y_SENSOR_NOISE: f64 = 2.;
const ROTATIONAL_VELOCITY_SENSOR_NOISE: f64 = 0.2;
const MAP_SCALE: f64 = 0.6;
const ROBOT_ACCEL: f64 = 400.;
const WHEEL_DIST: f64 = 120.;
const CAMERA_HEIGHT: f64 = 0.;
const CAMERA_ANGLE: f64 = 0.;
const CAMERA_FOV: Point = Point {
    x: 1.229,
    y: 0.7557,
};
const VISION_MAX_DIST: Option<f64> = Some(800.);

fn main() {
    let mut rng = thread_rng();

    let mut q: Matrix6<f64>;

    let mut r: Matrix6<f64>;

    let noise_x = Normal::new(410.5, X_NOISE);
    let noise_angle = Normal::new(0., ANGLE_NOISE);
    let noise_y = Normal::new(8., Y_NOISE);
    let sensor_map = Arc::new(Map2D::with_size(
        (1598., 821.).into(),
        vec![
            Object2D::Line((64.98, 0.).into(), (0., 178.54).into()),
            Object2D::Line((64.98, 821.).into(), (0., 642.46).into()),
            Object2D::Line((0., 642.46).into(), (0., 178.54).into()),
            Object2D::Line((1533.02, 0.).into(), (1598., 178.54).into()),
            Object2D::Line((1533.02, 821.).into(), (1598., 642.46).into()),
            Object2D::Line((1598., 642.46).into(), (1598., 178.54).into()),
            // Trench
            Object2D::Rectangle((630.54, 680.).into(), (706.54, 821.).into()),
            Object2D::Rectangle((974.5, 141.).into(), (898.5, 0.).into()),
            // Shield Generator
            Object2D::RectangleFour(
                (1065.17, 531.03).into(),
                (1072.22, 533.59).into(),
                (1074.78, 526.54).into(),
                (1067.73, 523.98).into(),
            ),
            Object2D::RectangleFour(
                (928.50, 141.).into(),
                (935.55, 143.57).into(),
                (932.98, 150.61).into(),
                (925.93, 148.05).into(),
            ),
            Object2D::RectangleFour(
                (665.02, 670.39).into(),
                (662.46, 677.44).into(),
                (669.5, 680.).into(),
                (672.07, 672.95).into(),
            ),
            Object2D::RectangleFour(
                (523.2, 294.46).into(),
                (525.78, 287.41).into(),
                (532.83, 289.97).into(),
                (530.27, 297.02).into(),
            ),
            Object2D::Target(Pose3D {
                angle: Point {
                    x: PI,
                    y: -CAMERA_ANGLE,
                },
                position: (1598., 239.54, 0. - CAMERA_HEIGHT).into(),
            }),
            Object2D::Target(Pose3D {
                angle: Point {
                    x: PI,
                    y: -CAMERA_ANGLE,
                },
                position: (1598., 566.54, 0. - CAMERA_HEIGHT).into(),
            }),
            Object2D::Target(Pose3D {
                angle: Point {
                    x: 0.,
                    y: -CAMERA_ANGLE,
                },
                position: (0., 581.54, 0. - CAMERA_HEIGHT).into(),
            }),
            Object2D::Target(Pose3D {
                angle: Point {
                    x: 0.,
                    y: -CAMERA_ANGLE,
                },
                position: (0., 254.54, 0. - CAMERA_HEIGHT).into(),
            }),
        ],
    ));

    let perceived_map = Arc::new(Map2D::with_size(
        (1598., 821.).into(),
        vec![
            Object2D::Line((64.98, 0.).into(), (0., 178.54).into()),
            Object2D::Line((64.98, 821.).into(), (0., 642.46).into()),
            Object2D::Line((0., 642.46).into(), (0., 178.54).into()),
            Object2D::Line((1533.02, 0.).into(), (1598., 178.54).into()),
            Object2D::Line((1533.02, 821.).into(), (1598., 642.46).into()),
            Object2D::Line((1598., 642.46).into(), (1598., 178.54).into()),
            Object2D::Line((64.98, 0.).into(), (1533.02, 0.).into()),
            Object2D::Line((64.98, 821.).into(), (1533.02, 821.).into()),
            // Trench
            Object2D::Rectangle((630.54, 680.).into(), (706.54, 821.).into()),
            Object2D::Rectangle((974.5, 141.).into(), (898.5, 0.).into()),
            // Shield Generator
            Object2D::RectangleFour(
                (1065.17, 531.03).into(),
                (1072.22, 533.59).into(),
                (1074.78, 526.54).into(),
                (1067.73, 523.98).into(),
            ),
            Object2D::RectangleFour(
                (928.50, 141.).into(),
                (935.55, 143.57).into(),
                (932.98, 150.61).into(),
                (925.93, 148.05).into(),
            ),
            Object2D::RectangleFour(
                (665.02, 670.39).into(),
                (662.46, 677.44).into(),
                (669.5, 680.).into(),
                (672.07, 672.95).into(),
            ),
            Object2D::RectangleFour(
                (523.2, 294.46).into(),
                (525.78, 287.41).into(),
                (532.83, 289.97).into(),
                (530.27, 297.02).into(),
            ),
            Object2D::Target(Pose3D {
                angle: Point {
                    x: PI,
                    y: -CAMERA_ANGLE,
                },
                position: (1598., 239.54, 41.91 - CAMERA_HEIGHT).into(),
            }),
            Object2D::Target(Pose3D {
                angle: Point {
                    x: PI,
                    y: -CAMERA_ANGLE,
                },
                position: (1598., 566.54, 249.55 - CAMERA_HEIGHT).into(),
            }),
            Object2D::Target(Pose3D {
                angle: Point {
                    x: 0.,
                    y: -CAMERA_ANGLE,
                },
                position: (0., 581.54, 41.91 - CAMERA_HEIGHT).into(),
            }),
            Object2D::Target(Pose3D {
                angle: Point {
                    x: 0.,
                    y: -CAMERA_ANGLE,
                },
                position: (0., 254.54, 249.55 - CAMERA_HEIGHT).into(),
            }),
        ],
    ));

    let real_map = Arc::new(Map2D::with_size(
        (1598., 821.).into(),
        vec![
            Object2D::Line((64.98, 0.).into(), (0., 178.54).into()),
            Object2D::Line((64.98, 821.).into(), (0., 642.46).into()),
            Object2D::Line((0., 642.46).into(), (0., 178.54).into()),
            Object2D::Line((1533.02, 0.).into(), (1598., 178.54).into()),
            Object2D::Line((1533.02, 821.).into(), (1598., 642.46).into()),
            Object2D::Line((1598., 642.46).into(), (1598., 178.54).into()),
            Object2D::Line((64.98, 0.).into(), (1533.02, 0.).into()),
            Object2D::Line((64.98, 821.).into(), (1533.02, 821.).into()),
            // Trench
            Object2D::Rectangle((630.54, 680.).into(), (706.54, 821.).into()),
            Object2D::Rectangle((974.5, 141.).into(), (898.5, 0.).into()),
            // Shield Generator
            Object2D::RectangleFour(
                (1065.17, 531.03).into(),
                (1072.22, 533.59).into(),
                (1074.78, 526.54).into(),
                (1067.73, 523.98).into(),
            ),
            Object2D::RectangleFour(
                (928.50, 141.).into(),
                (935.55, 143.57).into(),
                (932.98, 150.61).into(),
                (925.93, 148.05).into(),
            ),
            Object2D::RectangleFour(
                (665.02, 670.39).into(),
                (662.46, 677.44).into(),
                (669.5, 680.).into(),
                (672.07, 672.95).into(),
            ),
            Object2D::RectangleFour(
                (523.2, 294.46).into(),
                (525.78, 287.41).into(),
                (532.83, 289.97).into(),
                (530.27, 297.02).into(),
            ),
            Object2D::Target(Pose3D {
                angle: Point {
                    x: PI,
                    y: -CAMERA_ANGLE,
                },
                position: (1598., 239.54, 41.91 - CAMERA_HEIGHT).into(),
            }),
            Object2D::Target(Pose3D {
                angle: Point {
                    x: PI,
                    y: -CAMERA_ANGLE,
                },
                position: (1598., 566.54, 249.55 - CAMERA_HEIGHT).into(),
            }),
            Object2D::Target(Pose3D {
                angle: Point {
                    x: 0.,
                    y: -CAMERA_ANGLE,
                },
                position: (0., 581.54, 41.91 - CAMERA_HEIGHT).into(),
            }),
            Object2D::Target(Pose3D {
                angle: Point {
                    x: 0.,
                    y: -CAMERA_ANGLE,
                },
                position: (0., 254.54, 249.55 - CAMERA_HEIGHT).into(),
            }),
        ],
    ));
    let marker_map = Arc::new(Map2D::with_size(
        (1598., 821.).into(),
        vec![
            Object2D::Line((305., 0.).into(), (305., 821.).into()),
            Object2D::Line((1293., 0.).into(), (1293., 821.).into()),
            Object2D::Line((1073.5, 0.).into(), (1073.5, 141.).into()),
            Object2D::Line((524.5, 0.).into(), (524.5, 141.).into()),
            Object2D::Line((524.5, 141.).into(), (898.5, 141.).into()),
            Object2D::Line((974.5, 141.).into(), (1073.5, 141.).into()),
            Object2D::Line((1073.5, 821.).into(), (1073.5, 680.).into()),
            Object2D::Line((524.5, 821.).into(), (524.5, 680.).into()),
            Object2D::Line((524.5, 680.).into(), (630.54, 680.).into()),
            Object2D::Line((706.54, 680.).into(), (1073.5, 680.).into()),
            Object2D::RectangleFour(
                (1069.98, 528.785).into(),
                (930.74, 145.81).into(),
                (528.77, 292.22).into(),
                (667.26, 675.20).into(),
            ),
        ],
    ));
    let init_state = KinematicState {
        angle: noise_angle.sample(&mut rng),
        position: Point {
            x: (noise_x.sample(&mut rng)),
            y: (noise_y.sample(&mut rng)),
        },
        vel_angle: 0.,
        velocity: Point { x: 0., y: 0. },
    };
    let mut robot_state = KinematicState {
        angle: 0.,
        position: Point { x: 10., y: 410.5 },
        vel_angle: 0.,
        velocity: Point { x: 0., y: 0. },
    };
    let mut filter = LocalizationFilter::new(
        Matrix6::from_diagonal(&Vector6::new(
            ANGLE_NOISE.powi(2),
            X_NOISE.powi(2),
            Y_NOISE.powi(2),
            0.,
            0.,
            0.,
        )),
        init_state.into(),
        Config::default(),
    );
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
    let mut lidar = DummyLidar::new(
        sensor_map.clone(),
        robot_state.pose(),
        Normal::new(0., 1.),
        Normal::new(0., 0.005),
        180,
        Pose::default(),
        None,
    );
    let mut object_sensor = DummyObjectSensor3D::new(
        CAMERA_FOV,
        sensor_map.clone(),
        Pose::default(),
        robot_state.pose(),
        Pose3D {
            angle: Point { x: 0.05, y: 0. },
            position: Point3D {
                x: 1.,
                y: 1.,
                z: 0.5,
            },
        },
        VISION_MAX_DIST,
    );
    let mut mcl = {
        let particle_count = 1_000;
        let weight_sum_threshold = 200.;
        let death_threshold = DeathCondition {
            particle_count_threshold: 4_000,
            particle_concentration_threshold: 300.,
        };
        PoseMCL::from_distributions(
            (
                Uniform::new(0., 0. + 1e-3),
                (Uniform::new(10., 10. + 1e-3), Uniform::new(0., 821.)),
            ),
            particle_count,
            weight_sum_threshold,
            death_threshold,
            sensor_map.clone(),
            presets::exp_weight(1.05),
            presets::lidar_error(1.2, 1.),
            presets::object_3d_detection_error(1.2, 20., 1.),
            presets::uniform_resampler(0.001, 0.7),
        )
    };

    let map_visual_margins: Point = (25., 25.).into();
    let mut window: PistonWindow = WindowSettings::new("😎", [1000, 1000])
        .exit_on_esc(true)
        .build()
        .unwrap();
    let mut tick: u32 = 0;
    let control_noise_angle = Normal::new(0., CONTROL_ANGLE_NOISE);
    let control_noise_x = Normal::new(0., CONTROL_X_NOISE);
    let control_noise_y = Normal::new(0., CONTROL_Y_NOISE);
    let mut kalman_error: Pose = Pose::default();
    let mut mcl_error: Pose = Pose::default();
    let mut last_time: Instant = Instant::now();
    let mut delta_t;
    let start = Instant::now();
    let mut wheel_control = Point::default();
    let mut wheel_robot_state = DifferentialDriveState::default();
    wheel_robot_state.set_wheel_dist(WHEEL_DIST);
    while let Some(e) = window.next() {
        let mut control: Pose = Pose::default();
        delta_t = last_time.elapsed().as_secs_f64();
        last_time = Instant::now();
        println!("{}", delta_t);
        if tick > 10000 {
            let elapsed = start.elapsed();
            println!(
                "{}t in {:?}, {}tps",
                tick,
                elapsed,
                tick as f64 / elapsed.as_secs_f64()
            );
            break;
        }
        println!("T = {}", tick);
        // User input

        if let Some(Button::Keyboard(key)) = e.press_args() {
            match key {
                keyboard::Key::W => {
                    wheel_control.x = ROBOT_ACCEL;
                }
                keyboard::Key::S => {
                    wheel_control.x = -ROBOT_ACCEL;
                }
                keyboard::Key::Up => wheel_control.y = ROBOT_ACCEL,
                keyboard::Key::Down => wheel_control.y = -ROBOT_ACCEL,
                keyboard::Key::Space => {
                    wheel_control = Point::default();
                    control += Pose {
                        angle: -robot_state.vel_angle / delta_t,
                        position: Point {
                            x: Point { x: 1., y: 0. }
                                .rotate(robot_state.angle)
                                .dot(robot_state.velocity)
                                / delta_t,
                            y: Point { x: 1., y: 0. }
                                .rotate(robot_state.angle - FRAC_PI_2)
                                .dot(robot_state.velocity)
                                / delta_t,
                        },
                    };
                    wheel_robot_state.reset_velocity();
                }
                _ => (),
            }
        }
        if let Some(Button::Keyboard(key)) = e.release_args() {
            match key {
                keyboard::Key::W => {
                    wheel_control.x = 0.;
                }
                keyboard::Key::S => {
                    wheel_control.x = 0.;
                }
                keyboard::Key::Up => wheel_control.y = 0.,
                keyboard::Key::Down => wheel_control.y = 0.,

                _ => (),
            }
        }
        let filter_prediction: KinematicState = filter.known_state.into();

        // Rendering
        window.draw_2d(&e, |c, g, _device| {
            clear([1.0; 4], g);
            draw_map(
                marker_map.clone(),
                [0., 0., 1., 1.],
                [0., 0.5, 0., 1.],
                5.,
                1.,
                MAP_SCALE,
                map_visual_margins,
                c.transform,
                g,
            );
            draw_map(
                real_map.clone(),
                [1., 0., 1., 1.],
                [1., 0.5, 0., 1.],
                5.,
                1.,
                MAP_SCALE,
                map_visual_margins,
                c.transform,
                g,
            );
            draw_map(
                sensor_map.clone(),
                [0., 0., 0., 1.],
                [1., 0.5, 0., 1.],
                5.,
                1.,
                MAP_SCALE,
                map_visual_margins,
                c.transform,
                g,
            );

            point_cloud(
                &lidar
                    .sense()
                    .iter()
                    .map(|p| p.rotate(robot_state.angle))
                    .collect::<Vec<_>>(),
                [0., 1., 0., 1.],
                1.,
                MAP_SCALE,
                map_visual_margins + robot_state.position * MAP_SCALE,
                c.transform,
                g,
            );
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
                robot_state.pose(),
                c.transform,
                g,
            );
            isoceles_triangle(
                [1., 1., 1., 1.],
                map_visual_margins,
                MAP_SCALE,
                0.5,
                mcl.get_prediction(),
                c.transform,
                g,
            );
            let filter_prediction: KinematicState = filter.known_state.into();
            isoceles_triangle(
                [0., 0.75, 0., 1.],
                map_visual_margins,
                MAP_SCALE,
                0.5,
                filter_prediction.pose(),
                c.transform,
                g,
            );
        });
        let control_noise = Pose {
            angle: control_noise_angle.sample(&mut rng),
            position: (
                control_noise_x.sample(&mut rng),
                control_noise_y.sample(&mut rng),
            )
                .into(),
        };
        control += wheel_robot_state.control_update(wheel_control, delta_t, robot_state.angle);

        if robot_state.mapped_control_update(control, delta_t, &real_map) {
            wheel_robot_state.reset_velocity();
            control.angle = -robot_state.vel_angle / delta_t + control.angle;
            control.position = Point {
                x: Point { x: 1., y: 0. }
                    .rotate(robot_state.angle)
                    .dot(robot_state.velocity)
                    / delta_t
                    + control.position.x,
                y: Point { x: 1., y: 0. }
                    .rotate(robot_state.angle - FRAC_PI_2)
                    .dot(robot_state.velocity)
                    / delta_t
                    + control.position.y,
            };
            robot_state.vel_angle = 0.;
            robot_state.velocity = Point::default();
        };
        control += control_noise;

        // update sensors
        motion_sensor.update_pose(Pose {
            angle: robot_state.vel_angle,
            position: robot_state.velocity,
        });
        motion_sensor.set_delta_t(delta_t);
        let robot_pose = robot_state.pose();
        position_sensor.update_pose(robot_pose);
        lidar.update_pose(robot_pose);
        object_sensor.update_pose(robot_pose);

        println!("\tP = {}", mcl.belief.len());
        // update localization
        let mcl_pred = mcl.get_prediction();

        mcl.control_update(&position_sensor);
        mcl.observation_update(&lidar, &object_sensor);

        q = delta_t.powi(2)
            * Matrix6::from_diagonal(&Vector6::new(
                0.00000,
                0.00000,
                0.00000,
                CONTROL_ANGLE_NOISE.powi(2),
                CONTROL_X_NOISE.powi(2),
                CONTROL_Y_NOISE.powi(2),
            ));
        filter.prediction_update(delta_t, control.into(), q);

        let motion_measurements = motion_sensor.sense();
        let mcl_prediction = mcl.get_prediction();

        let mcl_uncertainty = variance_poses(
            &mcl.belief
                .clone()
                .into_iter()
                .choose_multiple(&mut rng, 500),
        );
        r = Matrix6::from_diagonal(&Vector6::from_vec(vec![
            mcl_uncertainty.angle,
            mcl_uncertainty.position.x,
            mcl_uncertainty.position.y,
            delta_t.powi(2) * ROTATIONAL_VELOCITY_SENSOR_NOISE.powi(2),
            delta_t.powi(2) * VELOCITY_X_SENSOR_NOISE.powi(2),
            delta_t.powi(2) * VELOCITY_Y_SENSOR_NOISE.powi(2),
        ]));

        filter.measurement_update(
            RowVector6::from_vec(vec![
                mcl_prediction.angle,
                mcl_prediction.position.x,
                mcl_prediction.position.y,
                motion_measurements.angle,
                motion_measurements.position.x,
                motion_measurements.position.y,
            ]),
            r,
        );

        tick += 1;

        kalman_error += Pose {
            angle: (filter_prediction.pose().angle - robot_state.pose().angle).powi(2),
            position: Point {
                x: (filter_prediction.pose().position.x - robot_state.pose().position.x).powi(2),
                y: (filter_prediction.pose().position.y - robot_state.pose().position.y).powi(2),
            },
        };
        mcl_error += Pose {
            angle: (mcl_pred.angle - robot_state.pose().angle).powi(2),
            position: Point {
                x: (mcl_pred.position.x - robot_state.pose().position.x).powi(2),
                y: (mcl_pred.position.y - robot_state.pose().position.y).powi(2),
            },
        };
    }
    println!(
        "KALMAN: {:?} \n MCL: {:?}\n\n",
        Pose {
            angle: kalman_error.angle.sqrt() / tick as f64,
            position: (
                kalman_error.position.x.sqrt() / tick as f64,
                kalman_error.position.y.sqrt() / tick as f64
            )
                .into()
        },
        Pose {
            angle: mcl_error.angle.sqrt() / tick as f64,
            position: (
                mcl_error.position.x.sqrt() / tick as f64,
                mcl_error.position.y.sqrt() / tick as f64
            )
                .into()
        }
    );
}
