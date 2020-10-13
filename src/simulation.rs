use global_robot_localization::{
    ai::{
        kalman_filter::{Config, KalmanFilter, LocalizationFilter},
        localization::{DeathCondition, PoseMCL},
        presets,
    },
    map::{Map2D, Object2D},
    replay::render::{draw_map, isoceles_triangle, point_cloud},
    sensors::{
        dummy::{DummyLidar, DummyPositionSensor, DummyVelocitySensor},
        Sensor,
    },
    utility::{KinematicState, Point, Pose},
};
use nalgebra::{Matrix6, RowVector6, Vector6};
use piston_window::*;
use rand::{
    distributions::{uniform::Uniform, Distribution, Normal},
    thread_rng,
};
use std::{
    f64::consts::{FRAC_PI_2, FRAC_PI_8, PI},
    ops::Range,
    sync::Arc,
    time::Instant,
};

const ANGLE_NOISE: f64 = 0.;
const X_NOISE: f64 = 25.;
const Y_NOISE: f64 = 3.;
const X_MCL_NOISE: f64 = 5. / 300.; // 10 cm
const Y_MCL_NOISE: f64 = 5. / 300.; // 10 cm
const ANGLE_MCL_NOISE: f64 = 0.1; // 3 cm
const CONTROL_X_NOISE: f64 = 0.01; // 3 cm / s^2 of noise
const CONTROL_Y_NOISE: f64 = 0.01; // 3 cm / s^2 of noise

const CONTROL_ANGLE_NOISE: f64 = 0.007; // 2 degrees / s^2 of noise
const VELOCITY_X_SENSOR_NOISE: f64 = 1.;
const VELOCITY_Y_SENSOR_NOISE: f64 = 1.;
const ROTATIONAL_VELOCITY_SENSOR_NOISE: f64 = 0.1;
const MAP_SCALE: f64 = 2.;
const ROBOT_ACCEL: f64 = 100.;
const ROBOT_ANGLE_ACCEL: f64 = 10.;

fn main() {
    let mut rng = thread_rng();

    let mut q: Matrix6<f64>;

    let mut r: Matrix6<f64>;

    let noise_x = Normal::new(100., X_NOISE);
    let noise_angle = Normal::new(FRAC_PI_2, ANGLE_NOISE);
    let noise_y = Normal::new(8., Y_NOISE);
    let perceived_map = Arc::new(Map2D::with_size(
        (200., 200.).into(),
        vec![
            Object2D::Line((0., 0.).into(), (200., 0.).into()),
            // Object2D::Line((0., 0.).into(), (0., 200.).into()),
            // Object2D::Line((200., 0.).into(), (200., 200.).into()),
            Object2D::Line((0., 200.).into(), (200., 200.).into()),
            Object2D::Line((20., 60.).into(), (50., 100.).into()),
            Object2D::Line((50., 100.).into(), (80., 50.).into()),
            Object2D::Line((80., 50.).into(), (20., 60.).into()),
            Object2D::Line((140., 100.).into(), (180., 120.).into()),
            Object2D::Line((180., 120.).into(), (160., 90.).into()),
            Object2D::Line((160., 90.).into(), (140., 100.).into()),
            Object2D::Line((100., 40.).into(), (160., 80.).into()),
        ],
    ));

    let real_map = Arc::new(Map2D::with_size(
        (200., 200.).into(),
        vec![
            Object2D::Line((0., 0.).into(), (200., 0.).into()),
            // Object2D::Line((0., 0.).into(), (0., 200.).into())),
            // Object2D::Line((200., 0.).into(), (200., 200.).into())),
            Object2D::Line((0., 200.).into(), (200., 200.).into()),
            Object2D::Line((20., 60.).into(), (50., 100.).into()),
            Object2D::Line((50., 100.).into(), (80., 50.).into()),
            Object2D::Line((80., 50.).into(), (20., 60.).into()),
            Object2D::Line((140., 100.).into(), (180., 120.).into()),
            Object2D::Line((180., 120.).into(), (160., 90.).into()),
            Object2D::Line((160., 90.).into(), (140., 100.).into()),
            Object2D::Line((100., 40.).into(), (160., 80.).into()),
            Object2D::Triangle((10., 10.).into(), (30., 30.).into(), (40., 20.).into()),
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
        angle: FRAC_PI_2,
        position: Point { x: 100., y: 8. },
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
        real_map.clone(),
        robot_state.pose(),
        Normal::new(0., 0.001),
        Normal::new(0., 0.0005),
        180,
        Pose::default(),
        None,
    );
    let mut mcl = {
        let particle_count = 40_000;
        let weight_sum_threshold = 200.;
        let death_threshold = DeathCondition {
            particle_count_threshold: 4_000,
            particle_concentration_threshold: 300.,
        };
        PoseMCL::from_distributions(
            (
                Uniform::new(FRAC_PI_2, FRAC_PI_2 + 1e-3),
                (Uniform::new(0., 200.), Uniform::new(8., 8. + 1e-3)),
            ),
            particle_count,
            weight_sum_threshold,
            death_threshold,
            perceived_map.clone(),
            presets::exp_weight(1.05),
            presets::lidar_error(1.2, 1.),
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
    let mut mcl_kalman_difference: Pose = Pose::default();
    let mut mcl_error: Pose = Pose::default();
    let mut last_time: Instant = Instant::now();
    let mut delta_t;
    let start = Instant::now();
    let mut sensor_noise = RowVector6::from_vec(vec![0.01; 6]);
    let mut control = Pose::default();
    while let Some(e) = window.next() {
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
                    control.position.x = ROBOT_ACCEL;
                }
                keyboard::Key::S => {
                    control.position.x = -ROBOT_ACCEL;
                }
                keyboard::Key::D => control.angle = ROBOT_ANGLE_ACCEL,
                keyboard::Key::A => control.angle = -ROBOT_ANGLE_ACCEL,

                _ => (),
            }
        }
        if let Some(Button::Keyboard(key)) = e.release_args() {
            match key {
                keyboard::Key::W => {
                    control.position.x = 0.;
                }
                keyboard::Key::S => {
                    control.position.x = 0.;
                }
                keyboard::Key::D => control.angle = 0.,
                keyboard::Key::A => control.angle = 0.,

                _ => (),
            }
        }
        let filter_prediction: KinematicState = filter.known_state.into();

        // Rendering
        window.draw_2d(&e, |c, g, _device| {
            clear([1.0; 4], g);
            draw_map(
                real_map.clone(),
                [1., 0., 1., 1.],
                0.5,
                1.,
                MAP_SCALE,
                map_visual_margins,
                c.transform,
                g,
            );
            draw_map(
                perceived_map.clone(),
                [0., 0., 0., 1.],
                0.5,
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
                [0., 1., 0., 1.],
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
        robot_state.control_update(control, delta_t, &perceived_map);
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

        println!("\tP = {}", mcl.belief.len());
        // update localization
        let mcl_pred = mcl.get_prediction();

        mcl.control_update(&position_sensor);
        mcl.observation_update(&lidar);
        q = delta_t.powi(2)
            * Matrix6::from_diagonal(&Vector6::new(
                0.00000,
                0.00000,
                0.00000,
                CONTROL_ANGLE_NOISE.powi(2),
                CONTROL_X_NOISE.powi(2),
                CONTROL_Y_NOISE.powi(2),
            ));
        filter.prediction_update(delta_t, control.into(), q, &perceived_map);

        let motion_sensor = motion_sensor.sense();
        let mcl_prediction = mcl.get_prediction();
        r = Matrix6::from_diagonal(&Vector6::from_vec(vec![
            delta_t.powi(2) * ROTATIONAL_VELOCITY_SENSOR_NOISE.powi(2),
            delta_t.powi(2) * VELOCITY_X_SENSOR_NOISE.powi(2),
            delta_t.powi(2) * VELOCITY_Y_SENSOR_NOISE.powi(2),
            ANGLE_MCL_NOISE.powi(2),
            X_MCL_NOISE.powi(2),
            Y_MCL_NOISE.powi(2),
        ]));
        filter.measurement_update(
            RowVector6::from_vec(vec![
                mcl_prediction.angle,
                mcl_prediction.position.x,
                mcl_prediction.position.y,
                motion_sensor.angle,
                motion_sensor.position.x,
                motion_sensor.position.y,
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
        mcl_kalman_difference += Pose {
            angle: (mcl_pred.angle - filter_prediction.pose().position.x).powi(2),
            position: Point {
                x: (mcl_pred.position.x - filter_prediction.pose().position.x).powi(2),
                y: (mcl_pred.position.y - filter_prediction.pose().position.y).powi(2),
            },
        };

        sensor_noise = RowVector6::from_vec(vec![
            delta_t,
            delta_t,
            delta_t,
            25. / mcl_kalman_difference.angle,
            1. / mcl_kalman_difference.position.x,
            1. / mcl_kalman_difference.position.y,
        ]);
        mcl_kalman_difference = Pose::default();

        if tick % 1000 == 0 {
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
    }
}
