use global_robot_localization::{
    ai::{
        kalman_filter::KalmanFilter,
        localization::{DeathCondition, PoseMCL},
        presets,
    },
    map::{Map2D, Object2D},
    replay::{draw_map, isoceles_triangle, point_cloud},
    sensors::{
        dummy::{DummyLidar, DummyPositionSensor, DummyVelocitySensor},
        Sensor,
    },
    utility::{KinematicState, Point, Pose},
};
use nalgebra::{Matrix6, Vector6};
use piston_window::*;
use rand::{
    distributions::{Distribution, Normal},
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
const X_MCL_NOISE: f64 = 1.;
const Y_MCL_NOISE: f64 = 1.;
const ANGLE_MCL_NOISE: f64 = 0.05;
const CONTROL_X_NOISE: f64 = 0.0005 / 10.;
const CONTROL_Y_NOISE: f64 = 0.0005 / 10.;
const CONTROL_ANGLE_NOISE: f64 = 0.000005 / 10.;
const VELOCITY_X_SENSOR_NOISE: f64 = 0.0005;
const VELOCITY_Y_SENSOR_NOISE: f64 = 0.0005;
const ROTATIONAL_VELOCITY_SENSOR_NOISE: f64 = 0.0005;
const MAP_SCALE: f64 = 2.;
const ROBOT_ACCEL: f64 = 3. / 1000.;
const ROBOT_ANGLE_ACCEL: f64 = 0.1 / 1000.;

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
    let percieved_map = Arc::new(Map2D::with_size(
        (200., 200.).into(),
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
    let real_map = Arc::new(Map2D::with_size(
        (200., 200.).into(),
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
            Object2D::Triangle(((10., 10.).into(), (30., 30.).into(), (40., 20.).into())),
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
        Normal::new(0., 0.01),
        Normal::new(0., 0.01),
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
        PoseMCL::new(
            particle_count,
            weight_sum_threshold,
            death_threshold,
            percieved_map.clone(),
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
    let mut mcl_error: Pose = Pose::default();
    let last_time: Instant = Instant::now();
    let mut delta_t;
    let start = Instant::now();
    while let Some(e) = window.next() {
        delta_t = last_time.elapsed().as_secs_f64();
        if tick >= 500 {
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
        let mut control = Pose::default();
        if let Some(Button::Keyboard(key)) = e.press_args() {
            match key {
                keyboard::Key::W => {
                    control.position.x += ROBOT_ACCEL / delta_t;
                }
                keyboard::Key::S => {
                    control.position.x -= ROBOT_ACCEL / delta_t;
                }
                keyboard::Key::A => control.angle -= ROBOT_ANGLE_ACCEL / delta_t,
                keyboard::Key::D => control.angle += ROBOT_ANGLE_ACCEL / delta_t,

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
                percieved_map.clone(),
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
        robot_state.position.x += robot_state.velocity.x * delta_t;
        robot_state.position.y += robot_state.velocity.y * delta_t;
        robot_state.velocity.x += (control.position.x * robot_state.angle.cos()
            + control.position.y * (robot_state.angle - FRAC_PI_2).cos())
            * delta_t;
        robot_state.velocity.y += (control.position.x * robot_state.angle.sin()
            + control.position.y * (robot_state.angle - FRAC_PI_2).sin())
            * delta_t;
        robot_state.angle = (robot_state.angle + robot_state.vel_angle * delta_t) % (2. * PI);
        robot_state.vel_angle += control.angle * delta_t;
        let temp = robot_state.clone();
        robot_state = temp
            .clamp_control_update(Range {
                start: Point { x: 0.1, y: 0.1 },
                end: Point { x: 199.9, y: 199. },
            })
            .into();

        control.angle -= (robot_state.vel_angle - temp.vel_angle) / delta_t;
        control.position.x -= (robot_state.velocity.x - temp.velocity.x) / delta_t;
        control.position.y -= (robot_state.velocity.y - temp.velocity.y) / delta_t;

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
        let robot_pose = robot_state.pose();
        position_sensor.update_pose(robot_pose);
        lidar.update_pose(robot_pose);

        println!("\tP = {}", mcl.belief.len());
        // update localization
        let mcl_pred = mcl.get_prediction();

        mcl.control_update(&position_sensor);
        mcl.observation_update(&lidar);
        filter.prediction_update(delta_t, control);
        filter.measurement_update(motion_sensor.sense(), mcl.get_prediction());

        tick += 1;

        kalman_error += Pose {
            angle: (filter_prediction.pose().angle - robot_state.pose().angle).abs(),
            position: Point {
                x: (filter_prediction.pose().position.x - robot_state.pose().position.x).abs(),
                y: (filter_prediction.pose().position.y - robot_state.pose().position.y).abs(),
            },
        };
        mcl_error += Pose {
            angle: (mcl_pred.angle - robot_state.pose().angle).abs(),
            position: Point {
                x: (mcl_pred.position.x - robot_state.pose().position.x).abs(),
                y: (mcl_pred.position.y - robot_state.pose().position.y).abs(),
            },
        };
        if tick % 1000 == 0 {
            println!(
                "KALMAN: {:?} \n MCL: {:?}\n\n",
                kalman_error / tick as f64,
                mcl_error / tick as f64
            );
        }
    }
}
