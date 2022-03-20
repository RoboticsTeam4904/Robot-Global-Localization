use global_robot_localization::{
    ai::{
        kalman_filter::{Config, KalmanFilter, LocalizationFilter},
        localization::{DeathCondition, PoseMCL},
        presets,
    },
    map::{Map2D, Object2D},
    replay::{
        graph,
        render::{draw_map, isoceles_triangle, point_cloud},
    },
    sensors::{
        dummy::{DummyLidar, DummyObjectSensor3D, DummyPositionSensor, DummyVelocitySensor},
        Sensor,
    },
    utility::{variance_poses, KinematicState, Point, Pose, Pose3D},
};
use nalgebra::{Matrix6, RowVector6, Vector6};
use piston_window::*;
use rand::{
    distributions::{uniform::Uniform, Distribution},
    seq::IteratorRandom,
    thread_rng,
};
use rand_distr::Normal;
use std::{
    f64::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_8, PI},
    sync::Arc,
    time::{Duration, Instant},
};

const ANGLE_NOISE: f64 = 0.;

const X_NOISE: f64 = 3.;

const Y_NOISE: f64 = 274.;

const CONTROL_X_NOISE: f64 = 3.;

const CONTROL_Y_NOISE: f64 = 3.;

const CONTROL_ANGLE_NOISE: f64 = 0.035; // 2 degrees / s^2 of noise

const VELOCITY_X_SENSOR_NOISE: f64 = 3.;

const VELOCITY_Y_SENSOR_NOISE: f64 = 3.;

const ROTATIONAL_VELOCITY_SENSOR_NOISE: f64 = 0.035;

/// Scale of map for visualization
const MAP_SCALE: f64 = 0.6;

/// Robot acceleration on keypress
const ROBOT_ACCEL: f64 = 400.;

/// Distance between the wheels for differential drive
const WHEEL_DIST: f64 = 120.;

/// Height of the camera on the robot.
const CAMERA_HEIGHT: f64 = 0.;

/// Relative angle of the camera line of sight and robot line of sight.
const CAMERA_ANGLE: f64 = 0.;

/// FOV of vision camera (horizontally and vertically, respectively).
const CAMERA_FOV: Point = Point {
    x: 1.229,
    y: 0.7557,
};

/// The maximum range of the camera.
const VISION_MAX_DIST: Option<f64> = Some(800.);

/// Dynamic friction of the robot with respect to the field (μ).
const FRICTION_COEFFICIENT: f64 = 0.1;

/// Gravity (cm/s^2)
const GRAVITY: f64 = 980.;

/// Differential drive state for the robot
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct DifferentialDriveState {
    pub wheel_dist: f64,
    pub velocity: Point,
    pub robot_velocity: Pose,
    pub control: Point,
    friction_coefficient: f64,
}

impl DifferentialDriveState {
    /// return velocity for the robot, given wheel velocities
    pub fn robot_velocity(&self, delta_t: f64, robot_angle: f64) -> Pose {
        let omega = (self.velocity.x - self.velocity.y) / self.wheel_dist;
        let d_angle = omega * delta_t;
        let r = (self.velocity.y + self.velocity.x) / 2.;
        Pose {
            angle: omega,
            position: Point {
                x: r * d_angle.cos(),
                y: r * d_angle.sin(),
            }
            .rotate(-robot_angle),
        }
    }
    /// Returns the appropriate control update for the given wheel control.
    /// Incorporates wheel friction to acceleration.
    pub fn control_update(&mut self, delta_t: f64, robot_angle: f64) -> Pose {
        self.velocity += self.control * delta_t;
        self.velocity = Point {
            x: (self.velocity.x.abs() - GRAVITY * self.friction_coefficient * delta_t).max(0.)
                * self.velocity.x.signum(),
            y: (self.velocity.y.abs() - GRAVITY * self.friction_coefficient * delta_t).max(0.)
                * self.velocity.y.signum(),
        };
        let new_velocity = self.robot_velocity(delta_t, robot_angle);
        let diff_vel = new_velocity - self.robot_velocity;
        self.robot_velocity = new_velocity;

        Pose {
            angle: diff_vel.angle / delta_t,
            position: diff_vel.position.rotate(robot_angle) / delta_t,
        }
    }

    pub fn reset_velocity(&mut self) {
        self.velocity = Point::default();
        self.robot_velocity = Pose::default();
    }

    /// Change the wheel distance of the differential drive
    pub fn with_wheel_dist(mut self, wheel_dist: f64) -> Self {
        self.wheel_dist = wheel_dist;
        self
    }

    /// Change the wheel friction in the differential drive
    pub fn with_friction(mut self, friction_coefficient: f64) -> Self {
        self.friction_coefficient = friction_coefficient;
        self
    }
}

/// Dummy robots on the field that follow semi-random motion.
/// Simulates LIDAR and vision noise caused by map uncertainty.
#[derive(Clone)]
struct DummyRobot {
    left_noise_distr: Normal<f64>,
    right_noise_distr: Normal<f64>,
    robot_state: KinematicState,
    wheel_state: DifferentialDriveState,
    radius: f64,
}

impl DummyRobot {
    pub fn new(
        noise_margins: Point,
        robot_state: KinematicState,
        radius: f64,
        wheel_state: DifferentialDriveState,
    ) -> Self {
        DummyRobot {
            left_noise_distr: Normal::new(0., noise_margins.x).unwrap(),
            right_noise_distr: Normal::new(0., noise_margins.y).unwrap(),
            robot_state,
            wheel_state,
            radius,
        }
    }

    pub fn with_robot_state(mut self, robot_state: KinematicState) -> Self {
        self.robot_state = robot_state;
        self
    }

    pub fn shape(&self) -> Object2D {
        let forward_point = Point {
            x: self.radius,
            y: 0.,
        }
        .rotate(self.robot_state.angle);
        Object2D::Triangle(
            self.robot_state.position + forward_point * 1.5,
            self.robot_state.position + forward_point.rotate(FRAC_PI_3),
            self.robot_state.position + forward_point.rotate(-FRAC_PI_3),
        )
    }
    pub fn control_update(&mut self, delta_t: f64, map: &Arc<Map2D>) {
        let mut rng = thread_rng();
        let control = self
            .wheel_state
            .control_update(delta_t, self.robot_state.angle);
        self.robot_state
            .mapped_control_update(control, delta_t, vec![map.clone()]);
        if self.robot_state.vel_angle == 0. && self.robot_state.velocity == Point::default() {
            self.wheel_state.control = (-5., 10.).into();
        } else {
            self.wheel_state.control += Point {
                x: self.left_noise_distr.sample(&mut rng),
                y: self.right_noise_distr.sample(&mut rng),
            } * delta_t;
        }
    }
}
fn main() {
    let mut rng = thread_rng();

    // Kalman filter error matrices for control and sensor data, respectively.
    let mut q: Matrix6<f64>;
    let mut r: Matrix6<f64>;

    let noise_x = Normal::new(8., X_NOISE).unwrap();
    let noise_angle = Normal::new(0., ANGLE_NOISE).unwrap();
    let noise_y = Normal::new(410.5, Y_NOISE).unwrap();

    let initial_positions = vec![
        Point {
            x: 10.,
            y: 821. / 3.,
        },
        Point {
            x: 10.,
            y: 821. / 3.,
        },
        Point {
            x: 10.,
            y: 821. / 3.,
        },
    ];
    // Correct robot state, initializated
    let mut robot_state = KinematicState {
        angle: 0.,
        position: Point { x: 10., y: 410.5 },
        vel_angle: 0.,
        velocity: Point { x: 0., y: 0. },
    };

    // Initialize dummy robots placed around the field.
    let dummy_robot = DummyRobot::new(
        (20., 20.).into(),
        KinematicState::default(),
        15.,
        DifferentialDriveState::default()
            .with_wheel_dist(WHEEL_DIST)
            .with_friction(FRICTION_COEFFICIENT),
    );
    let mut dummy_robots: Vec<DummyRobot> = vec![
        dummy_robot.clone().with_robot_state(
            KinematicState::default()
                .with_position((415., 415.).into())
                .with_angle(FRAC_PI_2),
        ),
        dummy_robot.clone().with_robot_state(
            KinematicState::default()
                .with_position((1200., 390.).into())
                .with_angle(0.),
        ),
        dummy_robot.clone().with_robot_state(
            KinematicState::default()
                .with_position((1350., 700.).into())
                .with_angle(PI),
        ),
        dummy_robot.clone().with_robot_state(
            KinematicState::default()
                .with_position((750., 250.).into())
                .with_angle(FRAC_PI_3),
        ),
        dummy_robot.clone().with_robot_state(
            KinematicState::default()
                .with_position((350., 700.).into())
                .with_angle(-FRAC_PI_2),
        ),
    ];

    // The objects that are known by both the LIDAR and are present in the real world
    let universal_objects = vec![
        // Terminals
        Object2D::Line((1480., 0.).into(), (1646., 166.25).into()),
        Object2D::Line((1660, 8229).into(), (0, 6568.5).into()),
        // Hangar Posts
        Object2D::Rectangle((20.3, 264.1).into(), (50.78, 294.58).into()),
        Object2D::Rectangle((296.8, 264.1).into(), (327.28, 294.58).into()),
        Object2D::Rectangle((20.3, 20.0).into(), (50.78, 50.48).into()),
        Object2D::Rectangle((296.8, 20.0).into(), (327.28, 50.48).into()),
        Object2D::Rectangle((1349.5, 802.6).into(), (1379.98, 833.08).into()),
        Object2D::Rectangle((1626.0, 802.6).into(), (1656.48, 833.08).into()),
        Object2D::Rectangle((1349.5, 558.5).into(), (1379.98, 588.98).into()),
        Object2D::Rectangle((1626.0, 558.5).into(), (1656.48, 588.98).into()),
        // Hub
        Object2D::RectangleFour(
            (691.1, 448.9).into(),
            (706.6, 483.6).into(),
            (955.1, 373.0).into(),
            (939.6, 338.3).into(),
        ),
        Object2D::RectangleFour(
            (861.0, 542.9).into(),
            (895.8, 527.5).into(),
            (785.1, 279.0).into(),
            (750.4, 294.4).into(),
        ),
        Object2D::RectangleFour(
            (712.9, 460.1).into(),
            (871.8, 521.1).into(),
            (932.6, 362.2).into(),
            (773.9, 301.2).into(),
        ),
    ];
    //
    let init_state = KinematicState {
        angle: noise_angle.sample(&mut rng),
        position: Point {
            x: (noise_x.sample(&mut rng)),
            y: (noise_y.sample(&mut rng)),
        },
        vel_angle: 0.,
        velocity: Point { x: 0., y: 0. },
    };

    let mut filter = LocalizationFilter::new(
        Matrix6::from_diagonal(&Vector6::from_iterator(vec![
            ANGLE_NOISE.powi(2),
            X_NOISE.powi(2),
            Y_NOISE.powi(2),
            0.,
            0.,
            0.,
        ])),
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
            position: Point { x: 2., y: 2. },
        },
    );
    let mut lidar = DummyLidar::new(
        sensor_map.clone(),
        robot_state.pose(),
        Normal::new(0., (400 as f64).powi(-2)).unwrap(),
        Normal::new(0., 0.05).unwrap(),
        180,
        Duration::from_millis(100),
        Pose::default(),
        Some(0.0..800.),
    );
    let mut object_sensor = DummyObjectSensor3D::new(
        CAMERA_FOV,
        sensor_map.clone(),
        Pose::default(),
        robot_state.pose(),
        Pose3D {
            angle: Point { x: 0.05, y: 0. },
            position: [(400 as f64).powi(-2); 3].into(),
        },
        VISION_MAX_DIST,
    );
    let mut mcl = {
        let particle_count = 300;
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
            perceived_map.clone(),
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

    let control_noise_angle = Normal::new(0., CONTROL_ANGLE_NOISE).unwrap();
    let control_noise_x = Normal::new(0., CONTROL_X_NOISE).unwrap();
    let control_noise_y = Normal::new(0., CONTROL_Y_NOISE).unwrap();

    // The robot pose created by simply following the control updates,
    //  with noise, every time step.
    let mut dead_reckoning = robot_state.pose();

    let mut dead_reckoning_error: Pose = Pose::default();
    let mut kalman_error: Pose = Pose::default();
    let mut mcl_error: Pose = Pose::default();
    let mut all_errors = vec![];

    let mut last_time: Instant = Instant::now();
    let mut delta_t;

    let start = Instant::now();

    // Create a differential drive state which takes keypress inputs for acceleration
    // As this state is updated, the velocity of the differential drive state
    // will imply an acceleration for the normal control state. This state,
    // robot_state, is rendered as well as predicted by the Kalman Filter.
    let mut wheel_robot_state = DifferentialDriveState::default()
        .with_wheel_dist(WHEEL_DIST)
        .with_friction(FRICTION_COEFFICIENT);

    while let Some(e) = window.next() {
        let mut control: Pose = Pose::default();
        delta_t = last_time.elapsed().as_secs_f64();
        last_time = Instant::now();
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
        // User input - based on keyboard inputs, the differential drive state
        // accelerates each wheel
        if let Some(Button::Keyboard(key)) = e.press_args() {
            match key {
                keyboard::Key::W => {
                    wheel_robot_state.control.x = ROBOT_ACCEL;
                }
                keyboard::Key::S => {
                    wheel_robot_state.control.x = -ROBOT_ACCEL;
                }
                keyboard::Key::Up => wheel_robot_state.control.y = ROBOT_ACCEL,
                keyboard::Key::Down => wheel_robot_state.control.y = -ROBOT_ACCEL,
                // Pressing space zeros the velocity and rotational velocity.
                keyboard::Key::Space => {
                    wheel_robot_state.control = Point::default();
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
                    wheel_robot_state.control.x = 0.;
                }
                keyboard::Key::S => {
                    wheel_robot_state.control.x = 0.;
                }
                keyboard::Key::Up => wheel_robot_state.control.y = 0.,
                keyboard::Key::Down => wheel_robot_state.control.y = 0.,

                _ => (),
            }
        }

        // Kalman filter predicted state
        let filter_prediction: KinematicState = filter.known_state.into();

        // The map with all of the dummy robots, for the sake of testing.
        let dummy_map = Arc::new(Map2D::with_size(
            (1598., 821.).into(),
            dummy_robots
                .iter()
                .map(|dummy_robot| dummy_robot.shape())
                .collect::<Vec<Object2D>>(),
        ));
        lidar.update_with_maps(vec![dummy_map.clone()]);

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
                [1., 0., 1., 0.5],
                [1., 0.5, 0., 0.5],
                5.,
                1.,
                MAP_SCALE,
                map_visual_margins,
                c.transform,
                g,
            );
            draw_map(
                perceived_map.clone(),
                [1., 0., 1., 1.],
                [1., 1., 0., 1.],
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
            draw_map(
                dummy_map.clone(),
                [0., 0., 0., 1.],
                [0.5, 0.5, 0.5, 1.],
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
        // Noise added to observed odometry control data
        let control_noise = Pose {
            angle: control_noise_angle.sample(&mut rng),
            position: (
                control_noise_x.sample(&mut rng),
                control_noise_y.sample(&mut rng),
            )
                .into(),
        };

        // Adding control data to the differential drive state without noise to the state.
        control += wheel_robot_state.control_update(delta_t, robot_state.angle);

        // If the robot passes through the map, set velocity to zero (and the robot
        // control update to make the velocity zero in the next time step).
        if robot_state.mapped_control_update(
            control,
            delta_t,
            vec![real_map.clone(), dummy_map.clone()],
        ) {
            wheel_robot_state.reset_velocity();

            // Set the rotational acceleration to zero the rotational velocity
            // in the next time step
            control.angle = -robot_state.vel_angle / delta_t + control.angle;

            // Set the acceleration to zero the velocity in the next time step.
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

            // The velocity of the real robot gets zeroed. The control acceleration
            // is fed into the localization
            robot_state.vel_angle = 0.;
            robot_state.velocity = Point::default();
        };

        // Add noise to the control data for an odometry sensor input.
        control += control_noise * delta_t;

        // Update sensors

        // Motion sensor that returns a velocity with some error - necessary
        // for Kalman Filter.
        motion_sensor.set_delta_t(delta_t);
        motion_sensor.update_pose(Pose {
            angle: robot_state.vel_angle,
            position: robot_state.velocity,
        });

        // Control updates for the dummy robots.
        let robot_pose = robot_state.pose();
        for dummy_robot in &mut dummy_robots {
            dummy_robot.control_update(delta_t, &real_map);
        }

        // Position sensor that, when sensed, returns the change
        // in position from the previous state to the current state
        // (with error).
        position_sensor.set_delta_t(delta_t);
        position_sensor.update_pose(robot_pose);

        // Lidar sensor that returns distance measurements in all directions,
        // with noise.
        lidar.update_pose(robot_pose);

        // Object sensor that detects the vision tapes, with some noise.
        object_sensor.update_pose(robot_pose);

        println!("\tP = {}", mcl.belief.len());

        // update localization
        dead_reckoning += position_sensor.sense();

        let mcl_pred = mcl.get_prediction();

        // Control update with noisy changes in positions
        mcl.control_update(&position_sensor);

        // Using sensor data in the observation update to compensate
        // for noisy odometry data
        mcl.observation_update(&lidar, &object_sensor);

        // Creating control data covariance matrix with the
        // sensor errors along the diagonals (and no correlation coefficients)
        q = delta_t.powi(2)
            * Matrix6::from_diagonal(&Vector6::from_iterator(vec![
                0.00000,
                0.00000,
                0.00000,
                CONTROL_ANGLE_NOISE.powi(2),
                CONTROL_X_NOISE.powi(2),
                CONTROL_Y_NOISE.powi(2),
            ]));

        // Prediction update using the noisy control data and covariance matrix.
        filter.prediction_update(delta_t, control.into(), q);

        // Take measurements from the velocity data to feed into the Kalman Filter
        let motion_measurements = motion_sensor.sense();

        // Get MCL prediction
        let mcl_prediction = mcl.get_prediction();

        // Get MCL uncertainty to feed into the Kalman Filter
        let mcl_uncertainty = variance_poses(
            &mcl.belief
                .clone()
                .into_iter()
                .choose_multiple(&mut rng, 500),
        );

        // Create a sensor covariance matrix with MCL uncertainty as well as
        // Velocity sensor noise uncertainty.
        r = Matrix6::from_diagonal(&Vector6::from_vec(vec![
            mcl_uncertainty.angle,
            mcl_uncertainty.position.x,
            mcl_uncertainty.position.y,
            delta_t.powi(2) * ROTATIONAL_VELOCITY_SENSOR_NOISE.powi(2),
            delta_t.powi(2) * VELOCITY_X_SENSOR_NOISE.powi(2),
            delta_t.powi(2) * VELOCITY_Y_SENSOR_NOISE.powi(2),
        ]));

        // Measurement update with sensor data
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

        // Calculate the squared error of each of the prediction mechanisms over time,
        // in angle and position.
        tick += 1;
        if tick > 0 {
            let calculate_error = |sum: &mut Pose, pred: Pose| -> Pose {
                let current_error = pred - robot_state.pose();
                *sum += Pose {
                    angle: current_error.angle.powi(2),
                    position: Point {
                        x: current_error.position.x.powi(2),
                        y: current_error.position.y.powi(2),
                    },
                };
                current_error
            };

            let current_kalman_error = calculate_error(&mut kalman_error, filter_prediction.pose());
            let current_mcl_error = calculate_error(&mut mcl_error, mcl_pred);
            let current_dead_reck_error =
                calculate_error(&mut dead_reckoning_error, dead_reckoning);

            all_errors.push((
                start.elapsed().as_secs_f64(),
                vec![
                    current_kalman_error.position.mag(),
                    current_mcl_error.position.mag(),
                    current_dead_reck_error.position.mag(),
                ],
            ));
        }
    }

    // Print the cumulative error of the Kalman Filter, MCL, and Dead Reckoning
    // over time.
    println!(
        "KALMAN: {:?} \nMCL: {:?}\nDEAD RECKONING: {:?}\n",
        Pose {
            angle: kalman_error.angle.sqrt(),
            position: (
                kalman_error.position.x.sqrt(),
                kalman_error.position.y.sqrt()
            )
                .into()
        } / tick as f64,
        Pose {
            angle: mcl_error.angle.sqrt(),
            position: (mcl_error.position.x.sqrt(), mcl_error.position.y.sqrt()).into()
        } / tick as f64,
        Pose {
            angle: dead_reckoning_error.angle.sqrt(),
            position: (
                dead_reckoning_error.position.x.sqrt(),
                dead_reckoning_error.position.y.sqrt(),
            )
                .into()
        } / tick as f64,
    );
    graph::auto_graph(
        all_errors.into_iter().skip(20).collect(),
        "logs/test.png",
        "Error over time",
        "Error (cm)",
        "Time (seconds)",
        vec![
            "Kalman Filter",
            "Monte-Carlo Localization",
            "Dead Reckoning",
        ],
        (2000, 1000),
    )
    .unwrap();
}
