use global_robot_localization::{
    ai::{
        kalman_filter::{Config, KalmanFilter, LocalizationFilter},
        localization::{DeathCondition, PoseMCL},
        presets::{exp_weight, lidar_error, object_3d_detection_error, uniform_resampler},
    },
    map::{Map2D, Object2D},
    sensors::{
        dummy::{DummyLimitedSensor, DummySensor},
        log::{LogSensor, LogSensorSink},
        rplidar::RplidarSensor,
        udp::{UDPSensor, UDPSensorSink},
        DeltaSensor, Sensor, SensorSink, WrappableSensor,
    },
    utility::{variance_poses, KinematicState, Point, Pose, Pose3D},
};
use nalgebra::{Matrix6, RowVector6, Vector6};
use rand::{
    distributions::{uniform::Uniform, Distribution},
    seq::IteratorRandom,
    thread_rng,
};
use rand_distr::Normal;
use std::f64::consts::PI;
use std::net::{IpAddr, SocketAddr, ToSocketAddrs, UdpSocket};
use std::ops::Range;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime};
use stdvis_core::types::{CameraConfig, VisionTarget};

const VISION_PORT: u16 = 4826;
const VISION_IP: &str = "nano4904-3";

const VISION_LOCAL_PORT: u16 = 3925;

const ROBORIO_PORT: u16 = 7654;
const ROBORIO_IP: &str = "zach"; // IP: 10.49.4.2
const ROBORIO_LOCAL_PORT: u16 = 1234;

const DRIVER_STATION_PORT: u16 = 6857;
const DRIVER_STATION_IP: &str = "driver";
const DRIVER_STATION_LOCAL_PORT: u16 = 1846;

const LIDAR_PORT: &'static str = "/dev/ttyUSB0";
const LIDAR_DIST_RANGE: Option<Range<f64>> = Some(0.0..8000.);
const LIDAR_ANGLE_RANGE: &[Range<f64>] = &[40. / 180. * PI..2. * PI];

const INIT_X_ERROR: f64 = 0.01;
const INIT_Y_ERROR: f64 = 0.01;
const INIT_ANGLE_ERROR: f64 = 0.01;

// Noise in m/s^2 - multiply by the time to get the uncertainty in change
// of velocity in the amount of time.
const CONTROL_ANGLE_NOISE: f64 = 0.01;
const CONTROL_X_NOISE: f64 = 0.01;
const CONTROL_Y_NOISE: f64 = 0.01;

const VELOCITY_X_SENSOR_NOISE: f64 = 3.;
const VELOCITY_Y_SENSOR_NOISE: f64 = 3.;
const ROTATIONAL_VELOCITY_SENSOR_NOISE: f64 = 0.035;
const FOV: Point = Point {
    x: 1.2291998872260792,
    y: 0.755844884927083,
};
const CAMERA_DIST_MAX: f64 = 9.;

// Make sensors and sensor sinks for lidar, vision, odometry data, localization hypothesis
fn main() {
    let mut rng = thread_rng();
    let now = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_secs()
        .to_string();

    // Cartograph the map
    // THE MAP IS IN MILLIMETERS
    // let map = Arc::new(Map2D::new(vec![Object2D::Rectangle(
    //     (0., 0.).into(),
    //     (7000., 2000.).into(),
    // )]));

    // Initialize sensors
    // roborio sensor returns a position each time step and IMU acceleration data.
    // This blocks for when the game starts, since the initial send is blocking. Once
    // the roborio sends the first packet (when it's enabled or whenever) localization
    // will start running (note that this includes logging a significant amount of data).
    let mut roborio_sensor = UDPSensor::<(Pose, Pose, f64)>::new(
        ROBORIO_LOCAL_PORT,
        SocketAddr::new(
            ROBORIO_IP
                .parse::<IpAddr>()
                .expect("RoboRIO hostname invalid. "),
            ROBORIO_PORT,
        ),
    )
    .expect("RoboRIO sensor failed to initialize.");

    let mut roborio_logger = LogSensorSink::new_from_file(&format!("logs/roborio_{}.txt", now));
    roborio_logger.push((roborio_sensor.sense(), roborio_sensor.timestamp));
    roborio_logger.update_sink();

    // The position sensor is a sensor for the first element received
    // from the roborio, the position in each time step
    let position_sensor = DummySensor::new(roborio_sensor.latest_data.0);

    // let init_state = position_sensor.sense();
    // let init_state_range = (
    //     Normal::new(init_state.angle, INIT_ANGLE_ERROR).unwrap(),
    //     (
    //         Normal::new(init_state.position.x, INIT_X_ERROR).unwrap(),
    //         Normal::new(init_state.position.y, INIT_Y_ERROR).unwrap(),
    //     ),
    // );

    // The motion sensor takes the position sensor data over time and
    // returns the differences, allowing both MCL and
    let mut motion_sensor = DeltaSensor::<DummySensor<Pose>, Pose>::new(position_sensor);

    let odometry_time_sensor = DummySensor::new(roborio_sensor.timestamp);
    let mut odometry_dt_sensor = DeltaSensor::<DummySensor<f64>, f64>::new(odometry_time_sensor);

    let mut imu_sensor = DummySensor::new(roborio_sensor.latest_data.1);
    let mut turret_sensor = DummySensor::new(roborio_sensor.latest_data.2);

    let mut vision_sensor = UDPSensor::<Vec<VisionTarget>>::new(
        VISION_LOCAL_PORT,
        SocketAddr::new(
            VISION_IP
                .parse::<IpAddr>()
                .expect("Nano hostname invalid. "),
            VISION_PORT,
        ),
    )
    .expect("Vision sensor failed to initialize.");
    let center_sensor: DummyLimitedSensor<Vec<Point>, (Point, f64)> = DummyLimitedSensor::new(
        vision_sensor
            .sense()
            .iter()
            .map(|vision_target| Point {
                x: (vision_target.theta + turret_sensor.sense()).sin() * vision_target.dist,
                y: (vision_target.theta + turret_sensor.sense()).cos() * vision_target.dist,
            })
            .collect(),
        Some((FOV, CAMERA_DIST_MAX)),
    );

    let mut vision_logger = LogSensorSink::new_from_file(&format!("logs/vision_{}.txt", now));

    let mut lidar_sensor = RplidarSensor::with_range(
        LIDAR_PORT,
        Pose::default(),
        LIDAR_DIST_RANGE,
        None,
        LIDAR_ANGLE_RANGE.into(),
    );
    let mut lidar_logger = LogSensorSink::new_from_file(&format!("logs/lidar_{}.txt", now));

    // Initialize mcl
    // let mut mcl = {
    //     let max_particle_count = 40_000;
    //     let weight_sum_threshold = 300.;
    //     let death_condition = DeathCondition {
    //         particle_count_threshold: max_particle_count / 2,
    //         particle_concentration_threshold: 300.,
    //     };

    //     PoseMCL::from_distributions(
    //         init_state_range,
    //         max_particle_count,
    //         weight_sum_threshold,
    //         death_condition,
    //         map.clone(),
    //         exp_weight(1.05),
    //         |pose: &Pose, lidar: &RplidarSensor, map: &Arc<Map2D>| 0.,
    //         object_3d_detection_error(1.3, 1.3, 1.3),
    //         uniform_resampler(0.01, 7.),
    //     )
    // };

    // let mut filter = LocalizationFilter::new(
    //     Matrix6::from_diagonal(&Vector6::from_iterator(vec![
    //         INIT_ANGLE_ERROR.powi(2),
    //         INIT_X_ERROR.powi(2),
    //         INIT_Y_ERROR.powi(2),
    //         0.,
    //         0.,
    //         0.,
    //     ])),
    //     KinematicState::from_pose(init_state).into(),
    //     Config::default(),
    // );

    // Start event loop
    loop {
        // Update sensors
        let old_lidar_data: &[Point] = &lidar_sensor.sense()[..10];
        lidar_sensor.update();
        if old_lidar_data != &lidar_sensor.sense()[..10] {
            lidar_logger.push(lidar_sensor.sense());
            lidar_logger.update_sink();
        }

        roborio_sensor.update();
        roborio_logger.push((roborio_sensor.sense(), roborio_sensor.timestamp));
        roborio_logger.update_sink();

        // odometry_dt_sensor.push(roborio_sensor.timestamp);
        // motion_sensor.push(roborio_sensor.sense().0);
        // imu_sensor.push(roborio_sensor.sense().1);
        // turret_sensor.push(roborio_sensor.sense().2);

        vision_sensor.update();
        vision_logger.push((vision_sensor.sense(), vision_sensor.timestamp));
        vision_logger.update_sink();
        // center_sensor.push(
        //     vision_sensor
        //         .sense()
        //         .iter()
        //         .map(|vision_target| Point {
        //             x: (vision_target.theta + turret_sensor.sense()).cos() * vision_target.dist,
        //             y: (vision_target.theta + turret_sensor.sense()).sin() * vision_target.dist,
        //         })
        //         .collect(),
        // );

        // let delta_t = odometry_dt_sensor.sense();

        // Update mcl
        // mcl.control_update(&motion_sensor);
        // mcl.observation_update(&lidar_sensor, &center_sensor);

        // // Get MCL uncertainty to feed into the Kalman Filter
        // let mcl_uncertainty = variance_poses(
        //     &mcl.belief
        //         .clone()
        //         .into_iter()
        //         .choose_multiple(&mut rng, 500),
        // );

        // Update Kalman Filter
        // let q = odometry_dt_sensor.sense().powi(2)
        //     * Matrix6::from_diagonal(&Vector6::from_iterator(vec![
        //         0.00000,
        //         0.00000,
        //         0.00000,
        //         CONTROL_ANGLE_NOISE.powi(2),
        //         CONTROL_X_NOISE.powi(2),
        //         CONTROL_Y_NOISE.powi(2),
        //     ]));
        // filter.prediction_update(odometry_dt_sensor.sense(), imu_sensor.sense().into(), q);

        // let r = Matrix6::from_diagonal(&Vector6::from_vec(vec![
        //     mcl_uncertainty.angle,
        //     mcl_uncertainty.position.x,
        //     mcl_uncertainty.position.y,
        //     ROTATIONAL_VELOCITY_SENSOR_NOISE.powi(2),
        //     VELOCITY_X_SENSOR_NOISE.powi(2),
        //     VELOCITY_Y_SENSOR_NOISE.powi(2),
        // ]));

        // filter.measurement_update(
        //     RowVector6::from_vec(vec![
        //         0.,
        //         0.,
        //         0.,
        //         motion_sensor.sense().angle / delta_t,
        //         motion_sensor.sense().position.x / delta_t,
        //         motion_sensor.sense().position.y / delta_t,
        //     ]),
        //     r,
        // );
        // Render frame
    }
}
