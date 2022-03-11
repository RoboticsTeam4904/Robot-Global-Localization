use global_robot_localization::{
    ai::{
        kalman_filter::{Config, KalmanFilter, LocalizationFilter},
        localization::{DeathCondition, PoseMCL},
        presets::{exp_weight, lidar_error, object_3d_detection_error, uniform_resampler},
    },
    map::{Map2D, Object2D},
    replay::render::*,
    sensors::{
        dummy::DummySensor,
        network::{networktables, MultiNTSensor, PoseNTSensor},
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

const VISION_PORT: u16 = 4826;
const VISION_IP: &str = "nano4904-3@nano4904-3";

const VISION_LOCAL_PORT: u16 = 3925;

const ROBORIO_PORT: u16 = 7654;
const ROBORIO_IP: &str = "zach@zach";
const ROBORIO_LOCAL_PORT: u16 = 1234;

const DRIVER_STATION_PORT: u16 = 6857;
const DRIVER_STATION_IP: &str = "driver@driver";
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

// Make sensors and sensor sinks for lidar, vision, odometry data, localization hypothesis
fn main() {
    let mut rng = thread_rng();
    // Cartograph the map
    // THE MAP IS IN MILLIMETERS
    let map = Arc::new(Map2D::new(vec![Object2D::Rectangle(
        (0., 0.).into(),
        (7000., 2000.).into(),
    )]));

    // Initialize sensors
    // roborio sensor returns a position each time step and IMU acceleration data.
    let mut roborio_sensor = UDPSensor::<(Pose, Pose)>::new(
        ROBORIO_LOCAL_PORT,
        SocketAddr::new(
            ROBORIO_IP
                .parse::<IpAddr>()
                .expect("RoboRIO hostname invalid. "),
            ROBORIO_PORT,
        ),
    )
    .expect("RoboRIO sensor failed to initialize.");

    // The position sensor is a sensor for the first element received
    // from the roborio, the position in each time step
    let position_sensor = DummySensor::new(roborio_sensor.latest_data.0);
    let init_state = position_sensor.sense();
    let init_state_range = (
        Normal::new(init_state.angle, INIT_ANGLE_ERROR).unwrap(),
        (
            Normal::new(init_state.position.x, INIT_X_ERROR).unwrap(),
            Normal::new(init_state.position.y, INIT_Y_ERROR).unwrap(),
        ),
    );

    // The motion sensor takes the position sensor data over time and
    // returns the differences, allowing both MCL and
    let mut motion_sensor = DeltaSensor::<DummySensor<Pose>, Pose>::new(position_sensor);

    let odometry_time_sensor = DummySensor::new(roborio_sensor.timestamp);
    let mut odometry_dt_sensor = DeltaSensor::<DummySensor<f64>, f64>::new(odometry_time_sensor);

    let mut imu_sensor = DummySensor::new(roborio_sensor.latest_data.1);

    let mut vision_sensor = UDPSensor::<Pose3D>::new(
        VISION_LOCAL_PORT,
        SocketAddr::new(
            VISION_IP
                .parse::<IpAddr>()
                .expect("Nano hostname invalid. "),
            VISION_PORT,
        ),
    )
    .expect("Vision sensor failed to initialize.");

    let mut lidar_sensor = RplidarSensor::with_range(
        LIDAR_PORT,
        Pose::default(),
        LIDAR_DIST_RANGE,
        None,
        LIDAR_ANGLE_RANGE.into(),
    );

    // Initialize mcl
    let mut mcl = {
        let max_particle_count = 40_000;
        let weight_sum_threshold = 300.;
        let death_condition = DeathCondition {
            particle_count_threshold: max_particle_count / 2,
            particle_concentration_threshold: 300.,
        };

        PoseMCL::from_distributions(
            init_state_range,
            max_particle_count,
            weight_sum_threshold,
            death_condition,
            map.clone(),
            exp_weight(1.05),
            lidar_error(1.3, 0.2),
            object_3d_detection_error(1.3, 1.3, 1.3),
            uniform_resampler(0.01, 7.),
        )
    };

    let mut filter = LocalizationFilter::new(
        Matrix6::from_diagonal(&Vector6::from_iterator(vec![
            INIT_ANGLE_ERROR.powi(2),
            INIT_X_ERROR.powi(2),
            INIT_Y_ERROR.powi(2),
            0.,
            0.,
            0.,
        ])),
        KinematicState::from_pose(init_state).into(),
        Config::default(),
    );

    // Start event loop
    loop {
        // Update sensors
        lidar_sensor.update();
        vision_sensor.update();
        roborio_sensor.update();
        motion_sensor.push(roborio_sensor.latest_data.0);
        odometry_dt_sensor.push(roborio_sensor.timestamp);
        imu_sensor.push(roborio_sensor.latest_data.1);
        let delta_t = odometry_dt_sensor.sense();
        // nt_imu.update();

        // Update mcl
        mcl.control_update(&motion_sensor);
        mcl.observation_update(&lidar_sensor, &vision_sensor);

        // Get MCL uncertainty to feed into the Kalman Filter
        let mcl_uncertainty = variance_poses(
            &mcl.belief
                .clone()
                .into_iter()
                .choose_multiple(&mut rng, 500),
        );
        // // Push prediction to the network
        // let prediction = mcl.get_prediction();
        // localization_output.push(prediction);

        // Update Kalman Filter
        let q = odometry_dt_sensor.sense().powi(2)
            * Matrix6::from_diagonal(&Vector6::from_iterator(vec![
                0.00000,
                0.00000,
                0.00000,
                CONTROL_ANGLE_NOISE.powi(2),
                CONTROL_X_NOISE.powi(2),
                CONTROL_Y_NOISE.powi(2),
            ]));
        filter.prediction_update(odometry_dt_sensor.sense(), imu_sensor.sense().into(), q);

        let r = Matrix6::from_diagonal(&Vector6::from_vec(vec![
            mcl_uncertainty.angle,
            mcl_uncertainty.position.x,
            mcl_uncertainty.position.y,
            delta_t.powi(2) * ROTATIONAL_VELOCITY_SENSOR_NOISE.powi(2),
            delta_t.powi(2) * VELOCITY_X_SENSOR_NOISE.powi(2),
            delta_t.powi(2) * VELOCITY_Y_SENSOR_NOISE.powi(2),
        ]));

        filter.measurement_update(
            RowVector6::from_vec(vec![
                0.,
                0.,
                0.,
                motion_sensor.sense().angle / delta_t,
                motion_sensor.sense().position.x / delta_t,
                motion_sensor.sense().position.y / delta_t,
            ]),
            r,
        );
        // Render frame
    }
}
