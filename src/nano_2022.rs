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
use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{IpAddr, SocketAddr, ToSocketAddrs, UdpSocket};
use std::ops::Range;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, SystemTime};
use stdvis_core::types::{CameraConfig, VisionTarget};

const VISION_ADDR: &str = "10.49.4.9:7698";
const VISION_LOCAL_PORT: u16 = 2746;

const ROBORIO_LOCAL_PORT: u16 = 4321;
const ROBORIO_ADDR: &str = "10.49.4.2:3375";

const CAMERA_TURRET_RADIUS: f64 = -0.1696;

const ITER_DELAY: f64 = 0.01;

const LIDAR_PORT: &'static str = "/dev/ttyUSB0";
const LIDAR_DIST_RANGE: Option<Range<f64>> = Some(0.0..8000.);
const LIDAR_ANGLE_RANGE: &[Range<f64>] = &[(0.)..2. * PI];
// Make sensors and sensor sinks for lidar, vision, odometry data, localization hypothesis
fn main() {
    let mut rng = thread_rng();
    let now = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_secs()
        .to_string();
    let mut file_read = OpenOptions::new()
        .read(true)
        .open("logs/counter")
        .expect("counter file could not be opened in read mode.");

    let mut now = BufReader::new(&mut file_read)
        .lines()
        .next()
        .unwrap()
        .unwrap();
    let mut counter_num: u64 = now.parse::<u64>().unwrap();
    counter_num += 1;
    let mut file_write = OpenOptions::new()
        .write(true)
        .open("logs/counter")
        .expect("counter file could not be opened in write mode");
    file_write
        .write_all(counter_num.to_string().as_bytes())
        .unwrap_or_else(|_| println!("unable to write to counter file."));

    // Initialize sensors
    // roborio sensor returns a position each time step and IMU acceleration data.
    // This blocks for when the game starts, since the initial send is blocking. Once
    // the roborio sends the first packet (when it's enabled or whenever) localization
    // will start running (note that this includes logging a significant amount of data).

    let roborio_sink_socket =
        UdpSocket::bind(SocketAddr::from(([0, 0, 0, 0], ROBORIO_LOCAL_PORT))).unwrap();

    let roborio_sensor_socket = roborio_sink_socket.try_clone().unwrap();
    let sensor_now = now.clone();

    thread::spawn(move || {
        let mut roborio_sensor = UDPSensor::<(Pose, Pose, f64)>::new(roborio_sensor_socket)
            .expect("RoboRIO sensor failed to initialize.");
        let mut roborio_logger =
            LogSensorSink::new_from_file(&format!("logs/roborio_{}.txt", sensor_now));
        roborio_logger.push((roborio_sensor.sense(), roborio_sensor.timestamp));
        roborio_logger.update_sink();
        loop {
            roborio_sensor.update();
            roborio_logger.push((roborio_sensor.sense(), roborio_sensor.timestamp));
            roborio_logger.update_sink();
            thread::sleep(Duration::from_secs_f64(ITER_DELAY));
        }
    });

    let mut roborio_sink =
        UDPSensorSink::<(f64, f64)>::new(roborio_sink_socket, ROBORIO_ADDR.to_string())
            .expect("RoboRIO sensor sink failed to initialize.");

    // println!("Success: {:?}", roborio_sensor.sense());

    println!("Initializing vision sensor.");
    let mut vision_sensor = UDPSensor::<Vec<VisionTarget>>::new_with_port(VISION_LOCAL_PORT)
        .expect("Vision sensor failed to initialize.");
    println!("Vision sensor initialized.");

    let mut vision_logger = LogSensorSink::new_from_file(&format!("logs/vision_{}.txt", now));

    let mut lidar_sensor = RplidarSensor::with_range(
        LIDAR_PORT,
        Pose::default(),
        LIDAR_DIST_RANGE,
        None,
        LIDAR_ANGLE_RANGE.into(),
    );
    let mut lidar_logger = LogSensorSink::new_from_file(&format!("logs/lidar_{}.txt", now));

    // Start event loop
    loop {
        lidar_sensor.update();
        lidar_logger.push(lidar_sensor.sense());
        lidar_logger.update_sink();

        vision_sensor.update();
        vision_logger.push((vision_sensor.sense(), vision_sensor.timestamp));
        vision_logger.update_sink();

        for target in &vision_sensor.sense() {
            roborio_sink.push((target.dist - CAMERA_TURRET_RADIUS, target.theta));
            roborio_sink.update_sink();
        }

        if vision_sensor.sense().len() == 0 {
            roborio_sink.push((4., 0.));
            roborio_sink.update_sink();
        }

        // Render frame
        thread::sleep(Duration::from_secs_f64(ITER_DELAY));
    }
}
