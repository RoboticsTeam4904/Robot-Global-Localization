use global_robot_localization::{
    ai::{
        localization::{DeathCondition, PoseMCL},
        presets::{exp_weight, lidar_error, uniform_resampler},
    },
    map::{Map2D, Object2D},
    replay::render::*,
    sensors::{
        network::{networktables, MultiNTSensor, PoseNTSensor},
        rplidar::RplidarSensor,
        udp::{UDPSensor, UDPSensorSink},
        DeltaSensor, Sensor, SensorSink, WrappableSensor,
    },
    utility::{Point, Pose},
};
use std::f64::consts::PI;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::ops::Range;
use std::sync::{Arc, Mutex};
const VISION_PORT: usize = 60480;
const LIDAR_PORT: &'static str = "/dev/ttyUSB0";
const LIDAR_DIST_RANGE: Option<Range<f64>> = Some(0.0..8000.);
const LIDAR_ANGLE_RANGE: Vec<Range<f64>> = vec![40. / 180. * PI..2. * PI];

fn main() {
    // Cartograph the map
    // THE MAP IS IN MILLIMETERS
    let map = Arc::new(Map2D::new(vec![Object2D::Rectangle(
        (0., 0.).into(),
        (7000., 2000.).into(),
    )]));

    // Initialize sensors
    let mut lidar = RplidarSensor::with_range(
        LIDAR_PORT,
        Pose::default(),
        LIDAR_DIST_RANGE,
        None,
        LIDAR_ANGLE_RANGE,
    );

    // Initialize mcl
    let mut mcl = {
        let max_particle_count = 40_000;
        let weight_sum_threshold = 300.;
        let death_condition = DeathCondition {
            particle_count_threshold: max_particle_count / 2,
            particle_concentration_threshold: 300.,
        };
        PoseMCL::new(
            max_particle_count,
            weight_sum_threshold,
            death_condition,
            map.clone(),
            exp_weight(1.05),
            lidar_error(1.3, 0.2),
            uniform_resampler(0.01, 7.),
        )
    };

    // Start event loop
    loop {
        // Update sensors
        // lidar.update();
        // nt_imu.update();

        // // Update mcl
        // mcl.control_update(&nt_imu);
        // mcl.observation_update(&lidar);

        // // Push prediction to the network
        // let prediction = mcl.get_prediction();
        // localization_output.push(prediction);

        // Render frame
    }
}
