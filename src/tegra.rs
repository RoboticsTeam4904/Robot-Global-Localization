use global_robot_localization::{
    ai::{
        localization::{DeathCondition, PoseMCL},
        presets::{exp_weight, lidar_error, uniform_resampler},
    },
    map::{Map2D, Object2D},
    networktables,
    replay::*,
    sensors::{
        network::PoseNTSensor, rplidar::RplidarSensor, DeltaSensor, Sensor, SensorSink,
        WrappableSensor,
    },
    utility::Pose,
};
use nt::NetworkTables;
use std::{
    fs::File,
    sync::{Arc, Mutex},
    time::{SystemTime, UNIX_EPOCH},
};

const LIDAR_PORT: &'static str = "/dev/ttyUSB0";

#[tokio::main]
async fn main() -> Result<(), ()> {
    // Cartograph the map
    // THE MAP IS IN MILLIMETERS
    let map = Arc::new(Map2D::new(vec![
        // TODO
    ]));
    // Initialize networktable entries for output
    let inst = Arc::new(Mutex::new(
        NetworkTables::connect(networktables::DEFAULT_ROBORIO_IP, "nano")
            .await
            .expect("Failed to start networktables"),
    ));
    let mut localization_output = PoseNTSensor::from_inst(
        Pose::default(),
        inst.clone(),
        "localization/x".to_owned(),
        "localization/y".to_owned(),
        "localization/angle".to_owned(),
    )
    .await;
    // Initialize sensors
    let mut lidar = RplidarSensor::with_range(
        LIDAR_PORT,
        Pose::default().with_position((329.8792, 330.1492).into()),
        Some(0.0..8000.),
        None,
    );
    let imu_log_file = File::create(format!(
        "imu_log_{}.txt",
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis()
    ))
    .unwrap();
    // Being given to us in meters, meters, and radians, respectively
    let mut nt_imu = DeltaSensor::new(LoggingSensor::new(
        PoseNTSensor::from_inst(
            Pose::default(),
            inst,
            "odometry/x".to_owned(),
            "odometry/y".to_owned(),
            "odometry/angle".to_owned(),
        )
        .await
        .map(|pose: Pose| pose.with_position(pose.position * 1000.)),
        imu_log_file,
        |pose: Pose| format!("{} {} {}", pose.position.x, pose.position.y, pose.angle),
    ));
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
        // Update sensors & log
        lidar.update();
        nt_imu.update();

        // Update mcl
        mcl.control_update(&nt_imu);
        mcl.observation_update(&lidar);

        // Push prediction to the network
        let prediction = mcl.get_prediction();
        localization_output.push(prediction);
    }
}
