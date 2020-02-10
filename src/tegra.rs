use global_robot_localization::{
    robot::{
        ai::{
            localization::{DeathCondition, PoseMCL},
            presets::{exp_weight, lidar_error, uniform_resampler},
        },
        map::{Map2D, Object2D},
        sensors::{
            network::{self, PoseNTSensor},
            rplidar::RplidarSensor,
            DeltaSensor, Sensor,
        },
    },
    utility::Pose,
};
use nt::{EntryData, EntryValue, NetworkTables};
use std::sync::Arc;

const LIDAR_PORT: &'static str = "/dev/ttyUSB0";

fn main() {
    // Cartograph the map
    let map = Arc::new(Map2D::new(
        7000.,
        2000.,
        vec![Object2D::Rectangle((
            (0., 0.).into(),
            (7000., 2000.).into(),
        ))],
    ));
    // Initialize networktables connection for output
    let inst = NetworkTables::connect(network::DEFAULT_ROBORIO_IP, "😎leo😎")
        .expect("Failed to initialize networktables connection");
    let mut angle_entry = {
        let id = inst.create_entry(EntryData::new(
            "localization/angle".to_owned(),
            0,
            EntryValue::Double(0.),
        ));
        inst.get_entry(id)
    };
    let mut x_entry = {
        let id = inst.create_entry(EntryData::new(
            "localization/x".to_owned(),
            0,
            EntryValue::Double(0.),
        ));
        inst.get_entry(id)
    };
    let mut y_entry = {
        let id = inst.create_entry(EntryData::new(
            "localization/y".to_owned(),
            0,
            EntryValue::Double(0.),
        ));
        inst.get_entry(id)
    };
    // Initialize sensors
    let mut lidar = RplidarSensor::new(LIDAR_PORT, Pose::default(), None);
    let mut nt_navx = DeltaSensor::new(
        PoseNTSensor::new(
            Pose::default(),
            network::DEFAULT_ROBORIO_IP,
            "navx/yaw".to_owned(),
            "navx/displacementX".to_owned(),
            "navx/displacementY".to_owned(),
        )
        .expect("Failed to initialized networktables sensor"),
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
        lidar.update();
        nt_navx.update();

        // Update mcl
        mcl.control_update(&nt_navx);
        mcl.observation_update(&lidar);

        // Push prediction to the network
        let prediction = mcl.get_prediction();
        angle_entry.set_value(EntryValue::Double(prediction.angle));
        x_entry.set_value(EntryValue::Double(prediction.position.x));
        y_entry.set_value(EntryValue::Double(prediction.position.y));
    }
}
