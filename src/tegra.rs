use global_robot_localization::{
    ai::{
        localization::{AdaptivePoseMCL, PoseMCL},
        presets::{exp_weight, lidar_error, uniform_resampler},
    },
    config::*,
    map::{Map2D, Object2D},
    networktables,
    replay::*,
    sensors::{network::MultiNTSensor, rplidar::RplidarSensor, *},
    utility::{Point, Pose},
};
use nt::{EntryValue, NetworkTables};
use piston_window::*;
use std::{env, fs::File, io::Read, sync::Arc};

#[tokio::main]
async fn main() -> Result<(), ()> {
    // Grab config from file
    let mut args = env::args();
    let config_path = args.nth(1).expect("Please provide a path to a config file");
    let mut config_file =
        File::open(config_path).expect("Please provide a valid path to a config file");
    let mut raw_config = String::new();
    config_file.read_to_string(&mut raw_config).unwrap();
    let config: TegraConfig =
        ron::de::from_str(&raw_config).expect("Could not parse provided config file");
    // Cartograph the map
    // THE MAP IS IN MILLIMETERS
    let map = Arc::new(Map2D::new(vec![Object2D::Rectangle((
        (0., 0.).into(),
        (7000., 2000.).into(),
    ))]));
    // Initialize networktable entries for output
    let inst = NetworkTables::connect(networktables::DEFAULT_ROBORIO_IP, "nano")
        .await
        .expect("Failed to start networktables");
    let mut x_entry =
        networktables::get_entry(&inst, "localization/x", EntryValue::Double(0.)).await;
    let mut y_entry =
        networktables::get_entry(&inst, "localization/y", EntryValue::Double(0.)).await;
    let mut angle_entry =
        networktables::get_entry(&inst, "localization/angle", EntryValue::Double(0.)).await;
    // Initialize sensors
    let mut lidar =
        RplidarSensor::with_range("/dev/ttyUSB0", Pose::default(), Some(0.0..8000.), None);
    let mut nt_imu = DeltaSensor::new(
        MultiNTSensor::new(
            Pose::default(),
            networktables::DEFAULT_ROBORIO_IP,
            vec![
                "navx/yaw".to_owned(),
                "encoders/netDisplacementAngle".to_owned(),
                "encoders/netDisplacement".to_owned(),
            ],
        )
        .await
        .expect("Failed to start networktables sensor")
        .map(|pose: Vec<f64>| Pose {
            angle: pose[0],
            position: Point::polar(pose[1], pose[2]) * 1000.,
        }),
    );
    // Initialize mcl
    let mut mcl = match config.mcl.variant {
        MCLVariant::Adaptive {
            weight_sum_threshold,
        } => AdaptivePoseMCL::new(
            config.mcl.max_particle_count,
            weight_sum_threshold,
            config.mcl.death_condition.clone(),
            map.clone(),
            exp_weight(1.05),
            lidar_error(1.3, 0.2),
            uniform_resampler(0.01, 7.),
        ),
        _ => unimplemented!("Getting types to work is annoying"),
    };
    // Initialize window
    let mut window: PistonWindow = WindowSettings::new("😎", config.render.window_size)
        .exit_on_esc(true)
        .build()
        .unwrap();
    // Start event loop
    while let Some(e) = window.next() {
        // Update sensors
        lidar.update();
        nt_imu.update();

        // Update mcl
        mcl.control_update(&nt_imu);
        mcl.observation_update(&lidar);

        // Push prediction to the network
        let prediction = mcl.get_prediction();
        x_entry.set_value(EntryValue::Double(prediction.position.x));
        y_entry.set_value(EntryValue::Double(prediction.position.y));
        angle_entry.set_value(EntryValue::Double(prediction.angle));

        // Render frame
        window.draw_2d(&e, |c, g, _device| {
            clear([1.0; 4], g);
            if config.render.render_map {
                draw_map(
                    map.clone(),
                    BLACK,
                    1.,
                    1.,
                    config.render.map_scale,
                    config.render.map_offset,
                    c.transform,
                    g,
                )
            }
            if config.render.render_scan {
                point_cloud(
                    &lidar.sense(),
                    RED,
                    0.5,
                    config.render.map_scale,
                    config.render.map_offset + map.size * config.render.map_scale,
                    c.transform,
                    g,
                );
            }
            if config.render.render_prediction {
                isoceles_triangle(
                    BLUE,
                    config.render.map_offset,
                    config.render.map_scale,
                    5.,
                    prediction,
                    c.transform,
                    g,
                )
            }
        });
    }
    Ok(())
}
