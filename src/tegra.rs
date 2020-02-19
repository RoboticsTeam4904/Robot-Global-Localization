use global_robot_localization::{
    ai::{
        localization::{DeathCondition, PoseMCL},
        presets::{exp_weight, lidar_error, uniform_resampler},
    },
    map::{Map2D, Object2D},
    networktables,
    replay::*,
    sensors::{network::MultiNTSensor, rplidar::RplidarSensor, *},
    utility::{Point, Pose},
};
use piston_window::*;
use std::sync::Arc;

const LIDAR_PORT: &'static str = "/dev/ttyUSB0";
const WINDOW_SIZE: [f64; 2] = [1000., 1000.];
const RENDER_MAP: bool = true;
const RENDER_SCAN: bool = true;
const RENDER_PREDICTION: bool = true;
const MAP_SCALE: f64 = 0.5;
const MAP_OFFSET: Point = Point { x: 25., y: 25. };

#[tokio::main]
async fn main() -> Result<(), ()> {
    // Cartograph the map
    // THE MAP IS IN MILLIMETERS
    let map = Arc::new(Map2D::new(vec![Object2D::Rectangle((
        (0., 0.).into(),
        (7000., 2000.).into(),
    ))]));
    // Initialize sensors
    let mut lidar = RplidarSensor::with_range(LIDAR_PORT, Pose::default(), Some(0.0..8000.), None);
    let mut nt_imu = DeltaSensor::new(
        MultiNTSensor::new(
            Pose::default(),
            networktables::DEFAULT_ROBORIO_IP,
            vec![
                "navx/yaw".to_owned(),
                "encoders/netDisplacementAngle".to_owned(),
                "encoders/netDisplacement".to_owned(),
            ]
        )
        .await
        .expect("Failed to initialized networktables sensor")
        .map(|pose: Vec<f64>| Pose {
            angle: pose[0].to_radians(),
            position: Point::polar(
                pose[1],
                pose[2],
            ) * 1000.,
        }),
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
    // Initialize window
    let mut window: PistonWindow = WindowSettings::new("😎", WINDOW_SIZE)
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
        // TODO

        // Render frame
        window.draw_2d(&e, |c, g, _device| {
            clear([1.0; 4], g);
            if RENDER_MAP {
                draw_map(
                    map.clone(),
                    BLACK,
                    1.,
                    1.,
                    MAP_SCALE,
                    MAP_OFFSET,
                    c.transform,
                    g,
                )
            }
            if RENDER_SCAN {
                point_cloud(
                    &lidar.sense(),
                    RED,
                    0.5,
                    MAP_SCALE,
                    MAP_OFFSET + map.size * MAP_SCALE,
                    c.transform,
                    g,
                );
            }
            if RENDER_PREDICTION {
                isoceles_triangle(BLUE, MAP_OFFSET, MAP_SCALE, 5., prediction, c.transform, g)
            }
        });
    }
    Ok(())
}
