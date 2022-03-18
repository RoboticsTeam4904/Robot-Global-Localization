#![feature(let_chains)]
use global_robot_localization::{
    ai::{
        localization::{DeathCondition, PoseMCL},
        presets::{exp_weight, lidar_error, uniform_resampler},
    },
    map::{Map2D, Object2D},
    replay::render::*,
    sensors::{
        //network::{networktables, MultiNTSensor, PoseNTSensor},
        rplidar::RplidarSensor,
        DeltaSensor, Sensor, SensorSink, WrappableSensor,
    },
    utility::{Point, Pose},
};
//use nt::NetworkTables;
use piston_window::*;
use std::sync::{Arc, Mutex};
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
    let map = Arc::new(Map2D::new(vec![Object2D::Rectangle(
        (0., 0.).into(),
        (7000., 2000.).into(),
    )]));
    // Initialize sensors
    let mut lidar = match RplidarSensor::with_range(
            LIDAR_PORT,
            Pose::default(),
            Some(0.0..8000.),
            None
        ) {
        Ok(lidar) => lidar,
        Err(msg) => panic!("{}", msg)
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

        // Render frame
        window.draw_2d(&e, |c, g, _device| {
            clear([1.0; 4], g);
            if RENDER_MAP {
                draw_map(
                    map.clone(),
                    BLACK,
                    PURPLE,
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
            //if RENDER_PREDICTION {
            //    isoceles_triangle(BLUE, MAP_OFFSET, MAP_SCALE, 5., prediction, c.transform, g)
            //}
        });
    }
    Ok(())
}
