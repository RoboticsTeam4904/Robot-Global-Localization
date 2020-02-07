use global_robot_localization::{
    replay::{draw_map, isoceles_triangle, point_cloud},
    robot::{
        ai::{
            localization::{DeathCondition, PoseMCL},
        },
        map::{Map2D, Object2D},
        sensors::{
            rplidar::RplidarSensor,
            LimitedSensor, Sensor,
        },
    },
    utility::{Point, Pose},
};
use piston_window::*;
use std::{
    f64::{
        consts::*,
        INFINITY,
    },
    sync::Arc,
};

const MAP_SCALE: f64 = 0.5;
const RENDER_MAP: bool = true;
const RENDER_SCAN: bool = true;
const RENDER_BELIEF: bool = true;
const RENDER_PREDICTION: bool = true;

fn main() {
    let map = Arc::new(Map2D::new(
        200.,
        200.,
        vec![
            Object2D::Point((0., 200.).into()),
            Object2D::Point((200., 200.).into()),
            Object2D::Point((200., 0.).into()),
            Object2D::Point((0., 0.).into()),
            Object2D::Line(((0., 0.).into(), (200., 0.).into())),
            Object2D::Line(((0., 0.).into(), (0., 200.).into())),
            Object2D::Line(((200., 0.).into(), (200., 200.).into())),
            Object2D::Line(((0., 200.).into(), (200., 200.).into())),
            Object2D::Line(((20., 60.).into(), (50., 100.).into())),
            Object2D::Line(((50., 100.).into(), (80., 50.).into())),
            Object2D::Line(((80., 50.).into(), (20., 60.).into())),
            Object2D::Line(((140., 100.).into(), (180., 120.).into())),
            Object2D::Line(((180., 120.).into(), (160., 90.).into())),
            Object2D::Line(((160., 90.).into(), (140., 100.).into())),
            Object2D::Line(((100., 40.).into(), (160., 80.).into())),
        ],
    ));
    let mut lidar = RplidarSensor::new(
        "/dev/ttyUSB0",
        Pose::default(),
        None,
    );
    let mut mcl = {
        let particle_count = 40_000;
        let weight_sum_threshold = 200.;
        let death_threshold = DeathCondition {
            particle_count_threshold: 4_000,
            particle_concentration_threshold: 300.,
        };
        PoseMCL::<RplidarSensor>::new(
            particle_count,
            weight_sum_threshold,
            death_threshold,
            map.clone(),
            Box::new(|e| 1.05f64.powf(-e)),
            Box::new(|&sample, lidar, map| {
                let sample = sample + lidar.relative_pose();
                let lidar_scan = lidar.sense();
                let len = lidar_scan.len() as f64;
                let lidar_range = lidar.range().unwrap_or(0.0..INFINITY);
                let mut error = 0.;
                for scan_point in lidar_scan {
                    error += match map.raycast(
                        sample
                            + Pose {
                                angle: scan_point.angle(),
                                ..Pose::default()
                            },
                    ) {
                        Some(predicted_point)
                            if lidar_range.contains(&predicted_point.dist(sample.position)) =>
                        {
                            (scan_point.mag() - predicted_point.dist(sample.position)).abs()
                        }
                        _ => 5.,
                    };
                }
                error / len
            }),
            Box::new(move |_| {
                Pose::random_from_range(Pose {
                    angle: FRAC_PI_8 / 100.,
                    position: (0.7, 0.7).into(),
                })
            }),
        )
    };

    let map_visual_margins: Point = (25., 25.).into();
    let mut window: PistonWindow = WindowSettings::new("😎", [1000, 1000])
        .exit_on_esc(true)
        .build()
        .unwrap();
    let mut tick: u32 = 0;
    while let Some(e) = window.next() {
        println!("T = {}", tick);
        println!("\tP = {}", mcl.belief.len());
        // Rendering
        window.draw_2d(&e, |c, g, _device| {
            clear([1.0; 4], g);
            if RENDER_MAP {
            draw_map(
                map.clone(),
                [0., 0., 0., 1.],
                0.5,
                1.,
                MAP_SCALE,
                map_visual_margins,
                c.transform,
                g,
            );
            }
            if RENDER_SCAN {
            point_cloud(
                &lidar.sense(),
                [0., 1., 0., 1.],
                1.,
                MAP_SCALE,
                map_visual_margins,
                c.transform,
                g,
            );
            }
            if RENDER_BELIEF {
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
            }
            if RENDER_PREDICTION {
            isoceles_triangle(
                [0., 0., 1., 1.],
                map_visual_margins,
                MAP_SCALE,
                0.5,
                mcl.get_prediction(),
                c.transform,
                g,
            );
            }
        });

        // update sensors
        lidar.update();

        // update localization
        mcl.observation_update(&lidar);

        tick += 1;
    }
}
