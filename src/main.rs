#![allow(dead_code)] // TODO: This will be removed after organizational overhall and a lib.rs
mod robot;
mod utility;
mod replay;

use piston_window::*;
use rand::distributions::{Distribution, Normal};
use robot::ai::localization::PoseMCL;
use robot::map::{Map2D, Object2D};
use robot::sensors::{nt::PoseNTSensor, rplidar::RplidarSensor, LimitedSensor, Sensor};
use std::{
    cmp::Ordering,
    f64::consts::*,
    ops::{Bound, Range, RangeBounds},
    sync::Arc,
};
use utility::{Point, Pose};
use replay::point_cloud;

const NETWORKTABLES_IP: &'static str = "localhost:1735";
const MAP_SCALE: f64 = 2.;
const ROBOT_ACCEL: f64 = 3.;
const ROBOT_ANGLE_ACCEL: f64 = 0.1;

struct Robot<R: RangeBounds<f64> + Clone> {
    mcl: PoseMCL<RplidarSensor<R>>,
    motion_sensor: PoseNTSensor,
    lidar: RplidarSensor<R>,
}

impl<R: RangeBounds<f64> + Clone> Robot<R> {
    fn update(&mut self) {
        self.lidar.update();
        self.mcl.control_update(&self.motion_sensor);
        self.mcl.observation_update(&self.lidar);
    }
}

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
        ],
    ));

    // let motion_sensor = PoseNTSensor::new(
    //     Pose::default(),
    //     NETWORKTABLES_IP,
    //     "navx/x".to_string(),
    //     "navx/y".to_string(),
    //     "navx/yaw".to_string(),
    // )
    // .expect("Failed to initialized motion sensor.");
    let lidar = RplidarSensor::with_range("/dev/ttyUSB0", Pose::default(), Some(0.15..8.0), None);

    // let mcl = PoseMCL::<RplidarSensor<Range<f64>>>::new(
    //     20_000,
    //     map.clone(),
    //     Box::new(|error| 1.05f64.powf(-error)),
    //     Box::new(|sample, lidar, map| {
    //         let angle_increment = 0.01570796327; // 0.45 degrees in radians
    //         let mut observed_scan = lidar.sense();
    //         let mut expected_scan = vec![];
    //         let mut angle = 0.;
    //         while angle < 2. * PI {
    //             let scan_point = map.raycast(
    //                 (lidar.relative_pose() + *sample)
    //                     + Pose {
    //                         angle,
    //                         ..Pose::default()
    //                     },
    //             );
    //             angle += angle_increment; // do not use angle from this point on in the loop
    //             if let Some(s) = scan_point {
    //                 if let Some(sense_range) = lidar.range() {
    //                     match sense_range.end_bound() {
    //                         Bound::Included(&max) if s.mag() > max => continue,
    //                         Bound::Excluded(&max) if s.mag() >= max => continue,
    //                         _ => (),
    //                     }
    //                     match sense_range.start_bound() {
    //                         Bound::Included(&max) if s.mag() < max => continue,
    //                         Bound::Excluded(&max) if s.mag() <= max => continue,
    //                         _ => (),
    //                     }
    //                 }
    //                 expected_scan.push(s);
    //             };
    //         }
    //         observed_scan.sort_by(|a, b| {
    //             a.angle(Point::default())
    //                 .partial_cmp(&b.angle(Point::default()))
    //                 .unwrap_or(Ordering::Equal)
    //         });
    //         expected_scan.sort_by(|a, b| {
    //             a.angle(Point::default())
    //                 .partial_cmp(&b.angle(Point::default()))
    //                 .unwrap_or(Ordering::Equal)
    //         });
    //         let mut error = 0.;
    //         for (observed, expected) in observed_scan.iter().zip(expected_scan) {
                
    //         }
    //         error
    //     }),
    //     Pose {
    //         angle: FRAC_PI_8 / 4.,
    //         position: Point { x: 0.5, y: 0.5 },
    //     },
    //     1.,
    // );
    // let mut robot = Robot {
    //     lidar,
    //     mcl,
    //     motion_sensor,
    // };

    // let map_visual_margins: Point = (25., 25.).into();
    let mut window: PistonWindow = WindowSettings::new("😎", [1000, 1000])
        .exit_on_esc(true)
        .build()
        .unwrap();
    let mut tick: u32 = 0;

    while let Some(e) = window.next() {
        // Rendering
        window.draw_2d(&e, |c, g, _device| {
            clear([1.0; 4], g);
            point_cloud(lidar.sense(), [1., 0.,0., 1.], 5., 10., (250., 250.).into(), c.transform, g);
            // for line in map.lines.clone() {
            //     line_from_to(
            //         [0., 0., 0., 1.],
            //         1.,
            //         map.vertices[line.0] * MAP_SCALE + map_visual_margins,
            //         map.vertices[line.1] * MAP_SCALE + map_visual_margins,
            //         c.transform,
            //         g,
            //     );
            // }
            // for point in map.clone().points.clone() {
            //     let v: Point = map.vertices[point] * MAP_SCALE + map_visual_margins;
            //     let size: Point = (5., 5.).into();
            //     ellipse_from_to([0.7, 0.3, 0.3, 1.], v + size, v - size, c.transform, g);
            // }
            // println!("Particle count: {}", robot.mcl.belief.len());
            // for particle in &robot.mcl.belief {
            //     isoceles_triangle(
            //         [0., 0., 0., 1.],
            //         map_visual_margins,
            //         MAP_SCALE,
            //         0.5,
            //         *particle,
            //         c.transform,
            //         g,
            //     );
            // }
            // isoceles_triangle(
            //     [0., 0., 1., 1.],
            //     map_visual_margins,
            //     MAP_SCALE,
            //     1.,
            //     robot.mcl.get_prediction(),
            //     c.transform,
            //     g,
            // );
        });

        // Update the filter
        // robot.update();
        tick += 1;
    }
}

fn isoceles_triangle<G: Graphics>(
    color: [f32; 4],
    margin: Point,
    pose_scale: f64,
    triangle_scale: f64,
    pose: Pose,
    transform: math::Matrix2d,
    g: &mut G,
) {
    polygon(
        color,
        &[
            [
                pose.position.x * pose_scale + margin.x + triangle_scale * 15. * pose.angle.cos(),
                pose.position.y * pose_scale + margin.y + triangle_scale * 15. * pose.angle.sin(),
            ],
            [
                pose.position.x * pose_scale
                    + margin.x
                    + triangle_scale * 10. * (pose.angle + 2. * FRAC_PI_3).cos(),
                pose.position.y * pose_scale
                    + margin.y
                    + triangle_scale * 10. * (pose.angle + 2. * FRAC_PI_3).sin(),
            ],
            [
                pose.position.x * pose_scale
                    + margin.x
                    + triangle_scale * 10. * (pose.angle + 4. * FRAC_PI_3).cos(),
                pose.position.y * pose_scale
                    + margin.y
                    + triangle_scale * 10. * (pose.angle + 4. * FRAC_PI_3).sin(),
            ],
        ],
        transform,
        g,
    );
}
