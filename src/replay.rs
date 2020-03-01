use crate::{
    map::Map2D,
    sensors::Sensor,
    utility::{Point, Pose},
};
use piston_window::*;
use std::{f64::consts::*, io::Write, sync::Arc};

pub const BLACK: [f32; 4] = [0., 0., 0., 1.];
pub const RED: [f32; 4] = [1., 0., 0., 1.];
pub const GREEN: [f32; 4] = [0., 1., 0., 1.];
pub const BLUE: [f32; 4] = [0., 0., 1., 1.];

/// HAS SIDE EFFECTS
pub struct LoggingSensor<S, O, T, M>
where
    S: Sensor<Output = T>,
    O: Write,
    M: Fn(T) -> String,
    T: Clone,
{
    internal_sensor: S,
    latest_data: T,
    output: O,
    to_string: M,
}

impl<S, O, T, M> LoggingSensor<S, O, T, M>
where
    S: Sensor<Output = T>,
    O: Write,
    M: Fn(T) -> String,
    T: Clone,
{
    pub fn new(internal_sensor: S, output: O, to_string: M) -> Self {
        let latest_data = internal_sensor.sense();
        Self {
            internal_sensor,
            latest_data,
            output,
            to_string,
        }
    }
}

impl<'a, S, O, T, M> Sensor for LoggingSensor<S, O, T, M>
where
    S: Sensor<Output = T>,
    O: Write,
    M: Fn(T) -> String,
    T: Clone,
{
    type Output = T;

    fn update(&mut self) {
        self.latest_data = self.internal_sensor.sense();
        self.output
            .write_all((self.to_string)(self.latest_data.clone()).as_bytes())
            .unwrap();
    }

    fn sense(&self) -> Self::Output {
        self.latest_data.clone()
    }
}

pub fn draw_map<G>(
    map: Arc<Map2D>,
    color: [f32; 4],
    point_radius: f64,
    line_radius: f64,
    scale: f64,
    offset: Point,
    transform: [[f64; 3]; 2],
    g: &mut G,
) where
    G: Graphics,
{
    let point_radius: Point = (point_radius, point_radius).into();
    for line in map.lines.clone() {
        line_from_to(
            color,
            line_radius,
            map.vertices[line.0] * scale + offset,
            map.vertices[line.1] * scale + offset,
            transform,
            g,
        );
    }
    for &point in &map.points {
        let v: Point = map.vertices[point] * scale + offset;
        ellipse_from_to(color, v + point_radius, v - point_radius, transform, g);
    }
}

pub fn point_cloud<G>(
    points: &[Point],
    color: [f32; 4],
    point_radius: f64,
    scale: f64,
    offset: Point,
    transform: [[f64; 3]; 2],
    g: &mut G,
) where
    G: Graphics,
{
    let point_radius: Point = (point_radius, point_radius).into();
    for point in points {
        let center = offset + *point * scale;
        ellipse_from_to(
            color,
            center - point_radius,
            center + point_radius,
            transform,
            g,
        );
    }
}

pub fn isoceles_triangle<G: Graphics>(
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
