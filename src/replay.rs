use std::f64::consts::PI;
use piston_window::*;
use crate::utility::Point;

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
    let point_radius: Point = (point_radius,point_radius).into();
    for point in points {
        let center = offset + *point * scale;
        ellipse_from_to(color, center - point_radius, center + point_radius, transform, g);
    }
}
