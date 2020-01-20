use piston_window::*;
use utility::Point;



fn point_cloud<G, I>(
    points: I,
    color: [f32; 4],
    point_radius: f64,
    scale: f64,
    offset: Point,
    transform: [[f64; 3]; 2],
    g: &mut G,
) where
    I: IntoIterator<Item = Point>,
    G: Graphics,
{
    let radius = Point { x: radius, y: radius };
    for point in points {
        let center = offset + point * scale;
        ellipse_from_to(color, (center - radius).into(), (center + radius).into(), transform, g);
    }
}
