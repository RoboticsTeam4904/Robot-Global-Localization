use crate::utility::{Point, Point3D, Pose, Pose3D};
use std::f64::consts::PI;

// TODO: this file is lazy

pub enum Object2D {
    Target(Pose3D),
    Line(Point, Point),
    Triangle(Point, Point, Point),
    Rectangle(Point, Point),
    RectangleFour(Point, Point, Point, Point),
}

/// A Simple 2D map of line segments
#[derive(Debug)]
pub struct Map2D {
    pub size: Point,
    pub targets: Vec<Pose3D>,
    pub vertices: Vec<Point>,
    pub lines: Vec<(usize, usize)>,
}

impl Map2D {
    pub fn new<U>(objects: U) -> Self
    where
        U: IntoIterator<Item = Object2D>,
    {
        let mut map = Self::with_size(Point::default(), objects);
        let mut max = Point::default();
        for vertex in &map.vertices {
            if vertex.x > max.x {
                max.x = vertex.x;
            }
            if vertex.y > max.y {
                max.y = vertex.y;
            }
        }
        map.size = max;
        map
    }

    pub fn with_size<U>(size: Point, objects: U) -> Self
    where
        U: IntoIterator<Item = Object2D>,
    {
        let mut vertices = Vec::new();
        let mut lines = Vec::new();
        let mut targets = Vec::new();
        let mut add_vert = |point: Point| -> usize {
            if let Some(idx) = vertices.iter().position(|&v| v == point) {
                idx
            } else {
                vertices.push(point);
                vertices.len() - 1
            }
        };

        for object in objects {
            match object {
                Object2D::Line(p1, p2) => lines.push((add_vert(p1), add_vert(p2))),
                Object2D::Triangle(c1, c2, c3) => {
                    let v1 = add_vert(c1);
                    let v2 = add_vert(c2);
                    let v3 = add_vert(c3);
                    lines.push((v1, v2));
                    lines.push((v2, v3));
                    lines.push((v3, v1));
                }
                Object2D::Rectangle(c1, c3) => {
                    let c2 = Point { x: c1.x, y: c3.y };
                    let c4 = Point { x: c3.x, y: c1.y };
                    let v1 = add_vert(c1);
                    let v2 = add_vert(c2);
                    let v3 = add_vert(c3);
                    let v4 = add_vert(c4);
                    lines.push((v1, v2));
                    lines.push((v2, v3));
                    lines.push((v3, v4));
                    lines.push((v4, v1));
                }
                Object2D::RectangleFour(c1, c2, c3, c4) => {
                    let v1 = add_vert(c1);
                    let v2 = add_vert(c2);
                    let v3 = add_vert(c3);
                    let v4 = add_vert(c4);
                    lines.push((v1, v2));
                    lines.push((v2, v3));
                    lines.push((v3, v4));
                    lines.push((v4, v1));
                }
                Object2D::Target(p) => targets.push(p),
            };
        }

        Self {
            size,
            vertices,
            lines,
            targets,
        }
    }

    /// Converts a file into a map. Returns `Ok` if file is formatted correctly.
    /// The file should be formated with the width as the first line of the file and height as the second line in the file
    /// followed by a linebreak delimitered list of map lines in the format x1 y1 x2 y2. All values are f64s. For example,
    /// ```
    /// width
    /// height
    /// x1 y1 x2 y2
    /// x1 y1 x2 y2
    /// ...
    /// ```
    pub fn from_file(path: &str) -> std::io::Result<Self> {
        use std::fs::File;
        use std::io::{Error, ErrorKind, Read};
        let mut file = File::open(path)?;
        let mut buf = String::new();
        let mut file_lines = {
            file.read_to_string(&mut buf)?;
            buf.lines()
        };
        // TODO: There is a better way to do error handling
        let width = match file_lines.next() {
            Some(w) => w
                .parse::<f64>()
                .map_err(|e| Error::new(ErrorKind::InvalidInput, e))?,
            None => {
                return Err(Error::new(
                    ErrorKind::InvalidInput,
                    "Incorrect map: no width",
                ));
            }
        };
        let height = match file_lines.next() {
            Some(h) => h
                .parse::<f64>()
                .map_err(|e| Error::new(ErrorKind::InvalidInput, e))?,
            None => {
                return Err(Error::new(
                    ErrorKind::InvalidInput,
                    "Incorrect map: no height",
                ));
            }
        };
        let mut lines = Vec::new();
        for line_text in file_lines {
            let point_vals = line_text.split(' ').collect::<Vec<&str>>();
            if point_vals.len() < 4 {
                return Err(Error::new(
                    ErrorKind::InvalidInput,
                    "Incorrect map: line with less than four values provided",
                ));
            }
            lines.push((
                Point {
                    x: point_vals[0]
                        .parse::<f64>()
                        .map_err(|e| Error::new(ErrorKind::InvalidInput, e))?,
                    y: point_vals[1]
                        .parse::<f64>()
                        .map_err(|e| Error::new(ErrorKind::InvalidInput, e))?,
                },
                Point {
                    x: point_vals[2]
                        .parse::<f64>()
                        .map_err(|e| Error::new(ErrorKind::InvalidInput, e))?,
                    y: point_vals[3]
                        .parse::<f64>()
                        .map_err(|e| Error::new(ErrorKind::InvalidInput, e))?,
                },
            ));
        }
        Ok(Self::with_size(
            (width, height).into(),
            lines.iter().map(|&l| Object2D::Line(l.0, l.1)),
        ))
    }

    pub fn get_vertex(&self, idx: usize) -> Point {
        self.vertices[idx]
    }

    pub fn raycast(&self, start: Pose) -> Option<Point> {
        let ray = Point {
            x: start.angle.cos(),
            y: start.angle.sin(),
        };
        let mut closest_intersection: Option<Point> = None;
        let mut closest_intersection_dist = 0.;
        for line in &self.lines {
            let v1 = start.position - self.get_vertex(line.0);
            let v2 = self.get_vertex(line.1) - self.get_vertex(line.0);
            let v3 = Point {
                x: -ray.y,
                y: ray.x,
            };
            let div = v2.dot(v3);
            if div == 0. {
                continue;
            }
            let t1 = v2.cross_mag(v1) / div;
            let t2 = v1.dot(v3) / div;
            if t1 >= 0. && t2 >= 0. && t2 <= 1. {
                let intersection = start.position + ray * t1;
                // let intersection_check = self.get_vertex(line.0) + (self.get_vertex(line.1) - self.get_vertex(line.0)) * t2;
                let dist = intersection.dist(start.position);
                if closest_intersection == None || closest_intersection_dist > dist {
                    closest_intersection = Some(intersection);
                    closest_intersection_dist = dist;
                }
            }
        }
        for target in &self.targets {
            // TODO: tune? fuzzy comparison for slope comparison
            let point2d = target.position.clone().without_z();
            if (start.position.angle_to(point2d) - start.angle).abs() < 0.01 {
                let dist = point2d.dist(start.position);
                if closest_intersection == None || closest_intersection_dist > dist {
                    closest_intersection = Some(point2d);
                    closest_intersection_dist = dist;
                }
            }
        }
        closest_intersection
    }

    // TODO: name this wtf
    pub fn cull_points(&self, start: Pose, fov: Point) -> Vec<Pose3D> {
        let mut sensed_objects = Vec::new();
        let start_pose: Point3D = start.position.into();
        for object in &self.targets {
            let object_angle = start_pose.angle_to(object.clone().position);
            let rel_angle = Point{x: PI - (PI - (start.angle - object_angle.x).abs()).abs(), y: -object_angle.y};
            if fov.x / 2. >= rel_angle.x && fov.y / 2. >= rel_angle.y
                && self
                    .raycast(start.with_angle(object_angle.x)) 
                    .unwrap() // if this panics then soemthing went wrong. it should at least return object
                    == object.clone().position.without_z()
            {
                sensed_objects.push(Pose3D{
                    angle: Point{x: object.angle.x - start.angle, y: -object_angle.y},
                    position: object.position.clone() - start.position
                    });
            }
        }
        sensed_objects
    }
}
