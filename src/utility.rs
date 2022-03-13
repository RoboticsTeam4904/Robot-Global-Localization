use crate::map::Map2D;
use nalgebra::RowVector6;
use rand::prelude::*;
use serde::{de::DeserializeOwned, Serialize};
use serde_derive::Deserialize;
use std::{
    f64::consts::{FRAC_PI_2, PI},
    ops::Range,
    sync::Arc,
};

pub const GRAVITY: f64 = 9800.;
/// Generic 2d point
#[derive(Default, Debug, PartialEq, Clone, Copy, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub fn polar(angle: f64, radius: f64) -> Self {
        Self {
            x: radius * angle.cos(),
            y: radius * angle.sin(),
        }
    }

    pub fn normalize(self) -> Self {
        self / self.mag()
    }

    pub fn rotate(self, angle: f64) -> Self {
        let start_angle = self.angle();
        let mag = self.mag();
        Self::polar(start_angle + angle, mag)
    }

    pub fn clamp(self, lower: Point, upper: Point) -> Point {
        Point {
            x: if self.x > upper.x {
                upper.x
            } else if self.x < lower.x {
                lower.x
            } else {
                self.x
            },
            y: if self.y > upper.y {
                upper.y
            } else if self.y < lower.y {
                lower.y
            } else {
                self.y
            },
        }
    }

    /// Angle of `self` relative to origin
    pub fn angle(&self) -> f64 {
        self.angle_to(Point::default())
    }

    /// Angle of `self` relative to `other`
    pub fn angle_to(&self, other: Point) -> f64 {
        (other.y - self.y).atan2(other.x - self.x)
    }

    pub fn mag(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }

    pub fn dist(&self, other: Point) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    pub fn dot(&self, other: Point) -> f64 {
        self.x * other.x + self.y * other.y
    }

    pub fn cross_mag(&self, other: Point) -> f64 {
        self.x * other.y - self.y * other.x
    }
}

impl Into<Point> for (f64, f64) {
    fn into(self) -> Point {
        Point {
            x: self.0,
            y: self.1,
        }
    }
}

impl Into<Point3D> for Point {
    fn into(self) -> Point3D {
        Point3D {
            x: self.x,
            y: self.y,
            z: 0.,
        }
    }
}

impl Into<[f64; 2]> for Point {
    fn into(self) -> [f64; 2] {
        [self.x, self.y]
    }
}

impl std::ops::Add for Point {
    type Output = Point;

    fn add(self, other: Point) -> Point {
        Point {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl std::ops::Sub for Point {
    type Output = Point;

    fn sub(self, other: Point) -> Point {
        Point {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl std::ops::Mul for Point {
    type Output = Point;

    fn mul(self, other: Point) -> Point {
        Point {
            x: self.x * other.x,
            y: self.y * other.y,
        }
    }
}

impl std::ops::Div for Point {
    type Output = Point;

    fn div(self, other: Point) -> Point {
        Point {
            x: self.x / other.x,
            y: self.y / other.y,
        }
    }
}

impl std::ops::Add<(f64, f64)> for Point {
    type Output = Point;

    fn add(self, other: (f64, f64)) -> Point {
        let other: Point = other.into();
        self + other
    }
}

impl std::ops::Sub<(f64, f64)> for Point {
    type Output = Point;

    fn sub(self, other: (f64, f64)) -> Point {
        let other: Point = other.into();
        self - other
    }
}

impl std::ops::Add<f64> for Point {
    type Output = Point;

    fn add(self, other: f64) -> Point {
        Point {
            x: self.x + other,
            y: self.y + other,
        }
    }
}

impl std::ops::Mul<f64> for Point {
    type Output = Point;

    fn mul(self, other: f64) -> Point {
        Point {
            x: self.x * other,
            y: self.y * other,
        }
    }
}

impl std::ops::Div<f64> for Point {
    type Output = Point;

    fn div(self, other: f64) -> Point {
        Point {
            x: self.x / other,
            y: self.y / other,
        }
    }
}

impl std::ops::AddAssign for Point {
    /// Does not normalize angle
    fn add_assign(&mut self, other: Point) {
        *self = Point {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl std::ops::SubAssign for Point {
    /// Does not normalize angle
    fn sub_assign(&mut self, other: Point) {
        *self = Point {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

/// Generic 2d point
#[derive(Default, Debug, PartialEq, Clone, Copy, Serialize, Deserialize)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3D {
    pub fn cylindrical(angle: f64, radius: f64, z: f64) -> Self {
        Self {
            x: radius * angle.cos(),
            y: radius * angle.sin(),
            z,
        }
    }
    pub fn spherical(inclination: f64, azimuth: f64, radius: f64) -> Self {
        Self {
            x: radius * inclination.cos() * azimuth.cos(),
            y: radius * inclination.cos() * azimuth.sin(),
            z: radius * inclination.sin(),
        }
    }

    pub fn normalize(self) -> Self {
        self / self.mag()
    }

    pub fn rotate(self, inclination: f64, azimuth: f64) -> Self {
        Self::spherical(
            self.inclination() + inclination,
            self.azimuth() + azimuth,
            self.mag(),
        )
    }

    pub fn clamp(self, lower: Self, upper: Self) -> Self {
        Self {
            x: if self.x > upper.x {
                upper.x
            } else if self.x < lower.x {
                lower.x
            } else {
                self.x
            },
            y: if self.y > upper.y {
                upper.y
            } else if self.y < lower.y {
                lower.y
            } else {
                self.y
            },
            z: if self.z > upper.z {
                upper.z
            } else if self.z < lower.z {
                lower.z
            } else {
                self.z
            },
        }
    }

    /// Angle of `self` relative to origin
    pub fn azimuth(&self) -> f64 {
        self.azimuth_to(Point3D::default())
    }

    /// Angle of `self` relative to origin
    pub fn inclination(&self) -> f64 {
        self.inclination_to(Point3D::default())
    }

    /// Angles of `self` relative to `other`
    pub fn angle_to(&self, other: Point3D) -> Point {
        let point: Point = self.clone().without_z();
        Point {
            x: point.angle_to(other.without_z()),
            y: (other.z - self.z).atan2(point.mag()),
        }
    }

    /// Angles of `self` relative to `other`
    pub fn azimuth_to(&self, other: Point3D) -> f64 {
        let point: Point = self.clone().without_z();
        point.angle_to(other.without_z())
    }

    /// Angles of `self` relative to `other`
    pub fn inclination_to(&self, other: Point3D) -> f64 {
        let point: Point = self.clone().without_z();
        (other.z - self.z).atan2(point.mag())
    }

    pub fn mag(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    pub fn dist(&self, other: Point3D) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2) + (self.z - other.z).powi(2))
            .sqrt()
    }

    pub fn dot(&self, other: Point3D) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn without_z(&self) -> Point {
        Point {
            x: self.x,
            y: self.y,
        }
    }
}

impl Into<Point3D> for (f64, f64, f64) {
    fn into(self) -> Point3D {
        Point3D {
            x: self.0,
            y: self.1,
            z: self.2,
        }
    }
}

impl Into<[f64; 3]> for Point3D {
    fn into(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
}
impl From<[f64; 3]> for Point3D {
    fn from(arr: [f64; 3]) -> Self {
        Point3D {
            x: *arr.get(0).unwrap(),
            y: *arr.get(1).unwrap(),
            z: *arr.get(2).unwrap(),
        }
    }
}

impl std::ops::Add for Point3D {
    type Output = Point3D;

    fn add(self, other: Point3D) -> Point3D {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl std::ops::Sub for Point3D {
    type Output = Point3D;

    fn sub(self, other: Point3D) -> Point3D {
        Point3D {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl std::ops::Add<Point> for Point3D {
    type Output = Point3D;

    fn add(self, other: Point) -> Point3D {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z,
        }
    }
}

impl std::ops::Sub<Point> for Point3D {
    type Output = Point3D;

    fn sub(self, other: Point) -> Point3D {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z,
        }
    }
}

impl std::ops::Mul for Point3D {
    type Output = Point3D;

    fn mul(self, other: Self) -> Self {
        Point3D {
            x: self.x * other.x,
            y: self.y * other.y,
            z: self.z * other.z,
        }
    }
}

impl std::ops::Div for Point3D {
    type Output = Self;

    fn div(self, other: Self) -> Self {
        Self {
            x: self.x / other.x,
            y: self.y / other.y,
            z: self.z / other.z,
        }
    }
}

impl std::ops::Add<(f64, f64, f64)> for Point3D {
    type Output = Point3D;

    fn add(self, other: (f64, f64, f64)) -> Point3D {
        let other: Point3D = other.into();
        self + other
    }
}

impl std::ops::Sub<(f64, f64, f64)> for Point3D {
    type Output = Self;

    fn sub(self, other: (f64, f64, f64)) -> Self {
        let other: Self = other.into();
        self - other
    }
}

impl std::ops::Add<f64> for Point3D {
    type Output = Self;

    fn add(self, other: f64) -> Self {
        Self {
            x: self.x + other,
            y: self.y + other,
            z: self.z + other,
        }
    }
}

impl std::ops::Mul<f64> for Point3D {
    type Output = Self;

    fn mul(self, other: f64) -> Point3D {
        Point3D {
            x: self.x * other,
            y: self.y * other,
            z: self.z * other,
        }
    }
}

impl std::ops::Div<f64> for Point3D {
    type Output = Point3D;

    fn div(self, other: f64) -> Point3D {
        Point3D {
            x: self.x / other,
            y: self.y / other,
            z: self.z / other,
        }
    }
}

impl std::ops::AddAssign for Point3D {
    /// Does not normalize angle
    fn add_assign(&mut self, other: Self) {
        *self = Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl std::ops::SubAssign for Point3D {
    /// Does not normalize angle
    fn sub_assign(&mut self, other: Self) {
        *self = Point3D {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct KinematicState {
    pub angle: f64,
    pub position: Point,
    pub vel_angle: f64,
    pub velocity: Point,
}

impl KinematicState {
    /// Creates a random pose from uniform distribitions for each range
    pub fn random(
        angle_range: Range<f64>,
        x_range: Range<f64>,
        y_range: Range<f64>,
        angle_vel_range: Range<f64>,
        x_vel_range: Range<f64>,
        y_vel_range: Range<f64>,
    ) -> KinematicState {
        let mut rng = thread_rng();
        KinematicState {
            angle: rng.gen_range(angle_range),
            position: Point {
                x: rng.gen_range(x_range),
                y: rng.gen_range(y_range),
            },
            vel_angle: rng.gen_range(angle_vel_range),
            velocity: Point {
                x: rng.gen_range(x_vel_range),
                y: rng.gen_range(y_vel_range),
            },
        }
    }

    pub fn from_pose(pose: Pose) -> Self {
        Self {
            angle: pose.angle,
            position: pose.position,
            vel_angle: 0.,
            velocity: Point::default(),
        }
    }

    pub fn random_from_range(range: KinematicState) -> KinematicState {
        KinematicState::random(
            -range.angle..range.angle,
            -range.position.x..range.position.x,
            -range.position.y..range.position.y,
            -0.001..0.001,
            -0.001..0.001,
            -0.001..0.001,
        )
    }

    pub fn pose(&self) -> Pose {
        Pose {
            position: self.position,
            angle: self.angle,
        }
    }

    /// Mod `angle` by 2π
    /// and make magnitude of `position`
    /// 1
    pub fn normalize(mut self) -> KinematicState {
        self.angle %= 2. * PI;
        self.position = self.position.normalize();
        self
    }

    pub fn control_update(&mut self, control: Pose, delta_t: f64) {
        self.velocity.x += (control.position.x * self.angle.cos()
            + control.position.y * (self.angle - FRAC_PI_2).cos())
            * delta_t;
        self.velocity.y += (control.position.x * self.angle.sin()
            + control.position.y * (self.angle - FRAC_PI_2).sin())
            * delta_t;
        self.vel_angle += control.angle * delta_t;
        self.position.x += self.velocity.x * delta_t;
        self.position.y += self.velocity.y * delta_t;

        self.angle = (self.angle + self.vel_angle * delta_t) % (2. * PI);
    }

    pub fn mapped_control_update(
        &mut self,
        control: Pose,
        delta_t: f64,
        maps: Vec<Arc<Map2D>>,
    ) -> bool {
        self.velocity.x += (control.position.x * self.angle.cos()
            + control.position.y * (self.angle - FRAC_PI_2).cos())
            * delta_t;

        self.velocity.y += (control.position.x * self.angle.sin()
            + control.position.y * (self.angle - FRAC_PI_2).sin())
            * delta_t;

        self.vel_angle += control.angle * delta_t;

        let init_pose = Pose {
            angle: (self.velocity.y).atan2(self.velocity.x),
            position: self.position,
        };

        self.position.x += self.velocity.x * delta_t;
        self.position.y += self.velocity.y * delta_t;
        self.angle = (self.angle + self.vel_angle * delta_t) % (2. * PI);

        if let Some(end_point) = Map2D::raycast_with_maps(init_pose, maps) {
            let max_length = (end_point - init_pose.position).mag();
            if max_length <= (self.position - init_pose.position).mag() {
                self.position = init_pose.position
                    + (end_point - init_pose.position) / max_length * (max_length - 0.01);
                return true;
            }
        }

        false
    }

    pub fn clamp(&mut self, range: Range<Point>) -> Pose {
        let clamped_position = self.position.clamp(range.start, range.end);
        if clamped_position == self.position {
            return Pose::default();
        }
        let mut diff_vel = Pose::default();
        diff_vel.angle = -self.vel_angle;
        self.vel_angle = 0.;

        self.velocity.x = if clamped_position.x == self.position.x {
            self.velocity.x
        } else {
            diff_vel.position.x = -self.velocity.x;
            0.
        };
        self.velocity.y = if clamped_position.y == self.position.y {
            self.velocity.y
        } else {
            diff_vel.position.y = -self.velocity.y;
            0.
        };
        self.position = clamped_position;
        diff_vel
    }

    pub fn with_angle(mut self, angle: f64) -> KinematicState {
        self.angle = angle;
        self
    }

    pub fn with_position(mut self, position: Point) -> KinematicState {
        self.position = position;
        self
    }

    pub fn with_velocity(mut self, velocity: Point) -> KinematicState {
        self.velocity = velocity;
        self
    }
}

impl From<RowVector6<f64>> for KinematicState {
    fn from(vector: RowVector6<f64>) -> KinematicState {
        KinematicState {
            angle: *vector.index(0),
            position: Point {
                x: *vector.index(1),
                y: *vector.index(2),
            },
            vel_angle: *vector.index(3),
            velocity: Point {
                x: *vector.index(4),
                y: *vector.index(5),
            },
        }
    }
}

impl Into<RowVector6<f64>> for KinematicState {
    fn into(self) -> RowVector6<f64> {
        RowVector6::from_iterator(vec![
            self.angle,
            self.position.x,
            self.position.y,
            self.vel_angle,
            self.velocity.x,
            self.velocity.y,
        ])
    }
}

impl Into<Vec<f64>> for KinematicState {
    fn into(self) -> Vec<f64> {
        vec![
            self.angle,
            self.position.x,
            self.position.y,
            self.vel_angle,
            self.velocity.x,
            self.velocity.y,
        ]
    }
}

impl std::ops::Add for KinematicState {
    type Output = KinematicState;

    /// Does normalize angle
    fn add(self, other: KinematicState) -> KinematicState {
        KinematicState {
            angle: (self.angle + other.angle) % (2. * PI),
            position: self.position + other.position,
            vel_angle: self.vel_angle + other.vel_angle,
            velocity: self.velocity + other.velocity,
        }
    }
}

impl std::ops::Sub for KinematicState {
    type Output = KinematicState;

    /// Does normalize angle
    fn sub(self, other: KinematicState) -> KinematicState {
        KinematicState {
            angle: (self.angle - other.angle) % (2. * PI),
            position: self.position - other.position,
            vel_angle: self.vel_angle - other.vel_angle,
            velocity: self.velocity - other.velocity,
        }
    }
}

impl std::ops::Div<f64> for KinematicState {
    type Output = KinematicState;

    /// Does normalize angle
    fn div(self, other: f64) -> KinematicState {
        KinematicState {
            angle: (self.angle / other) % (2. * PI),
            position: self.position * (1. / other),
            vel_angle: (self.vel_angle / other),
            velocity: self.velocity * (1. / other),
        }
    }
}

impl std::ops::AddAssign for KinematicState {
    /// Does not normalize angle
    fn add_assign(&mut self, other: KinematicState) {
        *self = KinematicState {
            angle: self.angle + other.angle,
            position: self.position + other.position,
            vel_angle: self.vel_angle + other.vel_angle,
            velocity: self.velocity + other.velocity,
        }
    }
}
#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub angle: f64,
    pub position: Point,
}

impl Pose {
    /// Creates a random pose from uniform distribitions for each range
    pub fn random(angle_range: Range<f64>, x_range: Range<f64>, y_range: Range<f64>) -> Pose {
        let mut rng = thread_rng();
        Pose {
            angle: rng.gen_range(angle_range),
            position: Point {
                x: rng.gen_range(x_range),
                y: rng.gen_range(y_range),
            },
        }
    }

    pub fn random_from_range(range: Pose) -> Pose {
        Pose::random(
            -range.angle..range.angle,
            -range.position.x..range.position.x,
            -range.position.y..range.position.y,
        )
    }

    /// Mod `angle` by 2π
    pub fn normalize(mut self) -> Pose {
        self.angle %= 2. * PI;
        self
    }

    pub fn clamp(self, lower: Pose, upper: Pose) -> Pose {
        Pose {
            angle: if self.angle > upper.angle {
                upper.angle
            } else if self.angle < lower.angle {
                lower.angle
            } else {
                self.angle
            },
            position: self.position.clamp(lower.position, upper.position),
        }
    }

    pub fn with_angle(mut self, angle: f64) -> Pose {
        self.angle = angle;
        self
    }

    pub fn with_position(mut self, position: Point) -> Pose {
        self.position = position;
        self
    }
}

impl std::ops::Add for Pose {
    type Output = Pose;

    /// Does normalize angle
    fn add(self, other: Pose) -> Pose {
        Pose {
            angle: (self.angle + other.angle) % (2. * PI),
            position: self.position + other.position,
        }
    }
}

impl std::ops::Sub for Pose {
    type Output = Pose;

    /// Does normalize angle
    fn sub(self, other: Pose) -> Pose {
        Pose {
            angle: (self.angle - other.angle) % (2. * PI),
            position: self.position - other.position,
        }
    }
}

impl std::ops::Div<f64> for Pose {
    type Output = Pose;

    /// Does normalize angle
    fn div(self, other: f64) -> Pose {
        Pose {
            angle: (self.angle / other) % (2. * PI),
            position: self.position * (1. / other),
        }
    }
}

impl std::ops::Mul<f64> for Pose {
    type Output = Pose;

    /// Does normalize angle
    fn mul(self, other: f64) -> Pose {
        Pose {
            angle: (self.angle * other) % (2. * PI),
            position: self.position * other,
        }
    }
}

impl std::ops::AddAssign for Pose {
    /// Does not normalize angle
    fn add_assign(&mut self, other: Pose) {
        *self = Pose {
            angle: self.angle + other.angle,
            position: self.position + other.position,
        }
    }
}

impl From<Vec<f64>> for Pose {
    fn from(vector: Vec<f64>) -> Pose {
        Pose {
            angle: *vector.get(0).unwrap(),
            position: Point {
                x: *vector.get(1).unwrap(),
                y: *vector.get(2).unwrap(),
            },
        }
    }
}

impl Into<Vec<f64>> for Pose {
    fn into(self) -> Vec<f64> {
        vec![self.angle, self.position.x, self.position.y]
    }
}

impl Into<Pose3D> for Pose {
    fn into(self) -> Pose3D {
        Pose3D {
            angle: Point {
                x: self.angle,
                y: 0.,
            },
            position: self.position.into(),
        }
    }
}

/// Clamps the `num` to the range `[lower, upper)`
///
/// If `T` is unsigned, do not use an `upper` of `0` because `upper` is tested exclusively
pub fn clamp<T>(num: T, lower: T, upper: Option<T>) -> T
where
    T: std::ops::Sub<Output = T> + From<i8> + PartialOrd,
{
    if num < lower {
        return lower;
    }
    if let Some(u) = upper {
        if num >= u {
            return u - 1i8.into();
        }
    }
    num
}

pub fn clamp_to_range<T>(num: f64, range: T) -> f64
where
    T: std::ops::RangeBounds<f64>,
{
    let num = num.into();
    let num = match range.start_bound() {
        std::ops::Bound::Excluded(lower) if num <= *lower => *lower + 1.,
        std::ops::Bound::Included(lower) if num < *lower => *lower,
        _ => num,
    };
    match range.end_bound() {
        std::ops::Bound::Excluded(upper) if num >= *upper => *upper - 1.,
        std::ops::Bound::Included(upper) if num > *upper => *upper,
        _ => num,
    }
    .into()
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pose3D {
    /// x coordinate is azimuth angle, y coordinate is inclination angle
    pub angle: Point,
    pub position: Point3D,
}

impl Pose3D {
    /// Creates a random pose from uniform distribitions for each range
    pub fn random(
        azimuth_range: Range<f64>,
        inclination_range: Range<f64>,
        x_range: Range<f64>,
        y_range: Range<f64>,
        z_range: Range<f64>,
    ) -> Pose3D {
        let mut rng = thread_rng();
        Pose3D {
            angle: Point {
                x: rng.gen_range(azimuth_range),
                y: rng.gen_range(inclination_range),
            },
            position: Point3D {
                x: rng.gen_range(x_range),
                y: rng.gen_range(y_range),
                z: rng.gen_range(z_range),
            },
        }
    }

    pub fn random_from_range(range: Pose3D) -> Pose3D {
        Pose3D::random(
            -range.angle.x..range.angle.x,
            -range.angle.y..range.angle.y,
            -range.position.x..range.position.x,
            -range.position.y..range.position.y,
            -range.position.z..range.position.z,
        )
    }

    /// Mod `angle` by 2π
    pub fn normalize(mut self) -> Pose3D {
        self.angle.x %= 2. * PI;
        self.angle.y %= 2. * PI;
        self
    }

    pub fn clamp(self, lower: Pose3D, upper: Pose3D) -> Pose3D {
        Pose3D {
            angle: self.angle.clamp(lower.angle, upper.angle),
            position: self.position.clamp(lower.position, upper.position),
        }
    }

    pub fn with_angle(self, angle: Point) -> Pose3D {
        Pose3D {
            angle: angle,
            position: self.position,
        }
    }

    pub fn with_position(self, position: Point3D) -> Pose3D {
        Pose3D {
            angle: self.angle,
            position: position,
        }
    }
}

impl std::ops::Add for Pose3D {
    type Output = Pose3D;

    /// Does normalize angle
    fn add(self, other: Pose3D) -> Pose3D {
        Pose3D {
            angle: Point {
                x: (self.angle.x + other.angle.x) % (2. * PI),
                y: (self.angle.y + other.angle.y) % (2. * PI),
            },
            position: self.position + other.position,
        }
    }
}

impl std::ops::Sub for Pose3D {
    type Output = Pose3D;

    /// Does normalize angle
    fn sub(self, other: Pose3D) -> Pose3D {
        Pose3D {
            angle: Point {
                x: (self.angle.x - other.angle.x) % (2. * PI),
                y: (self.angle.y - other.angle.y) % (2. * PI),
            },
            position: self.position - other.position,
        }
    }
}

impl std::ops::Add<Pose> for Pose3D {
    type Output = Pose3D;

    /// Does normalize angle
    fn add(self, other: Pose) -> Pose3D {
        Pose3D {
            angle: Point {
                x: (self.angle.x + other.angle) % (2. * PI),
                y: (self.angle.y) % (2. * PI),
            },
            position: self.position + other.position,
        }
    }
}

impl std::ops::Sub<Pose> for Pose3D {
    type Output = Pose3D;

    /// Does normalize angle
    fn sub(self, other: Pose) -> Pose3D {
        Pose3D {
            angle: Point {
                x: (self.angle.x - other.angle) % (2. * PI),
                y: self.angle.y,
            },
            position: self.position - other.position,
        }
    }
}

impl std::ops::Div<f64> for Pose3D {
    type Output = Pose3D;

    /// Does normalize angle
    fn div(self, other: f64) -> Pose3D {
        Pose3D {
            angle: Point {
                x: (self.angle.x / other) % (2. * PI),
                y: (self.angle.y / other) % (2. * PI),
            },
            position: self.position / other,
        }
    }
}

impl std::ops::Mul<f64> for Pose3D {
    type Output = Pose3D;

    /// Does normalize angle
    fn mul(self, other: f64) -> Pose3D {
        Pose3D {
            angle: Point {
                x: (self.angle.x * other) % (2. * PI),
                y: (self.angle.y * other) % (2. * PI),
            },
            position: self.position * other,
        }
    }
}

impl std::ops::AddAssign for Pose3D {
    /// Does not normalize angle
    fn add_assign(&mut self, other: Pose3D) {
        *self = Pose3D {
            angle: Point {
                x: (self.angle.x + other.angle.x) % (2. * PI),
                y: (self.angle.y + other.angle.y) % (2. * PI),
            },
            position: self.position + other.position,
        }
    }
}

impl From<Vec<f64>> for Pose3D {
    fn from(vector: Vec<f64>) -> Pose3D {
        Pose3D {
            angle: Point {
                x: *vector.get(0).unwrap(),
                y: *vector.get(1).unwrap(),
            },
            position: Point3D {
                x: *vector.get(2).unwrap(),
                y: *vector.get(3).unwrap(),
                z: *vector.get(4).unwrap(),
            },
        }
    }
}

impl Into<Vec<f64>> for Pose3D {
    fn into(self) -> Vec<f64> {
        vec![
            self.angle.x,
            self.angle.y,
            self.position.x,
            self.position.y,
            self.position.z,
        ]
    }
}

pub fn mean(list: &Vec<f64>) -> f64 {
    list.iter().sum::<f64>() / (list.len() as f64)
}

pub fn variance(list: &Vec<f64>) -> f64 {
    let average = mean(list);
    mean(
        &list
            .iter()
            .map(|elem| (average - elem).powi(2))
            .collect::<Vec<f64>>(),
    )
}

pub fn variance_poses(poses: &Vec<Pose>) -> Pose {
    let mut angles: Vec<f64> = Vec::new();
    let mut x_coords: Vec<f64> = Vec::new();
    let mut y_coords: Vec<f64> = Vec::new();
    for pose in poses {
        angles.push(pose.angle);
        x_coords.push(pose.position.x);
        y_coords.push(pose.position.y);
    }
    angles = angles
        .iter()
        .map(|angle| (angle - mean(&angles)).abs() % PI)
        .collect();
    Pose {
        angle: variance(&angles),
        position: Point {
            x: variance(&x_coords),
            y: variance(&y_coords),
        },
    }
}
