use crate::map::Map2D;
use nalgebra::RowVector6;
use rand::prelude::*;
use std::{
    f64::consts::{FRAC_PI_2, PI},
    ops::Range,
    sync::Arc,
};

/// Generic 2d point
#[derive(Default, Debug, PartialEq, Clone, Copy)]
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
        let dif = other - *self;
        if dif.x == 0. {
            if other.y > self.y {
                FRAC_PI_2
            } else {
                PI + FRAC_PI_2
            }
        } else {
            let mut angle = (dif.y / dif.x).atan();
            if dif.x < 0. {
                angle += PI;
            }
            angle % (2. * PI)
        }
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
            angle: rng.gen_range(angle_range.start, angle_range.end),
            position: Point {
                x: rng.gen_range(x_range.start, x_range.end),
                y: rng.gen_range(y_range.start, y_range.end),
            },
            vel_angle: rng.gen_range(angle_vel_range.start, angle_vel_range.end),
            velocity: Point {
                x: rng.gen_range(x_vel_range.start, x_vel_range.end),
                y: rng.gen_range(y_vel_range.start, y_vel_range.end),
            },
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

    pub fn control_update(&mut self, control: Pose, delta_t: f64, map: &Arc<Map2D>) {
        let init_pose = Pose {
            angle: (self.velocity.y).atan2(self.velocity.x),
            position: self.position,
        };

        self.position.x += self.velocity.x * delta_t;
        self.position.y += self.velocity.y * delta_t;
        self.velocity.x += (control.position.x * self.angle.cos()
            + control.position.y * (self.angle - FRAC_PI_2).cos())
            * delta_t;
        self.velocity.y += (control.position.x * self.angle.sin()
            + control.position.y * (self.angle - FRAC_PI_2).sin())
            * delta_t;
        self.angle = (self.angle + self.vel_angle * delta_t) % (2. * PI);
        self.vel_angle += control.angle * delta_t;
        if let Some(end_point) = map.raycast(init_pose) {
            let max_length = (end_point - init_pose.position).mag();
            if max_length <= (self.position - init_pose.position).mag() {
                self.velocity = Point::default();
                self.vel_angle = 0.;
                self.position = init_pose.position
                    + (end_point - init_pose.position) / max_length * (max_length - 0.01)
            }
        }
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
        RowVector6::new(
            self.angle,
            self.position.x,
            self.position.y,
            self.vel_angle,
            self.velocity.x,
            self.velocity.y,
        )
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

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub angle: f64,
    pub position: Point,
}

impl Pose {
    /// Creates a random pose from uniform distribitions for each range
    pub fn random(angle_range: Range<f64>, x_range: Range<f64>, y_range: Range<f64>) -> Pose {
        let mut rng = thread_rng();
        Pose {
            angle: rng.gen_range(angle_range.start, angle_range.end),
            position: Point {
                x: rng.gen_range(x_range.start, x_range.end),
                y: rng.gen_range(y_range.start, y_range.end),
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
