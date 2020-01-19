use nalgebra::RowVector6;
use rand::prelude::*;
use std::f64::consts::{FRAC_PI_2, PI};
use std::ops::Range;

/// Generic 2d point
#[derive(Default, Debug, PartialEq, Clone, Copy)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
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

    /// Angle of `self` relative to `other`
    pub fn angle(&self, other: Point) -> f64 {
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
    pub fn normalize(mut self) -> KinematicState {
        self.angle %= 2. * PI;
        self
    }

    pub fn clamp_control_update(self, range: Range<Point>) -> KinematicState {
        let clamped_position = self.position.clamp(range.start, range.end);
        KinematicState {
            angle: self.angle,
            position: clamped_position,
            vel_angle: if clamped_position != self.position {
                0.
            } else {
                self.vel_angle
            },
            velocity: if clamped_position.x != self.position.x
                && clamped_position.y != self.position.y
            {
                Point { x: 0., y: 0. }
            } else if clamped_position.x != self.position.x {
                Point {
                    x: 0.,
                    y: self.velocity.y,
                }
            } else if clamped_position.y != self.position.y {
                Point {
                    x: self.velocity.x,
                    y: 0.,
                }
            } else {
                Point {
                    x: self.velocity.x,
                    y: self.velocity.y,
                }
            },
        }
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
            angle: self.angle + other.angle,
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
    where T: std::ops::RangeBounds<f64>,
{
    let num = num.into();
    let num = match range.start_bound() {
        std::ops::Bound::Excluded(lower) if num <= *lower => *lower + 1.,
        std::ops::Bound::Included(lower) if num < *lower => *lower,
        _ => num
    };
    match range.end_bound() {
        std::ops::Bound::Excluded(upper) if num >= *upper => *upper - 1.,
        std::ops::Bound::Included(upper) if num > *upper => *upper,
        _ => num
    }.into()
}
