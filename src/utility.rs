use rand::prelude::*;
use std::ops::Range;
use std::f64::consts::{PI, FRAC_PI_2};

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

    fn add(self, other: (f64, f64)) -> Point  {
        let other: Point = other.into();
        self + other
    }
}

impl std::ops::Sub<(f64, f64)> for Point {
    type Output = Point;

    fn sub(self, other: (f64, f64)) -> Point  {
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

// pub fn clamp_to_range<T, U>(num: T, range: U) -> T
// where U: std::ops::RangeBounds<isize>, T: Into<isize> + From<isize>,
// {
//     let num = num.into();
//     let num = match range.start_bound() {
//         std::ops::Bound::Excluded(lower) if num <= *lower => *lower + 1isize,
//         std::ops::Bound::Included(lower) if num < *lower => *lower,
//         _ => num
//     };
//     match range.end_bound() {
//         std::ops::Bound::Excluded(upper) if num >= *upper => *upper - 1isize,
//         std::ops::Bound::Included(upper) if num > *upper => *upper,
//         _ => num
//     }.into()
// }
