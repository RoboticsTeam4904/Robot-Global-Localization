/// Generic 2d point
#[derive(Debug, PartialEq)]
pub struct Point {
    x: f64,
    y: f64
}

impl Point {
    pub fn dist(&self, other: Point) -> f64 {
        ((self.x - other.x).powf(2.) + (self.y - other.y).powf(2.)).sqrt()
    }

    pub fn dot(&self, other: Point) -> f64 {
        self.x * other.x + self.y + other.y
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
