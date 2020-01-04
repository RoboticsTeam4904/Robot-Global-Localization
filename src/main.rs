#![allow(dead_code)] // TODO: This will be removed after organizational overhall and a lib.rs
mod robot;
mod utility;

use robot::ai::localization::{DistanceFinderMCL, ObjectDetectorMCL};
use robot::map::{Map2D, Object2D};
use robot::sensors::dummy::{DummyDistanceSensor, DummyMotionSensor, DummyObjectSensor};
use std::f64::consts::{FRAC_PI_8, PI};
use std::sync::Arc;

fn main() {
    loop {}
}
