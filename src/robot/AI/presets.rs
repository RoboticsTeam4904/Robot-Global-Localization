use crate::{
    robot::{
        ai::localization::{ErrorCalculator, ResampleNoiseCalculator, WeightCalculator},
        sensors::{LimitedSensor, Sensor},
    },
    utility::{Point, Pose},
};
use rand::{distributions::Normal, prelude::*};
use std::{f64::INFINITY, ops::Range};

/// Creates a `ResampleNoiseCalculator` which produces uniform noise within the range ±`angle_margin` ±`position_margin`
pub fn uniform_resampler(angle_margin: f64, position_margin: f64) -> ResampleNoiseCalculator {
    Box::new(move |_| {
        Pose::random_from_range(Pose {
            angle: angle_margin,
            position: (position_margin, position_margin).into(),
        })
    })
}

/// Creates a `ResmapleNoiseCalculator` which produces noise normally distributed with the standard deviations provided
pub fn normal_resampler(angle_dev: f64, position_dev: f64) -> ResampleNoiseCalculator {
    let angle_dist = Normal::new(0., angle_dev);
    let pos_dist = Normal::new(0., position_dev);
    Box::new(move |_| {
        let mut rng = thread_rng();
        Pose {
            angle: angle_dist.sample(&mut rng),
            position: (pos_dist.sample(&mut rng), pos_dist.sample(&mut rng)).into(),
        }
    })
}

/// Creates a `WeightCalculator` which takes `base` to the power of negative error
pub fn exp_weight(base: f64) -> WeightCalculator {
    Box::new(move |error| base.powf(-error))
}

/// Creates a `ErrorCalculator` which follows the following algorithm:
///
/// For each scan point in the observed scan, predict what the scan point would look like at that angle for the sample.
///
/// The first part of the error is
/// the sum of the differences between the observed scan point and the predicted scan point divided by the number of scan points
///
/// The second part of the error is the number of predicted scan points that did not exist put to the power of `discrepancy_pow`.
///
/// Total error is the first part plus the second part multiplied by `error_scale`
pub fn lidar_error<S>(discrepancy_pow: f64, error_scale: f64) -> ErrorCalculator<S>
where
    S: Sensor<Output = Vec<Point>> + LimitedSensor<Range<f64>>,
{
    Box::new(move |&sample, lidar, map| {
        let sample = sample + lidar.relative_pose();
        let lidar_scan = lidar.sense();
        let len = lidar_scan.len() as f64;
        let lidar_range = lidar.range().unwrap_or(0.0..INFINITY);
        let mut error = 0.;
        let mut discrepancy_count: i32 = 0;
        for scan_point in lidar_scan {
            match map.raycast(
                sample
                    + Pose {
                        angle: scan_point.angle(),
                        ..Pose::default()
                    },
            ) {
                Some(predicted_point)
                    if lidar_range.contains(&predicted_point.dist(sample.position)) =>
                {
                    error += (scan_point.mag() - predicted_point.dist(sample.position)).abs()
                }
                _ => discrepancy_count += 1,
            };
        }
        error_scale * (error + (discrepancy_count as f64).powf(discrepancy_pow)) / len
    })
}
