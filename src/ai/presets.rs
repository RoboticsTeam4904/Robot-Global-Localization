use crate::{
    ai::localization::{ErrorCalculator, ResampleNoiseCalculator, WeightCalculator},
    map::Map2D,
    sensors::{LimitedSensor, Sensor},
    utility::{Point, Pose, Pose3D},
};
use rand::prelude::*;
use rand_distr::Normal;
use rayon::prelude::*;
use std::{
    f64::{consts::PI, INFINITY},
    ops::Range,
    sync::Arc,
};

/// Creates a `ResampleNoiseCalculator` which produces uniform noise within the range ±`angle_margin` ±`position_margin`
pub fn uniform_resampler(angle_margin: f64, position_margin: f64) -> impl ResampleNoiseCalculator {
    move |_| {
        Pose::random_from_range(Pose {
            angle: angle_margin,
            position: (position_margin, position_margin).into(),
        })
    }
}

/// Creates a `ResmapleNoiseCalculator` which produces noise normally distributed with the standard deviations provided
pub fn normal_resampler(angle_dev: f64, position_dev: f64) -> impl ResampleNoiseCalculator {
    let angle_dist = Normal::new(0., angle_dev).unwrap();
    let pos_dist = Normal::new(0., position_dev).unwrap();
    move |_| {
        let mut rng = thread_rng();
        Pose {
            angle: angle_dist.sample(&mut rng),
            position: (pos_dist.sample(&mut rng), pos_dist.sample(&mut rng)).into(),
        }
    }
}

/// Creates a `WeightCalculator` which takes `base` to the power of negative error
pub fn exp_weight(base: f64) -> impl WeightCalculator {
    move |error: &f64| base.powf(-error)
}

/// Creates a `ErrorCalculator` for a sensor which senses a point cloud of scans and has a range described by `Range<f64>`
///
/// The return `ErrorCalculator`  follows the following algorithm:
///
/// For each scan point in the observed scan, predict what the scan point would look like at that angle for the sample.
///
/// The first part of the error is
/// the sum of the differences between the observed scan point and the predicted scan point divided by the number of scan points
///
/// The second part of the error is the number of predicted scan points that did not exist put to the power of `discrepancy_pow`.
///
/// Total error is the first part plus the second part multiplied by `error_scale`
pub fn lidar_error<S>(discrepancy_pow: f64, error_scale: f64) -> impl ErrorCalculator<S>
where
    S: Sensor<Output = Vec<Point>> + LimitedSensor<Range<f64>>,
{
    move |&sample: &Pose, lidar: &S, map: &Arc<Map2D>| -> f64 {
        let sample = sample + lidar.relative_pose();
        let lidar_scan = lidar.sense();
        let len = lidar_scan.len() as f64;
        let lidar_range = lidar.range().unwrap_or(0.0..INFINITY);
        let error: (f64, f64) = lidar_scan // TODO: this parallelization could be better ith
            .par_iter()
            .map(|scan_point| {
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
                        (
                            (scan_point.mag() - predicted_point.dist(sample.position)).abs(),
                            0.,
                        )
                    }
                    _ => (0., 1.),
                }
            })
            .reduce(|| (0., 0.), |a, b| (a.0 + b.0, a.1 + b.1));
        error_scale * (error.0 + error.1.powf(discrepancy_pow)) / len
    }
}

/// Creates an `ErrorCalculator` for a sensor which detects objects in its vicinity
/// and is bounded by an fov returned by its impl of `LimitedSensor<f64>`
/// and a detection range returned by its impl of `LimitedSensor<Range<f64>>`.
///
/// It calculates error based on the difference in position
/// of each detected object and
/// `discrepancy_factor` multiplied by the difference in length of the expected objects
/// and predicted objects.
pub fn object_detection_error<S>(
    discrepancy_factor: f64,
    error_scale: f64,
) -> impl ErrorCalculator<S>
where
    S: Sensor<Output = Vec<Point>> + LimitedSensor<(f64, f64)>,
{
    move |&sample: &Pose, object_detector: &S, map: &Arc<Map2D>| {
        let fov: f64;
        let max_dist: Option<f64>;
        if let Some((sensor_fov, sensor_max_dist)) = object_detector.range() {
            fov = sensor_fov;
            max_dist = Some(sensor_max_dist);
        } else {
            fov = 2. * PI;
            max_dist = None;
        }

        let mut sum_error = 0.;
        let mut pred_observation: Vec<Point> = map
            .cull_points(
                sample + object_detector.relative_pose(),
                Point { x: fov, y: 2. * PI },
                max_dist,
            )
            .iter()
            .map(|elem| elem.position.clone().without_z())
            .collect();
        let mut observation = object_detector.sense();
        observation.sort_by(|a, b| a.mag().partial_cmp(&b.mag()).unwrap());
        pred_observation.sort_by(|a, b| a.mag().partial_cmp(&b.mag()).unwrap());
        // TODO: This method of calculating error is not entirely sound
        for (&real, &pred) in observation.iter().zip(pred_observation.iter()) {
            sum_error += (real - pred).mag();
        }
        sum_error +=
            discrepancy_factor * (observation.len() as f64 - pred_observation.len() as f64).abs();
        sum_error * error_scale
    }
}

// pub fn point_3d_detection_error<S>(
//     discrepancy_factor: f64,
//     angle_factor: f64,
//     error_scale: f64,
// ) -> impl ErrorCalculator<S>
// where
//     S: Sensor<Output = Vec<Point>> + LimitedSensor<(Point, f64)>,
// {
//     move |&sample: &Pose, object_detector: &S, map: &Arc<Map2D>| {
//         let fov: Point;
//         let max_dist: Option<f64>;
//         if let Some((sensor_fov, sensor_max_dist)) = object_detector.range() {
//             fov = sensor_fov;
//             max_dist = Some(sensor_max_dist);
//         } else {
//             fov = (2. * PI, 2. * PI).into();
//             max_dist = None;
//         }

//         let mut sum_error = 0.;
//         let mut pred_observation: Vec<Pose3D> = map
//             .cull_points(sample + object_detector.relative_pose(), fov, max_dist)
//             .iter()
//             .map(|elem| elem.clone())
//             .collect();
//         let mut observation = object_detector.sense();
//         observation.sort_by(|a, b| a.mag().partial_cmp(&b.mag()).unwrap());
//         pred_observation.sort_by(|a, b| a.position.mag().partial_cmp(&b.position.mag()).unwrap());
//         // TODO: This method of calculating error is not entirely sound
//         for (&real, &pred) in observation.iter().zip(pred_observation.iter()) {
//             sum_error += (real - pred.position).mag();
//         }
//         sum_error +=
//             discrepancy_factor * (observation.len() as f64 - pred_observation.len() as f64).abs();
//         sum_error * error_scale
//     }
// }

pub fn object_3d_detection_error<S>(
    discrepancy_factor: f64,
    angle_factor: f64,
    error_scale: f64,
) -> impl ErrorCalculator<S>
where
    S: Sensor<Output = Vec<Pose3D>> + LimitedSensor<(Point, f64)>,
{
    move |&sample: &Pose, object_detector: &S, map: &Arc<Map2D>| {
        let fov: Point;
        let max_dist: Option<f64>;
        if let Some((sensor_fov, sensor_max_dist)) = object_detector.range() {
            fov = sensor_fov;
            max_dist = Some(sensor_max_dist);
        } else {
            fov = (2. * PI, 2. * PI).into();
            max_dist = None;
        }

        let mut sum_error = 0.;
        let mut pred_observation: Vec<Pose3D> = map
            .cull_points(sample + object_detector.relative_pose(), fov, max_dist)
            .iter()
            .map(|elem| elem.clone())
            .collect();
        let mut observation = object_detector.sense();
        observation.sort_by(|a, b| a.position.mag().partial_cmp(&b.position.mag()).unwrap());
        pred_observation.sort_by(|a, b| a.position.mag().partial_cmp(&b.position.mag()).unwrap());
        // TODO: This method of calculating error is not entirely sound
        for (&real, &pred) in observation.iter().zip(pred_observation.iter()) {
            sum_error += (real.position - pred.position).mag();
            sum_error += angle_factor * (real.angle - pred.angle).mag();
        }
        sum_error +=
            discrepancy_factor * (observation.len() as f64 - pred_observation.len() as f64).abs();
        sum_error * error_scale
    }
}
