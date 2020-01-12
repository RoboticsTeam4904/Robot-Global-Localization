use super::{LimitedSensor, Sensor};
use crate::robot::map::Map2D;
use crate::utility::{KinematicState, Point, Pose};
use rand::distributions::{Distribution, Normal};
use rand::thread_rng;
use std::sync::Arc;
use std::time::Instant;

/// A sensor which senses all objects' relative positions within a certain fov
pub struct DummyObjectSensor {
    pub map: Arc<Map2D>,
    pub relative_pose: KinematicState,
    pub robot_pose: KinematicState,
    fov: f64,
    x_noise_distr: Normal,
    y_noise_distr: Normal,
}

impl DummyObjectSensor {
    pub fn new(
        fov: f64,
        map: Arc<Map2D>,
        relative_pose: KinematicState,
        robot_pose: KinematicState,
        noise_margins: Point,
    ) -> Self {
        Self {
            fov,
            map,
            relative_pose,
            robot_pose,
            x_noise_distr: Normal::new(0., noise_margins.x / 3.),
            y_noise_distr: Normal::new(0., noise_margins.y / 3.),
        }
    }

    pub fn update_pose(&mut self, new_pose: KinematicState) {
        self.robot_pose = new_pose
    }
}

impl Sensor for DummyObjectSensor {
    type Output = Vec<Point>;

    fn relative_pose(&self) -> KinematicState {
        self.relative_pose
    }

    fn sense(&self) -> Self::Output {
        let sensor_pose = self.robot_pose + self.relative_pose();
        self.map
            .cull_points(sensor_pose, self.fov)
            .iter()
            .map(|o| {
                *o + Point {
                    x: self.x_noise_distr.sample(&mut thread_rng()),
                    y: self.y_noise_distr.sample(&mut thread_rng()),
                }
            })
            .collect()
    }

    fn sense_from_pose(&self, pose: KinematicState) -> Vec<Point> {
        let sensor_pose = self.robot_pose + pose;
        self.map
            .cull_points(sensor_pose, self.fov)
            .iter()
            .map(|o| {
                *o + Point {
                    x: self.x_noise_distr.sample(&mut thread_rng()),
                    y: self.y_noise_distr.sample(&mut thread_rng()),
                }
            })
            .collect()
    }
}

impl LimitedSensor<f64> for DummyObjectSensor {
    fn range(&self) -> Option<f64> {
        Some(self.fov)
    }
}

#[derive(Clone)]
pub struct DummyDistanceSensor {
    noise_distr: Normal,
    relative_pose: KinematicState,
    pub map: Arc<Map2D>,
    pub robot_pose: KinematicState,
    pub max_dist: Option<f64>,
}

impl DummyDistanceSensor {
    /// Noise is Guassian and `noise_margin` is each equal to three standard deviations of the noise distribution.
    /// `relative_pose` is the pose of the sensor relative to the robot
    pub fn new(
        noise_margin: f64,
        relative_pose: KinematicState,
        map: Arc<Map2D>,
        robot_pose: KinematicState,
        max_dist: Option<f64>,
    ) -> Self {
        Self {
            noise_distr: Normal::new(0., noise_margin / 3.),
            relative_pose,
            map,
            robot_pose,
            max_dist,
        }
    }

    pub fn update_pose(&mut self, new_pose: KinematicState) {
        self.robot_pose = new_pose
    }
}

impl Sensor for DummyDistanceSensor {
    type Output = f64;

    fn sense(&self) -> Self::Output {
        let sensor_pose = self.relative_pose + self.robot_pose;
        let ray = self.map.raycast(sensor_pose);
        if let Some(max_dist) = self.max_dist {
            let dist = match ray {
                Some(c) => c.dist(sensor_pose.position),
                None => max_dist,
            };
            if dist > max_dist {
                max_dist
            } else {
                dist + self.noise_distr.sample(&mut thread_rng())
            }
        } else {
            let dist = match ray {
                Some(c) => c.dist(sensor_pose.position),
                None => unreachable!(),
            };
            dist + self.noise_distr.sample(&mut thread_rng())
        }
    }

    fn sense_from_pose(&self, pose: KinematicState) -> f64 {
        let sensor_pose = self.relative_pose + pose;
        let ray = self.map.raycast(sensor_pose);
        if let Some(max_dist) = self.max_dist {
            let dist = match ray {
                Some(c) => c.dist(sensor_pose.position),
                None => max_dist,
            };
            if dist > max_dist {
                max_dist
            } else {
                dist + self.noise_distr.sample(&mut thread_rng())
            }
        } else {
            let dist = match ray {
                Some(c) => c.dist(sensor_pose.position),
                None => unreachable!(),
            };
            dist + self.noise_distr.sample(&mut thread_rng())
        }
    }

    fn relative_pose(&self) -> KinematicState {
        self.relative_pose
    }
}

impl LimitedSensor<f64> for DummyDistanceSensor {}

pub struct DummyAccelerationSensor {
    x_noise_distr: Normal,
    y_noise_distr: Normal,
    angle_noise_distr: Normal,
}

impl DummyAccelerationSensor {
    pub fn new(noise_margins: Pose) -> Self {
        Self {
            x_noise_distr: Normal::new(0., noise_margins.position.x / 3.),
            y_noise_distr: Normal::new(0., noise_margins.position.x / 3.),
            angle_noise_distr: Normal::new(0., noise_margins.angle / 3.),
        }
    }
}

impl Sensor for DummyAccelerationSensor {
    type Output = (Pose, Pose);

    fn sense(&self) -> Self::Output {
        return (Pose::default(), Pose::default());
    }

    fn sense_from_pose(&self, pose: KinematicState) -> Self::Output {
        return (Pose::default(), Pose::default());
    }
}

pub struct DummyPositionSensor {
    angle_noise_distr: Normal,
    x_noise_distr: Normal,
    y_noise_distr: Normal,
    x_vel_noise_distr: Normal,
    y_vel_noise_distr: Normal,
    angle_vel_noise_distr: Normal,
    prev_robot_pose: KinematicState,
    pub robot_pose: KinematicState,
}

impl DummyPositionSensor {
    /// Noise is Guassian and `noise_margins` are each equal to three standard deviations of the noise distributions
    pub fn new(robot_pose: KinematicState, noise_margins: KinematicState) -> Self {
        Self {
            angle_noise_distr: Normal::new(0., noise_margins.angle / 3.),
            x_noise_distr: Normal::new(0., noise_margins.position.x / 3.),
            y_noise_distr: Normal::new(0., noise_margins.position.y / 3.),
            x_vel_noise_distr: Normal::new(0., noise_margins.velocity.x / 3.),
            y_vel_noise_distr: Normal::new(0., noise_margins.velocity.y / 3.),
            angle_vel_noise_distr: Normal::new(0., noise_margins.vel_angle / 3.),
            prev_robot_pose: robot_pose,
            robot_pose,
        }
    }

    pub fn update_pose(&mut self, new_pose: KinematicState) {
        self.prev_robot_pose = self.robot_pose;
        self.robot_pose = new_pose;
    }
}

impl Sensor for DummyPositionSensor {
    type Output = KinematicState;

    fn sense(&self) -> Self::Output {
        let mut rng = thread_rng();
        self.robot_pose - self.prev_robot_pose
            + KinematicState {
                angle: self.angle_noise_distr.sample(&mut rng),
                position: Point {
                    x: self.x_noise_distr.sample(&mut rng),
                    y: self.y_noise_distr.sample(&mut rng),
                },
                vel_angle: self.angle_vel_noise_distr.sample(&mut rng),
                velocity: Point {
                    x: self.x_vel_noise_distr.sample(&mut rng),
                    y: self.y_vel_noise_distr.sample(&mut rng),
                },
            }
    }

    fn sense_from_pose(&self, pose: KinematicState) -> Self::Output {
        let mut rng = thread_rng();
        pose - self.prev_robot_pose
            + KinematicState {
                angle: self.angle_noise_distr.sample(&mut rng),
                position: Point {
                    x: self.x_noise_distr.sample(&mut rng),
                    y: self.y_noise_distr.sample(&mut rng),
                },
                vel_angle: self.angle_vel_noise_distr.sample(&mut rng),
                velocity: Point {
                    x: self.x_vel_noise_distr.sample(&mut rng),
                    y: self.y_vel_noise_distr.sample(&mut rng),
                },
            }
    }
}

pub struct DummyMotionSensor {
    left_noise_distr: Normal,
    right_noise_distr: Normal,
    prev_robot_vel: Point,
    prev_measurement_timestep: Instant,
    pub robot_vel: Point,
}

impl Sensor for DummyMotionSensor {
    type Output = Point;

    fn update(&mut self) {
        self.prev_measurement_timestep = Instant::now();
    }

    fn sense(&self) -> Self::Output {
        let mut rng = thread_rng();
        (self.robot_vel - self.prev_robot_vel)
            / self
                .prev_measurement_timestep
                .duration_since(Instant::now())
                .as_millis() as f64
            / 1000.
            + Point {
                x: self.left_noise_distr.sample(&mut rng),
                y: self.left_noise_distr.sample(&mut rng),
            }
    }

    fn sense_from_pose(&self, pose: KinematicState) -> Self::Output {
        Point::default()
    }
}
