use crate::{
    map::Map2D,
    sensors::{LimitedSensor, Sensor, SensorSink},
    utility::{Point, Pose},
};
use rand::{
    distributions::{Distribution, Normal},
    thread_rng,
};
use std::{
    f64::{consts::PI, INFINITY},
    ops::Range,
    sync::Arc,
    time::Instant,
};


/// A general dummy sensor which simply echoes whatever data is pushed to it.
pub struct DummySensor<T> {
    latest_data: T
}

impl<T> DummySensor<T> {
    pub fn new(starting_data: T) -> DummySensor<T> {
        Self {
            latest_data: starting_data
        }
    }
}

impl<T: Clone> Sensor for DummySensor<T> {
    type Output = T;

    fn sense(&self) -> Self::Output {
        self.latest_data.clone()
    }
}

impl<T> SensorSink for DummySensor<T> {
    type Input = T;

    fn push(&mut self, input: Self::Input) {
        self.latest_data = input
    }
}

/// A sensor which senses all objects' relative positions within a certain fov
pub struct DummyObjectSensor {
    pub map: Arc<Map2D>,
    pub relative_pose: Pose,
    pub robot_pose: Pose,
    fov: f64,
    x_noise_distr: Normal,
    y_noise_distr: Normal,
}

impl DummyObjectSensor {
    pub fn new(
        fov: f64,
        map: Arc<Map2D>,
        relative_pose: Pose,
        robot_pose: Pose,
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

    pub fn update_pose(&mut self, new_pose: Pose) {
        self.robot_pose = new_pose
    }
}

impl Sensor for DummyObjectSensor {
    type Output = Vec<Point>;

    fn relative_pose(&self) -> Pose {
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
}

impl LimitedSensor<f64> for DummyObjectSensor {
    fn range(&self) -> Option<f64> {
        Some(self.fov)
    }
}

#[derive(Clone)]
pub struct DummyDistanceSensor {
    noise_distr: Normal,
    relative_pose: Pose,
    tester: Normal,
    pub map: Arc<Map2D>,
    pub robot_pose: Pose,
    pub max_dist: Option<f64>,
}

impl DummyDistanceSensor {
    /// Noise is Guassian and `noise_margin` is each equal to three standard deviations of the noise distribution.
    /// `relative_pose` is the pose of the sensor relative to the robot
    pub fn new(
        noise_margin: f64,
        relative_pose: Pose,
        map: Arc<Map2D>,
        robot_pose: Pose,
        max_dist: Option<f64>,
    ) -> Self {
        Self {
            tester: Normal::new(0., 0.1),
            noise_distr: Normal::new(0., noise_margin),
            relative_pose,
            map,
            robot_pose,
            max_dist,
        }
    }

    pub fn update_pose(&mut self, new_pose: Pose) {
        self.robot_pose = new_pose
    }
}

impl Sensor for DummyDistanceSensor {
    type Output = Option<f64>;

    fn sense(&self) -> Self::Output {
        let sensor_pose = self.relative_pose + self.robot_pose;
        let ray = self.map.raycast(sensor_pose);
        if let Some(max_dist) = self.max_dist {
            let dist = match ray {
                Some(c) => c.dist(sensor_pose.position),
                None => max_dist,
            };
            if dist > max_dist {
                None
            } else {
                Some(dist + self.noise_distr.sample(&mut thread_rng()))
            }
        } else {
            let dist = match ray {
                Some(c) => c.dist(sensor_pose.position),
                None => 300. + self.tester.sample(&mut thread_rng()),
            };
            Some(dist + self.noise_distr.sample(&mut thread_rng()))
        }
    }

    fn relative_pose(&self) -> Pose {
        self.relative_pose
    }
}

impl LimitedSensor<f64> for DummyDistanceSensor {}

pub struct DummyLidar {
    pub map: Arc<Map2D>,
    pub robot_pose: Pose,
    pub range: Option<Range<f64>>,
    dist_noise: Normal,
    angle_noise: Normal,
    resolution: usize,
    relative_pose: Pose,
}

impl DummyLidar {
    pub fn new(
        map: Arc<Map2D>,
        robot_pose: Pose,
        dist_noise: Normal,
        angle_noise: Normal,
        resolution: usize,
        relative_pose: Pose,
        range: Option<Range<f64>>,
    ) -> Self {
        Self {
            map,
            robot_pose,
            dist_noise,
            angle_noise,
            resolution,
            relative_pose,
            range,
        }
    }

    pub fn update_pose(&mut self, new_pose: Pose) {
        self.robot_pose = new_pose;
    }
}

impl Sensor for DummyLidar {
    type Output = Vec<Point>;

    fn sense(&self) -> Self::Output {
        let mut rng = thread_rng();
        let mut scan = vec![];
        let increment = 2. * PI / self.resolution as f64;
        for i in 0..self.resolution {
            match self.map.raycast(
                self.robot_pose
                    + Pose {
                        angle: increment * i as f64,
                        ..Pose::default()
                    },
            ) {
                Some(scan_point)
                    if self
                        .range
                        .clone()
                        .unwrap_or(0.0..INFINITY)
                        .contains(&scan_point.dist(self.robot_pose.position)) =>
                {
                    scan.push(Point::polar(
                        scan_point.angle_to(self.robot_pose.position) - self.robot_pose.angle
                            + self.angle_noise.sample(&mut rng),
                        scan_point.dist(self.robot_pose.position)
                            + self.dist_noise.sample(&mut rng),
                    ))
                }
                _ => (),
            }
        }
        scan
    }

    fn relative_pose(&self) -> Pose {
        self.relative_pose
    }
}

impl LimitedSensor<Range<f64>> for DummyLidar {
    fn range(&self) -> Option<Range<f64>> {
        self.range.clone()
    }
}

#[derive(Clone)]
pub struct DummyVelocitySensor {
    x_noise_distr: Normal,
    y_noise_distr: Normal,
    angle_noise_distr: Normal,
    real_velocity: Pose,
}

impl DummyVelocitySensor {
    pub fn new(noise_margins: Pose, real_velocity: Pose) -> Self {
        Self {
            x_noise_distr: Normal::new(0., noise_margins.position.x),
            y_noise_distr: Normal::new(0., noise_margins.position.y),
            angle_noise_distr: Normal::new(0., noise_margins.angle),
            real_velocity,
        }
    }

    pub fn update_pose(&mut self, pose: Pose) {
        self.real_velocity = pose;
    }
}

impl Sensor for DummyVelocitySensor {
    type Output = Pose;

    fn sense(&self) -> Self::Output {
        let mut rng = thread_rng();
        Pose {
            angle: self.real_velocity.angle + self.angle_noise_distr.sample(&mut rng),
            position: Point {
                x: self.real_velocity.position.x + self.x_noise_distr.sample(&mut rng),
                y: self.real_velocity.position.y + self.y_noise_distr.sample(&mut rng),
            },
        }
    }
}

pub struct DummyPositionSensor {
    angle_noise_distr: Normal,
    x_noise_distr: Normal,
    y_noise_distr: Normal,
    prev_robot_state: Pose,
    pub robot_pose: Pose,
    last_update: Instant,
}

impl DummyPositionSensor {
    /// Noise is Guassian and `noise_margins` are each equal to three standard deviations of the noise distributions
    pub fn new(robot_pose: Pose, noise_margins: Pose) -> Self {
        Self {
            angle_noise_distr: Normal::new(0., noise_margins.angle / 3.),
            x_noise_distr: Normal::new(0., noise_margins.position.x / 3.),
            y_noise_distr: Normal::new(0., noise_margins.position.y / 3.),
            prev_robot_state: robot_pose,
            robot_pose,
            last_update: Instant::now(),
        }
    }

    pub fn update_pose(&mut self, new_pose: Pose) {
        self.prev_robot_state = self.robot_pose;
        self.robot_pose = new_pose;
        self.last_update = Instant::now();
    }
}

impl Sensor for DummyPositionSensor {
    type Output = Pose;

    fn sense(&self) -> Self::Output {
        let mut rng = thread_rng();
        let elapsed = self.last_update.elapsed().as_secs_f64();
        self.robot_pose - self.prev_robot_state
            + Pose {
                angle: self.angle_noise_distr.sample(&mut rng) * elapsed,
                position: Point {
                    x: self.x_noise_distr.sample(&mut rng),
                    y: self.y_noise_distr.sample(&mut rng),
                } * elapsed,
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
                .as_secs_f64()
            / 1000.
            + Point {
                x: self.left_noise_distr.sample(&mut rng),
                y: self.right_noise_distr.sample(&mut rng),
            }
    }
}
