use crate::{
    map::Map2D,
    sensors::{LimitedSensor, Sensor, SensorSink},
    utility::{Point, Point3D, Pose, Pose3D},
};
use rand::{distributions::Distribution, thread_rng};
use rand_distr::Normal;
use std::{
    f64::{consts::PI, INFINITY},
    ops::Range,
    sync::Arc,
    time::{Duration, Instant},
};
// TODO: More thorough documentation
/// A general dummy sensor which simply echoes whatever data is pushed to it.
pub struct DummySensor<T> {
    latest_data: T,
}

impl<T> DummySensor<T> {
    pub fn new(starting_data: T) -> DummySensor<T> {
        Self {
            latest_data: starting_data,
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
    max_dist: Option<f64>,
    x_noise_distr: Normal<f64>,
    y_noise_distr: Normal<f64>,
}

impl DummyObjectSensor {
    pub fn new(
        fov: f64,
        map: Arc<Map2D>,
        relative_pose: Pose,
        robot_pose: Pose,
        max_dist: Option<f64>,
        noise_margins: Point,
    ) -> Self {
        Self {
            fov,
            map,
            relative_pose,
            robot_pose,
            max_dist,
            x_noise_distr: Normal::new(0., noise_margins.x / 3.).unwrap(),
            y_noise_distr: Normal::new(0., noise_margins.y / 3.).unwrap(),
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
            .cull_points(
                sensor_pose,
                Point {
                    x: self.fov,
                    y: 2. * PI,
                },
                self.max_dist,
            )
            .iter()
            .map(|o| {
                Point::from(o.position.clone().without_z())
                    + Point {
                        x: self.x_noise_distr.sample(&mut thread_rng()),
                        y: self.y_noise_distr.sample(&mut thread_rng()),
                    } * o.position.mag().powi(2)
            })
            .collect()
    }
}
impl LimitedSensor<(f64, f64)> for DummyObjectSensor {
    fn range(&self) -> Option<(f64, f64)> {
        Some((self.fov, self.max_dist.unwrap_or(INFINITY)))
    }
}

/// A sensor which senses all objects' relative positions within a certain fov
pub struct DummyObjectSensor3D {
    pub map: Arc<Map2D>,
    pub relative_pose: Pose,
    pub robot_pose: Pose,
    /// x fov is azimuth angle fov, y fov is inclination angle fov
    fov: Point,
    max_dist: Option<f64>,
    x_noise_distr: Normal<f64>,
    y_noise_distr: Normal<f64>,
    z_noise_distr: Normal<f64>,
    azimuth_noise_distr: Normal<f64>,
}

impl DummyObjectSensor3D {
    /// noise margins are inputted as standard deviations
    pub fn new(
        fov: Point,
        map: Arc<Map2D>,
        relative_pose: Pose,
        robot_pose: Pose,
        noise_margins: Pose3D,
        max_dist: Option<f64>,
    ) -> Self {
        Self {
            fov,
            map,
            relative_pose,
            robot_pose,
            x_noise_distr: Normal::new(0., noise_margins.position.x).unwrap(),
            y_noise_distr: Normal::new(0., noise_margins.position.y).unwrap(),
            z_noise_distr: Normal::new(0., noise_margins.position.z).unwrap(),
            azimuth_noise_distr: Normal::new(0., noise_margins.angle.x).unwrap(),
            max_dist,
        }
    }

    pub fn update_pose(&mut self, new_pose: Pose) {
        self.robot_pose = new_pose
    }
}

impl Sensor for DummyObjectSensor3D {
    type Output = Vec<Pose3D>;

    fn relative_pose(&self) -> Pose {
        self.relative_pose
    }

    fn sense(&self) -> Self::Output {
        let sensor_pose = self.robot_pose + self.relative_pose();
        self.map
            .cull_points(sensor_pose, self.fov, self.max_dist)
            .iter()
            .map(|o| {
                *o + Pose3D {
                    angle: Point {
                        x: self.azimuth_noise_distr.sample(&mut thread_rng()),
                        y: 0.,
                    },
                    position: Point3D {
                        x: self.x_noise_distr.sample(&mut thread_rng()),
                        y: self.y_noise_distr.sample(&mut thread_rng()),
                        z: self.z_noise_distr.sample(&mut thread_rng()),
                    } * o.position.mag().powi(2),
                }
            })
            .collect()
    }
}
impl LimitedSensor<(Point, f64)> for DummyObjectSensor3D {
    fn range(&self) -> Option<(Point, f64)> {
        Some((self.fov, self.max_dist.unwrap_or(INFINITY)))
    }
}

#[derive(Clone)]
pub struct DummyDistanceSensor {
    noise_distr: Normal<f64>,
    relative_pose: Pose,
    tester: Normal<f64>,
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
            tester: Normal::new(0., 0.1).unwrap(),
            noise_distr: Normal::new(0., noise_margin).unwrap(),
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
    dist_noise: Normal<f64>,
    angle_noise: Normal<f64>,
    resolution: usize,
    period: Duration,
    relative_pose: Pose,
    scan: Vec<Point>,
    scan_timestamp: Instant,
}

impl DummyLidar {
    /// Creates a new dummy lidar
    /// `map` is the map it scans on
    /// `robot_pose` is the starting pose of the robot
    /// `dist_noise` is the noise that's added to the distance of each point scanned
    /// `angle_noise` is the noise that's added to the angle of each point scanned
    /// `resolution` is the number of points per scan
    /// `period` is the number of seconds in between scans
    /// `relative_pose` is the pose of the lidar relative to the robot
    /// `range` this is the range of the lidar, defaults to infinite
    pub fn new(
        map: Arc<Map2D>,
        robot_pose: Pose,
        dist_noise: Normal<f64>,
        angle_noise: Normal<f64>,
        resolution: usize,
        period: Duration,
        relative_pose: Pose,
        range: Option<Range<f64>>,
    ) -> Self {
        Self {
            map,
            robot_pose,
            dist_noise,
            angle_noise,
            resolution,
            period,
            relative_pose,
            range,
            scan: vec![],
            scan_timestamp: Instant::now(),
        }
    }

    pub fn update_pose(&mut self, new_pose: Pose) {
        self.robot_pose = new_pose;
    }
    pub fn update_with_maps(&mut self, maps: Vec<Arc<Map2D>>) {
        let mut rng = thread_rng();
        let mut scan = vec![];
        let increment = 2. * PI / self.resolution as f64;
        let mut all_maps = maps;
        all_maps.push(self.map.clone());
        for i in 0..self.resolution {
            match Map2D::raycast_with_maps(
                self.robot_pose
                    + Pose {
                        angle: increment * i as f64,
                        ..Pose::default()
                    },
                all_maps.clone(),
            ) {
                Some(scan_point)
                    if self
                        .range
                        .clone()
                        .unwrap_or(0.0..INFINITY)
                        .contains(&scan_point.dist(self.robot_pose.position)) =>
                {
                    let lidar_dist = scan_point.dist(self.robot_pose.position);
                    scan.push(Point::polar(
                        scan_point.angle_to(self.robot_pose.position) - self.robot_pose.angle
                            + self.angle_noise.sample(&mut rng),
                        lidar_dist + lidar_dist.powi(2) * self.dist_noise.sample(&mut rng),
                    ))
                }
                _ => (),
            }
        }
        self.scan = scan;
    }
}
impl Sensor for DummyLidar {
    type Output = Vec<Point>;

    fn update(&mut self) {
        let elapsed = self.scan_timestamp.elapsed();
        if elapsed < self.period {
            return;
        } else {
            self.scan_timestamp += self.period;
        }

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
                    let lidar_dist = scan_point.dist(self.robot_pose.position);
                    scan.push(Point::polar(
                        scan_point.angle_to(self.robot_pose.position) - self.robot_pose.angle
                            + self.angle_noise.sample(&mut rng),
                        lidar_dist + lidar_dist.powi(2) * self.dist_noise.sample(&mut rng),
                    ))
                }
                _ => (),
            }
        }
        self.scan = scan;
    }

    fn sense(&self) -> Self::Output {
        self.scan.clone()
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
    noise_margins: Pose,
    real_velocity: Pose,
    delta_t: f64,
}

impl DummyVelocitySensor {
    pub fn set_delta_t(&mut self, delta_t: f64) {
        self.delta_t = delta_t;
    }
}

impl DummyVelocitySensor {
    pub fn new(noise_margins: Pose, real_velocity: Pose) -> Self {
        Self {
            noise_margins,
            real_velocity,
            delta_t: 1. / 20.,
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
        let x_noise_distr = Normal::new(0., self.noise_margins.position.x * self.delta_t).unwrap();
        let y_noise_distr = Normal::new(0., self.noise_margins.position.y * self.delta_t).unwrap();
        let angle_noise_distr = Normal::new(0., self.noise_margins.angle * self.delta_t).unwrap();
        Pose {
            angle: self.real_velocity.angle + angle_noise_distr.sample(&mut rng),
            position: Point {
                x: self.real_velocity.position.x + x_noise_distr.sample(&mut rng),
                y: self.real_velocity.position.y + y_noise_distr.sample(&mut rng),
            },
        }
    }
}

pub struct DummyPositionSensor {
    angle_noise_distr: Normal<f64>,
    x_noise_distr: Normal<f64>,
    y_noise_distr: Normal<f64>,
    prev_robot_state: Pose,
    pub robot_pose: Pose,
    delta_t: f64,
}

impl DummyPositionSensor {
    /// Noise is Guassian and `noise_margins` are each equal to three standard deviations of the noise distributions
    pub fn new(robot_pose: Pose, noise_margins: Pose) -> Self {
        Self {
            angle_noise_distr: Normal::new(0., noise_margins.angle).unwrap(),
            x_noise_distr: Normal::new(0., noise_margins.position.x).unwrap(),
            y_noise_distr: Normal::new(0., noise_margins.position.y).unwrap(),
            prev_robot_state: robot_pose,
            robot_pose,
            delta_t: 0.10,
        }
    }

    pub fn update_pose(&mut self, new_pose: Pose) {
        self.prev_robot_state = self.robot_pose;
        self.robot_pose = new_pose;
    }
    pub fn set_delta_t(&mut self, delta_t: f64) {
        self.delta_t = delta_t;
    }
}

impl Sensor for DummyPositionSensor {
    type Output = Pose;

    fn sense(&self) -> Self::Output {
        let mut rng = thread_rng();
        self.robot_pose - self.prev_robot_state
            + Pose {
                angle: self.angle_noise_distr.sample(&mut rng) * self.delta_t,
                position: Point {
                    x: self.x_noise_distr.sample(&mut rng),
                    y: self.y_noise_distr.sample(&mut rng),
                } * self.delta_t,
            }
    }
}

pub struct DummyMotionSensor {
    delta_t: f64,
    x_noise: f64,
    y_noise: f64,
    prev_robot_vel: Point,
    prev_measurement_timestep: Instant,
    pub robot_vel: Point,
}

impl Sensor for DummyMotionSensor {
    type Output = Point;

    fn update(&mut self) {}

    fn sense(&self) -> Self::Output {
        let mut rng = thread_rng();
        let x_noise_distr = Normal::new(0., self.x_noise * self.delta_t).unwrap();
        let y_noise_distr = Normal::new(0., self.y_noise * self.delta_t).unwrap();
        (self.robot_vel - self.prev_robot_vel)
            / self
                .prev_measurement_timestep
                .duration_since(Instant::now())
                .as_secs_f64()
            / 1000.
            + Point {
                x: x_noise_distr.sample(&mut rng),
                y: y_noise_distr.sample(&mut rng),
            }
    }
}
