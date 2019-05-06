#![allow(dead_code)] // TODO: This will be removed after organizational overhall and a lib.rs
mod robot;
mod utility;

use rand::distributions::WeightedIndex;
use rand::prelude::*;
use robot::map::{Map2D, Object2D};
use robot::sensors::dummy::{DummyDistanceSensor, DummyMotionSensor};
use robot::Sensor;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_8, PI};
use utility::{Point, Pose};
use vitruvia::{
    graphics_2d,
    graphics_2d::{Color, Transform, Vector},
    interaction::keyboard::{Arrow, Key},
    path::{Builder, Stroke},
    text::Text,
};

struct LHBLocalizer {
    max_particle_count: usize,
    weight_sum_threshold: f64,
    map: Map2D,
    sensor_poses: Vec<Pose>,
    belief: Vec<Pose>,
}

// TODO: this can be multithreaded :D (and by this I mean everything basically)
impl LHBLocalizer {
    fn new(max_particle_count: usize, map: Map2D, sensor_poses: Vec<Pose>) -> Self {
        let mut rng = thread_rng();
        let mut belief = Vec::with_capacity(max_particle_count);
        for _ in 0..max_particle_count {
            belief.push(Pose {
                angle: rng.gen_range(0., 2. * PI),
                position: Point {
                    x: rng.gen_range(0., map.width),
                    y: rng.gen_range(0., map.height),
                },
            });
        }
        LHBLocalizer {
            max_particle_count: max_particle_count,
            weight_sum_threshold: max_particle_count as f64 / 50., // TODO: fixed parameter
            map,
            sensor_poses,
            belief,
        }
    }

    fn from_distributions<T, U>(
        x_distr: T,
        y_distr: T,
        angle_distr: T,
        max_particle_count: usize,
        map: Map2D,
        sensor_poses: Vec<Pose>,
    ) -> Self
    where
        T: Distribution<U>,
        U: Into<f64>,
    {
        let mut belief = Vec::with_capacity(max_particle_count);
        for ((x, y), angle) in x_distr
            .sample_iter(&mut thread_rng())
            .zip(y_distr.sample_iter(&mut thread_rng()))
            .zip(angle_distr.sample_iter(&mut thread_rng()))
            .take(max_particle_count)
        {
            belief.push(Pose {
                angle: angle.into(),
                position: Point {
                    x: x.into(),
                    y: y.into(),
                },
            });
        }
        Self {
            max_particle_count,
            weight_sum_threshold: max_particle_count as f64 / 50., // TODO: fixed parameter
            map,
            sensor_poses,
            belief,
        }
    }

    fn control_update(&mut self, u: Pose) {
        for i in 0..self.belief.len() {
            self.belief[i] += u;
        }
    }

    /// Takes in a vector of ranges indexed synchronously with `self.sensor_poses`
    fn observation_update(&mut self, z: Vec<Option<f64>>) {
        // TODO: These weights need a lot more fixing
        // TODO: Don't just use a constant, c'mon
        const MAX_SENSOR_RANGE: f64 = 15.;
        let mut errors: Vec<f64> = Vec::with_capacity(self.belief.len());
        for sample in &self.belief {
            let mut sum_error = 0.;
            for (i, observation) in z.iter().enumerate() {
                let pred_observation = self.map.raycast(*sample + self.sensor_poses[i]);
                sum_error += match observation {
                    Some(real_dist) => match pred_observation {
                        Some(pred) => {
                            let pred_dist = pred.dist(sample.position);
                            if pred_dist <= MAX_SENSOR_RANGE {
                                (real_dist - pred_dist).abs() //powi(2) // TODO: fixed parameter
                            } else {
                                0.
                            }
                        }
                        None => 20., // TODO: fixed parameter
                    },
                    None => match pred_observation {
                        Some(_) => 20., // TODO: fixed parameter
                        None => 0.,
                    },
                };
            }
            errors.push(sum_error);
        }

        let mut new_particles = Vec::new();
        let mut rng = thread_rng();
        #[allow(clippy::float_cmp)]
        let weights: Vec<f64> = if errors.iter().all(|error| error == &0.) {
            errors
                .iter()
                .map(|_| 5. * self.weight_sum_threshold / self.belief.len() as f64) // TODO: fixed parameter
                .collect()
        } else {
            errors
                .iter()
                .map(|error| 2f64.powf(-error)) // TODO: fixed parameter
                .collect()
        };
        let distr = WeightedIndex::new(weights.clone()).unwrap();
        let mut sum_weights = 0.;
        // TODO: rather than have max particle count and weight sum threshold parameters,
        // it might be beneficial to use some dynamic combination of the two as the break condition.
        while sum_weights < self.weight_sum_threshold
            && new_particles.len() < self.max_particle_count
        {
            let idx = distr.sample(&mut rng);
            sum_weights += weights[idx];
            new_particles.push(
                self.belief[idx]
                    + Pose::random(
                        (-FRAC_PI_8 / 8.)..(FRAC_PI_8 / 8.),
                        -0.03..0.03,
                        -0.03..0.03,
                    ), // TODO: fixed parameter
            );
        }
        self.belief = new_particles;
    }

    fn get_prediction(&self) -> Pose {
        let mut average_pose = Pose::default();
        for sample in &self.belief {
            average_pose += *sample;
        }
        average_pose / (self.belief.len() as f64)
    }
}

struct Robot {
    mcl: LHBLocalizer,
    motion_sensor: DummyMotionSensor,
    distance_sensors: Vec<DummyDistanceSensor>,
}

impl Robot {
    fn repeat(&mut self) {
        self.mcl.control_update(self.motion_sensor.sense()); // TODO: dummy motion sensor and its usage are currently oversimplified
        self.mcl.observation_update(
            self.distance_sensors
                .iter()
                .map(|sensor| sensor.sense())
                .collect(),
        );
    }
}

fn main() {
    // Make a robot
    let mut robot = {
        let map = Map2D::new(
            10.,
            10.,
            vec![
                Object2D::Rectangle((Point::default(), Point { x: 10., y: 10. })),
                Object2D::Rectangle((Point { x: 10., y: 10. }, Point { x: 9., y: 7. })),
                Object2D::Rectangle((Point { x: 2.5, y: 2.5 }, Point { x: 7.5, y: 3. })),
                Object2D::Triangle((
                    Point { x: 1., y: 8. },
                    Point { x: 3., y: 8. },
                    Point { x: 2., y: 7. },
                )),
                Object2D::Line((Point { x: 5., y: 5. }, Point { x: 5., y: 10. })),
            ],
        );
        let starting_robot_pose = Pose {
            angle: 0.,
            position: Point { x: 8., y: 8. },
        };
        let distance_sensor_noise = 0.5;
        let distance_sensors = vec![
            DummyDistanceSensor::new(
                distance_sensor_noise,
                Pose::default(),
                map.clone(),
                starting_robot_pose,
                None,
            ),
            DummyDistanceSensor::new(
                distance_sensor_noise,
                Pose::default()
                    + Pose {
                        angle: FRAC_PI_2,
                        position: Point::default(),
                    },
                map.clone(),
                starting_robot_pose,
                None,
            ),
            DummyDistanceSensor::new(
                distance_sensor_noise,
                Pose::default()
                    + Pose {
                        angle: PI,
                        position: Point::default(),
                    },
                map.clone(),
                starting_robot_pose,
                None,
            ),
            DummyDistanceSensor::new(
                distance_sensor_noise,
                Pose::default()
                    + Pose {
                        angle: PI + FRAC_PI_2,
                        position: Point::default(),
                    },
                map.clone(),
                starting_robot_pose,
                None,
            ),
        ];
        Robot {
            mcl: LHBLocalizer::new(
                20_000,
                map.clone(),
                distance_sensors
                    .iter()
                    .map(|sensor| sensor.get_relative_pose())
                    .collect(),
            ),
            distance_sensors,
            motion_sensor: DummyMotionSensor::new(
                starting_robot_pose,
                Pose {
                    angle: FRAC_PI_8,
                    position: Point { x: 0.1, y: 0.1 },
                },
            ),
        }
    };

    // Setup visuals
    let gfx = graphics_2d::new();
    let mut root = gfx.frame();
    let mut builder = Builder::new();
    // Make map visual
    for line in &robot.mcl.map.lines {
        let v1 = robot.mcl.map.get_vertex(line.0);
        let v2 = robot.mcl.map.get_vertex(line.1);
        builder = builder
            .move_to(v1 * 50.)
            .line_to(v2 * 50.);
    }
    let mut map_visual = root.add(builder.done().stroke(Stroke::default()).finalize().into());
    map_visual.apply_transform(Transform::default().with_position((50., 50.)));
    // Make tick, time, and particle counters' visuals
    let mut ticks = 0;
    let mut tick_visual = root.add(Text::new("0t").with_size(30.).into());
    tick_visual.set_transform(Transform::default().with_position((500., 20.)));
    let start_time = std::time::Instant::now();
    let mut time_visual = root.add(
        Text::new(format!("{:?}", start_time.elapsed()).as_str())
            .with_size(30.)
            .into(),
    );
    time_visual.set_transform(Transform::default().with_position((50., 20.)));
    let mut particle_count_visual = root.add(
        Text::new(format!("{}p", robot.mcl.belief.len()).as_str())
            .with_size(30.)
            .into(),
    );
    particle_count_visual.set_transform(Transform::default().with_position((50., 570.)));
    // Make particle visuals
    let mut particle_visuals = Vec::with_capacity(robot.mcl.belief.len());
    let path: vitruvia::graphics_2d::Content = isoceles_triangle(5., 8.)
        .fill(Color::black().into())
        .finalize()
        .into();
    for particle in &robot.mcl.belief {
        let mut particle_visual = root.add(path.clone());
        particle_visual.apply_transform(
            Transform::default()
                .with_position(particle.position * 50. + 50.)
                .with_rotation(-particle.angle)
                .with_scale(0.5),
        );
        particle_visuals.push(particle_visual);
    }
    // Make position visuals
    let mut predicted_pose_visual = {
        let path = isoceles_triangle(10., 16.)
            .fill(Color::rgb(255, 0, 0).into())
            .finalize();
        let mut visual = root.add(path.into());
        let estimation = robot.mcl.get_prediction();
        visual.apply_transform(
            Transform::default()
                .with_position(estimation.position * 50. + 50.)
                .with_rotation(-estimation.angle),
        );
        visual
    };
    let mut real_pose_visual = {
        let path = isoceles_triangle(10., 16.)
            .fill(Color::rgb(0, 0, 255).into())
            .finalize();
        let mut visual = root.add(path.into());
        let pose = robot.motion_sensor.robot_pose;
        visual.apply_transform(
            Transform::default()
                .with_position(pose.position * 50. + 50.)
                .with_rotation(-pose.angle),
        );
        visual
    };

    // Start up the window
    let mut root2 = root.clone();
    let mut ctx = gfx.start(root);
    ctx.bind(Box::new(move |_| {
        ticks += 1;
        let movement_cmd = robot.motion_sensor.robot_pose;
        // if ctx.keyboard().poll(Key::Arrow(Arrow::Up)) {

        // }
        // if ctx.keyboard().poll(Key::Arrow(Arrow::Down)) {

        // }

        // Move the robot and tick the mcl algorithm
        robot.repeat();
        robot.motion_sensor.update_pose(movement_cmd);
        robot
            .distance_sensors
            .iter_mut()
            .for_each(|s| s.update_pose(movement_cmd));
        let real_pose = robot.motion_sensor.robot_pose;
        let predicted_pose = robot.mcl.get_prediction();
        // Update visuals
        // Update particle visuals
        if robot.mcl.belief.len() < particle_visuals.len() {
            for i in robot.mcl.belief.len()..particle_visuals.len() {
                particle_visuals[i].set_transform(Transform::default().with_scale(0.00001));
            }
        } else if robot.mcl.belief.len() != particle_visuals.len() {
            let path: vitruvia::graphics_2d::Content = isoceles_triangle(5., 8.)
                .fill(Color::black().into())
                .finalize()
                .into();
            for _ in particle_visuals.len()..robot.mcl.belief.len() {
                particle_visuals.push(root2.add(path.clone()));
            }
        }
        for i in 0..robot.mcl.belief.len() {
            let particle = robot.mcl.belief[i];
            particle_visuals[i].set_transform(
                Transform::default()
                    .with_position(particle.position * 50. + 50.)
                    .with_rotation(-particle.angle)
                    .with_scale(0.5),
            );
        }
        // Update tick, time, particle counters' visuals
        tick_visual.update(
            Text::new(format!("{}t", ticks).as_str())
                .with_size(30.)
                .into(),
        );
        time_visual.update(
            Text::new(format!("{:?}", start_time.elapsed()).as_str())
                .with_size(30.)
                .into(),
        );
        particle_count_visual.update(
            Text::new(format!("{}p", robot.mcl.belief.len()).as_str())
                .with_size(30.)
                .into(),
        );
        // Update position visuals
        predicted_pose_visual.set_transform(
            Transform::default()
                .with_position(predicted_pose.position * 50. + 50.)
                .with_rotation(-predicted_pose.angle),
        );
        real_pose_visual.set_transform(
            Transform::default()
                .with_position(real_pose.position * 50. + 50.)
                .with_rotation(-real_pose.angle),
        );
    }));
    ctx.run();
}

/// Creates an isoceles triangle
pub fn isoceles_triangle(base: f64, height: f64) -> vitruvia::path::StyleHelper {
    Builder::new()
        .move_to(Vector::default())
        .line_to(Vector { x: 0., y: base })
        .line_to(Vector {
            x: height,
            y: base / 2.,
        })
        .done()
}

impl Into<Vector> for Point {
    fn into(self) -> Vector {
        Vector { x: self.x, y: self.y }
    }
}
