mod robot;
mod utility;

use rand::prelude::*;
use robot::map::{Map2D, Object2D};
use robot::sensors::dummy::{DummyDistanceSensor, DummyMotionSensor};
use robot::Sensor;
use std::io;
use std::io::prelude::*;
use utility::{Point, Pose};
use vitruvia::{
    graphics_2d,
    graphics_2d::{Color, Transform},
    path::{Builder, Path, Primitive, Stroke},
};

struct Localizer {
    map: Map2D,
    sensor_poses: Vec<Pose>,
    belief: Vec<Pose>,
}

// TODO: this can be multithreaded :D
impl Localizer {
    fn new(particle_count: usize, map: Map2D, sensor_poses: Vec<Pose>) -> Self {
        let mut rng = thread_rng();
        let mut belief = Vec::with_capacity(particle_count);
        for _ in 0..particle_count {
            belief.push(Pose {
                angle: rng.gen_range(0., 2. * std::f64::consts::PI),
                position: Point {
                    x: rng.gen_range(0., map.width),
                    y: rng.gen_range(0., map.height),
                },
            });
        }
        Self {
            map,
            sensor_poses,
            belief,
        }
    }

    fn from_distributions<T, U>(
        x_distr: T,
        y_distr: T,
        angle_distr: T,
        particle_count: usize,
        map: Map2D,
        sensor_poses: Vec<Pose>,
    ) -> Self
    where
        T: rand::distributions::Distribution<U>,
        U: Into<f64>,
    {
        let mut belief = Vec::with_capacity(particle_count);
        for ((x, y), angle) in x_distr
            .sample_iter(&mut thread_rng())
            .zip(y_distr.sample_iter(&mut thread_rng()))
            .zip(angle_distr.sample_iter(&mut thread_rng()))
            .take(particle_count)
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
        let mut highest_error = 0.;
        let mut all_weights_zero = true;
        for sample in &self.belief {
            let mut sum_error = 0.;
            for (i, observation) in z.iter().enumerate() {
                let pred_observation = self.map.raycast(*sample + self.sensor_poses[i]);
                sum_error += match observation {
                    Some(real) => match pred_observation {
                        Some(pred) => {
                            let dist = pred.dist(sample.position);
                            if dist <= MAX_SENSOR_RANGE {
                                (real - dist).powi(2)
                            } else {
                                0.
                            }
                        }
                        Some(_) => 0.,
                        None => 20.,
                    },
                    None => match pred_observation {
                        Some(_) => 20.,
                        None => 0.,
                    },
                };
            }
            if highest_error < sum_error {
                highest_error = sum_error;
            }
            errors.push(sum_error);
        }

        let mut new_particles = Vec::with_capacity(self.belief.len());
        let mut rng = thread_rng();
        let distr = if errors.iter().all(|error| error == &errors[0]) {
            rand::distributions::WeightedIndex::new(errors.iter().map(|_| 1.))
        } else {
            rand::distributions::WeightedIndex::new(
                errors.iter().map(|error| highest_error - error),
            )
        }
        .unwrap();
        for _ in 0..self.belief.len() {
            new_particles.push(self.belief[distr.sample(&mut rng)]);
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
    mcl: Localizer,
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
                Object2D::Line((Point { x: 0., y: 0. }, Point { x: 10., y: 0. })),
                Object2D::Line((Point { x: 10., y: 0. }, Point { x: 10., y: 10. })),
                Object2D::Line((Point { x: 10., y: 10. }, Point { x: 0., y: 10. })),
                Object2D::Line((Point { x: 0., y: 10. }, Point { x: 0., y: 0. })),
            ],
        );
        let starting_robot_pose = Pose {
            angle: 0.,
            position: Point { x: 8., y: 8. },
        };
        Robot {
            mcl: Localizer::new(10_000, map.clone(), vec![Pose::default()]),
            distance_sensors: vec![DummyDistanceSensor {
                map,
                robot_pose: starting_robot_pose,
                max_dist: None,
            }],
            motion_sensor: DummyMotionSensor::new(starting_robot_pose),
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
            .move_to((v1.x * 50., v1.y * 50.))
            .line_to((v2.x * 50., v2.y * 50.));
    }
    let mut map_visual = root.add(builder.done().stroke(Stroke::default()).finalize().into());
    map_visual.apply_transform(Transform::default().with_position((50., 50.)));
    // Make particle visuals
    let mut particle_visuals = Vec::with_capacity(robot.mcl.belief.len());
    for particle in &robot.mcl.belief {
        let path = Primitive::isoceles_triangle(5., 8.)
            .fill(Color::black().into())
            .finalize();
        let mut particle_visual = root.add(path.into());
        particle_visual.apply_transform(
            Transform::default()
                .with_position((
                    particle.position.x * 50. + 50.,
                    particle.position.y * 50. + 50.,
                ))
                .with_rotation(-particle.angle)
                .with_scale((0.5, 0.5)),
        );
        particle_visuals.push(particle_visual);
    }
    // Make position visuals
    let mut predicted_pose_visual = {
        let path = Primitive::isoceles_triangle(10., 16.)
            .fill(Color::rgb(255, 0, 0).into())
            .finalize();
        let mut visual = root.add(path.into());
        let estimation = robot.mcl.get_prediction();
        visual.apply_transform(
            Transform::default()
                .with_position((
                    estimation.position.x * 50. + 50.,
                    estimation.position.y * 50. + 50.,
                ))
                .with_rotation(-estimation.angle),
        );
        visual
    };
    let mut real_pose_visual = {
        let path = Primitive::isoceles_triangle(10., 16.)
            .fill(Color::rgb(0, 0, 255).into())
            .finalize();
        let mut visual = root.add(path.into());
        let pose = robot.motion_sensor.robot_pose;
        visual.apply_transform(
            Transform::default()
                .with_position((pose.position.x * 50. + 50., pose.position.y * 50. + 50.))
                .with_rotation(-pose.angle),
        );
        visual
    };

    // Start up the window
    let mut ctx = gfx.start(root);
    let mut temp = 0.;
    ctx.bind(Box::new(move |ctx| {
        let movement_cmd = Pose::default() + robot.motion_sensor.robot_pose;

        // Move the robot and tick the mcl algorithm
        robot.repeat();
        robot.motion_sensor.update_pose(movement_cmd);
        robot
            .distance_sensors
            .iter_mut()
            .for_each(|s| s.update_pose(movement_cmd));
        let real_pose = robot.motion_sensor.robot_pose;
        let predicted_pose = robot.mcl.get_prediction();
        println!("{:?}, {:?}", predicted_pose, robot.mcl.map.raycast(predicted_pose));
        // println!("Predicted: {:?}", predicted_pose);
        // println!("Real: {:?}", real_pose);

        // Update visuals
        // Update particle visuals
        for i in 0..robot.mcl.belief.len() {
            let particle = robot.mcl.belief[i];
            particle_visuals[i].set_transform(
                Transform::default()
                    .with_position((
                        particle.position.x * 50. + 50.,
                        particle.position.y * 50. + 50.,
                    ))
                    .with_rotation(-particle.angle)
                    .with_scale((0.5, 0.5)),
            );
        }
        // Update position visuals
        predicted_pose_visual.set_transform(
            Transform::default()
                .with_position((
                    predicted_pose.position.x * 50. + 50.,
                    predicted_pose.position.y * 50. + 50.,
                ))
                .with_rotation(-predicted_pose.angle),
        );
        real_pose_visual.set_transform(
            Transform::default()
                .with_position((
                    real_pose.position.x * 50. + 50.,
                    real_pose.position.y * 50. + 50.,
                ))
                .with_rotation(-real_pose.angle),
        );
    }));
    ctx.run();
}

fn parse_movement_command<'a>(cmd: String) -> Result<Pose, &'a str> {
    Ok(if cmd.starts_with('x') {
        Pose {
            angle: 0.,
            position: Point {
                x: cmd
                    .replace('x', "")
                    .parse::<f64>()
                    .map_err(|_| "Bad float")?,
                y: 0.,
            },
        }
    } else if cmd.starts_with('y') {
        Pose {
            angle: 0.,
            position: Point {
                x: 0.,
                y: cmd
                    .replace('y', "")
                    .parse::<f64>()
                    .map_err(|_| "Bad float")?,
            },
        }
    } else if cmd.starts_with('a') {
        Pose {
            angle: cmd
                .replace('a', "")
                .parse::<f64>()
                .map_err(|_| "Bad float")?,
            position: Point::default(),
        }
    } else {
        return Err("Invalid command prefix");
    })
}
