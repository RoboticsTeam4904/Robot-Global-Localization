use crate::robot::map::Map2D;
use crate::robot::Sensor;
use crate::utility::Pose;

pub struct DummyDistanceSensor {
    pub map: Map2D,
    pub robot_pose: Pose,
    pub max_dist: Option<f64>,
}

impl DummyDistanceSensor {
    pub fn update_pose(&mut self, new_pose: Pose) {
        self.robot_pose = new_pose
    }
}

impl Sensor<Option<f64>> for DummyDistanceSensor {
    fn sense(&self) -> Option<f64> {
        let dist = self
            .map
            .raycast(self.robot_pose.clone())?
            .dist(self.robot_pose.position);
        if let Some(max_dist) = self.max_dist {
            if dist > max_dist {
                None
            } else {
                Some(dist)
            }
        } else {
            Some(dist)
        }
    }
}

pub struct DummyMotionSensor {
    pub prev_robot_pose: Pose,
    pub robot_pose: Pose,
}

impl DummyMotionSensor {
    pub fn new(robot_pose: Pose) -> Self {
        Self {
            prev_robot_pose: robot_pose,
            robot_pose,
        }
    }

    pub fn update_pose(&mut self, new_pose: Pose) {
        self.prev_robot_pose = self.robot_pose;
        self.robot_pose = new_pose
    }
}

impl Sensor<Pose> for DummyMotionSensor {
    fn sense(&self) -> Pose {
        self.robot_pose - self.prev_robot_pose
    }
}
