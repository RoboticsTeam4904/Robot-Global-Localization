use super::Sensor;
use crate::{
    networktables,
    utility::{Point, Pose},
};
use failure;
use nt::{Client, EntryValue, NetworkTables};

pub struct NTSensor {
    inst: NetworkTables<Client>,
    entry_id: u16,
    relative_pose: Pose,
}

impl NTSensor {
    pub async fn new(relative_pose: Pose, ip: &str, path: String) -> Result<Self, failure::Error> {
        let inst = NetworkTables::connect(ip, "sensor").await?;
        let entry_id = *networktables::get_entry(&inst, path, EntryValue::Double(0.))
            .await
            .id();
        Ok(Self {
            inst,
            entry_id,
            relative_pose,
        })
    }
}

impl Sensor for NTSensor {
    type Output = f64;

    fn sense(&self) -> Self::Output {
        match self.inst.get_entry(self.entry_id).value().value {
            EntryValue::Double(val) => val,
            _ => unreachable!(),
        }
    }

    fn relative_pose(&self) -> Pose {
        self.relative_pose
    }
}

pub struct PoseNTSensor {
    x: NTSensor,
    y: NTSensor,
    angle: NTSensor,
}

impl PoseNTSensor {
    pub async fn new(
        relative_pose: Pose,
        ip: &str,
        x_path: String,
        y_path: String,
        angle_path: String,
    ) -> Result<Self, failure::Error> {
        Ok(Self {
            x: NTSensor::new(relative_pose, ip, x_path).await?,
            y: NTSensor::new(relative_pose, ip, y_path).await?,
            angle: NTSensor::new(relative_pose, ip, angle_path).await?,
        })
    }
}

impl Sensor for PoseNTSensor {
    type Output = Pose;

    fn sense(&self) -> Self::Output {
        Pose {
            position: Point {
                x: self.x.sense(),
                y: self.y.sense(),
            },
            angle: self.angle.sense(),
        }
    }

    fn relative_pose(&self) -> Pose {
        self.x.relative_pose
    }
}
