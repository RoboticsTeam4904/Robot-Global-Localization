use super::Sensor;
use crate::utility::{Point, Pose};
use failure;
use nt::{Client, EntryData, EntryValue, NetworkTables};

pub struct NTSensor {
    inst: NetworkTables<Client>,
    entry_id: u16,
    relative_pose: Pose,
}

impl NTSensor {
    fn new(relative_pose: Pose, ip: &str, path: String) -> Result<Self, failure::Error> {
        let inst = NetworkTables::connect(ip, "sensor")?;
        let entry_id = inst.create_entry(EntryData::new(path, 0, EntryValue::Double(0.)));
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
    fn new(
        relative_pose: Pose,
        ip: &str,
        x_path: String,
        y_path: String,
        angle_path: String,
    ) -> Result<Self, failure::Error> {
        Ok(Self {
            x: NTSensor::new(relative_pose, ip, x_path)?,
            y: NTSensor::new(relative_pose, ip, y_path)?,
            angle: NTSensor::new(relative_pose, ip, angle_path)?,
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
