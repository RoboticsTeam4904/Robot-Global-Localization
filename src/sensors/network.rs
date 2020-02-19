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
            _ => panic!("The ROBORIO has betrayed us. It is time to go dark."),
        }
    }

    fn relative_pose(&self) -> Pose {
        self.relative_pose
    }
}

pub struct MultiNTSensor {
    sensors: Vec<NTSensor>,
}

impl MultiNTSensor {
    pub async fn new<T>(relative_pose: Pose, ip: &str, paths: T) -> Result<Self, failure::Error>
    where
        T: IntoIterator<Item = String>,
    {
        let mut sensors = vec![];
        for path in paths {
            sensors.push(NTSensor::new(relative_pose, ip, path).await?);
        }
        Ok(Self { sensors })
    }
}

impl Sensor for MultiNTSensor {
    type Output = Vec<f64>;

    fn sense(&self) -> Self::Output {
        self.sensors.iter().map(|sensor| sensor.sense()).collect()
    }

    fn relative_pose(&self) -> Pose {
        match self.sensors.first() {
            Some(s) => s.relative_pose(),
            None => Pose::default(),
        }
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
