use super::Sensor;
use crate::{networktables, utility::Pose};
use failure;
use nt::{Client, EntryValue, NetworkTables};
use std::sync::{Arc, Mutex};

pub struct NTSensor {
    inst: Arc<Mutex<NetworkTables<Client>>>,
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
            inst: Arc::new(Mutex::new(inst)),
            entry_id,
            relative_pose,
        })
    }

    pub async fn from_inst(
        relative_pose: Pose,
        inst: Arc<Mutex<NetworkTables<Client>>>,
        path: String,
    ) -> Self {
        let entry_id =
            *networktables::get_entry(&*inst.lock().unwrap(), path, EntryValue::Double(0.))
                .await
                .id();
        Self {
            inst,
            entry_id,
            relative_pose,
        }
    }
}

impl Sensor for NTSensor {
    type Output = f64;

    fn sense(&self) -> Self::Output {
        match self
            .inst
            .lock()
            .unwrap()
            .get_entry(self.entry_id)
            .value()
            .value
        {
            EntryValue::Double(val) => val,
            _ => panic!("The ROBORIO has betrayed us. It is time to go dark."),
        }
    }

    fn relative_pose(&self) -> Pose {
        self.relative_pose
    }
}

pub struct MultiNTSensor {
    inst: Arc<Mutex<NetworkTables<Client>>>,
    relative_pose: Pose,
    entry_ids: Vec<u16>,
}

impl MultiNTSensor {
    pub async fn new<T>(relative_pose: Pose, ip: &str, paths: T) -> Result<Self, failure::Error>
    where
        T: IntoIterator<Item = String>,
    {
        let inst = NetworkTables::connect(ip, "sensor").await?;
        let mut entry_ids = vec![];
        for path in paths {
            entry_ids.push(
                *networktables::get_entry(&inst, path, EntryValue::Double(0.))
                    .await
                    .id(),
            );
        }
        Ok(Self {
            inst: Arc::new(Mutex::new(inst)),
            entry_ids,
            relative_pose,
        })
    }

    pub async fn from_inst<T>(
        relative_pose: Pose,
        inst: Arc<Mutex<NetworkTables<Client>>>,
        paths: T,
    ) -> Self
    where
        T: IntoIterator<Item = String>,
    {
        let mut entry_ids = vec![];
        {
            let inst_ = inst.lock().unwrap();
            for path in paths {
                entry_ids.push(
                    *networktables::get_entry(&*inst_, path, EntryValue::Double(0.))
                        .await
                        .id(),
                );
            }
        }
        Self {
            inst,
            entry_ids,
            relative_pose,
        }
    }
}

impl Sensor for MultiNTSensor {
    type Output = Vec<f64>;

    fn sense(&self) -> Self::Output {
        let inst = self.inst.lock().unwrap();
        self.entry_ids
            .iter()
            .map(|&id| match inst.get_entry(id).value().value {
                EntryValue::Double(val) => val,
                _ => panic!("The ROBORIO has betrayed us. It is time to go dark."),
            })
            .collect()
    }

    fn relative_pose(&self) -> Pose {
        self.relative_pose
    }
}

pub struct PoseNTSensor {
    internal_sensor: MultiNTSensor,
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
            internal_sensor: MultiNTSensor::new(
                relative_pose,
                ip,
                vec![angle_path, x_path, y_path],
            )
            .await?,
        })
    }

    pub async fn from_inst(
        relative_pose: Pose,
        inst: Arc<Mutex<NetworkTables<Client>>>,
        x_path: String,
        y_path: String,
        angle_path: String,
    ) -> Self {
        Self {
            internal_sensor: MultiNTSensor::from_inst(
                relative_pose,
                inst,
                vec![angle_path, x_path, y_path],
            )
            .await,
        }
    }
}

impl Sensor for PoseNTSensor {
    type Output = Pose;

    fn sense(&self) -> Self::Output {
        let p = self.internal_sensor.sense();
        Pose {
            angle: p[0],
            position: (p[1], p[2]).into(),
        }
    }

    fn relative_pose(&self) -> Pose {
        self.internal_sensor.relative_pose
    }
}
