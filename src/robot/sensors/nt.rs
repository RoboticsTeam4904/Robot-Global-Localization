use super::Sensor;
use crate::utility::Pose;
use nt::{NetworkTables, Client, EntryData, EntryValue};
use failure;

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
