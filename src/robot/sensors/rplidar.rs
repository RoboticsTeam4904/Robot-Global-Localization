use super::{LimitedSensor, Sensor};
use crate::utility::{Point, Pose};
use rplidar_drv::{RplidarDevice, ScanPoint};
use serialport::prelude::*;
use std::{
    ops::Range,
    time::Duration,
};

const DEFAULT_BUAD_RATE: u32 = 115200;

// TODO: untested

/// An Rplidar wrapper.
/// More info found here: https://www.robotshop.com/en/rplidar-a2m8-360-laser-scanner.html.
pub struct RplidarSensor {
    pub device: RplidarDevice<dyn serialport::SerialPort>,
    pub latest_scan: Vec<ScanPoint>,
    pub relative_pose: Pose,
    pub sense_range: Option<Range<f64>>,
}

impl RplidarSensor {
    pub fn new(
        serial_port: &str,
        relative_pose: Pose,
        baud_rate: Option<u32>,
    ) -> RplidarSensor {
        let mut sensor = Self::with_range(serial_port, relative_pose, None, baud_rate);
        let typical_mode = sensor.device.get_typical_scan_mode().unwrap();
        let sense_range = sensor
            .device
            .get_all_supported_scan_modes()
            .unwrap()
            .iter()
            .find(|mode| mode.id == typical_mode)
            .map(|mode| 0.0..mode.max_distance as f64);
        RplidarSensor {
            device: sensor.device,
            latest_scan: vec![],
            relative_pose,
            sense_range,
        }
    }

    pub fn with_range(
        serial_port: &str,
        relative_pose: Pose,
        sense_range: Option<Range<f64>>,
        baud_rate: Option<u32>,
    ) -> RplidarSensor {
        let s = SerialPortSettings {
            baud_rate: baud_rate.unwrap_or(DEFAULT_BUAD_RATE),
            data_bits: DataBits::Eight,
            flow_control: FlowControl::None,
            parity: Parity::None,
            stop_bits: StopBits::One,
            timeout: Duration::from_millis(1),
        };
        let mut serial_port =
            serialport::open_with_settings(serial_port, &s).expect("failed to open serial port");
        serial_port
            .write_data_terminal_ready(false)
            .expect("failed to clear DTR");
        let mut device = RplidarDevice::with_stream(serial_port);
        device.start_scan().expect("Failed to start scan");
        RplidarSensor {
            device,
            latest_scan: vec![],
            relative_pose,
            sense_range,
        }
    }
}

impl Sensor for RplidarSensor {
    type Output = Vec<Point>;

    fn update(&mut self) {
        self.latest_scan = self.device.grab_scan().unwrap(); // TODO: pass error upward
    }

    fn sense(&self) -> Self::Output {
        self.latest_scan
            .iter()
            .map(|rp_point| {
                Point::polar(
                    rp_point.angle() as f64 * std::f64::consts::PI / 180., // By default it returns milimeters and degrees ith
                    rp_point.distance() as f64 / 1000.,
                )
            })
            .collect()
    }

    fn relative_pose(&self) -> Pose {
        self.relative_pose
    }
}

impl LimitedSensor<Range<f64>> for RplidarSensor {
    fn range(&self) -> Option<Range<f64>> {
        self.sense_range.clone()
    }
}
