use super::{LimitedSensor, Sensor};
use crate::utility::{Point, Pose};
use rplidar_drv::{RplidarDevice, RplidarHostProtocol, ScanPoint};
use rpos_drv::Channel;
use serialport::prelude::*;
use std::time::Duration;

const DEFAULT_BUAD_RATE: u32 = 115200;

// TODO: untested

/// An Rplidar wrapper.
/// More info found here: https://www.robotshop.com/en/rplidar-a2m8-360-laser-scanner.html.
struct RplidarSensor {
    pub device: RplidarDevice<dyn serialport::SerialPort>,
    pub latest_scan: Vec<ScanPoint>,
    pub relative_pose: Pose,
    pub max_range: Option<f64>,
}

impl RplidarSensor {
    pub fn new(serial_port: &str, relative_pose: Pose, baud_rate: Option<u32>) -> Self {
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
        let channel = Channel::<RplidarHostProtocol, dyn serialport::SerialPort>::new(
            RplidarHostProtocol::new(),
            serial_port,
        );
        let mut device = RplidarDevice::new(channel);
        let typical_mode = device.get_typical_scan_mode().unwrap();
        let max_range = device
            .get_all_supported_scan_modes()
            .unwrap()
            .iter()
            .find(|mode| mode.id == typical_mode)
            .map(|mode| mode.max_distance as f64);
        Self {
            device,
            latest_scan: vec![],
            relative_pose,
            max_range,
        }
    }
}

impl Sensor<Vec<Point>> for RplidarSensor {
    fn update(&mut self) {
        self.latest_scan = self.device.grab_scan().unwrap(); // TODO: pass error upward
    }

    fn sense(&self) -> Vec<Point> {
        self.latest_scan
            .iter()
            .map(|rp_point| {
                Point {
                    x: rp_point.angle().cos() as f64,
                    y: rp_point.angle().sin() as f64,
                } * rp_point.distance() as f64
            })
            .collect()
    }

    fn relative_pose(&self) -> Pose {
        self.relative_pose
    }
}

impl LimitedSensor<f64, Vec<Point>> for RplidarSensor {
    fn range(&self) -> Option<f64> {
        self.max_range
    }
}
