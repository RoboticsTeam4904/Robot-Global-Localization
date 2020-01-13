use super::{LimitedSensor, Sensor};
use crate::utility::{Point, Pose};
use rplidar_drv::{RplidarDevice, RplidarHostProtocol, ScanPoint};
use rpos_drv::Channel;
use serialport::prelude::*;
use std::{ops::{RangeBounds, Bound, Range}, time::Duration};

const DEFAULT_BUAD_RATE: u32 = 115200;

// TODO: untested

/// An Rplidar wrapper.
/// More info found here: https://www.robotshop.com/en/rplidar-a2m8-360-laser-scanner.html.
struct RplidarSensor<T: RangeBounds<f64> + Clone> {
    pub device: RplidarDevice<dyn serialport::SerialPort>,
    pub latest_scan: Vec<ScanPoint>,
    pub relative_pose: Pose,
    pub sense_range: Option<T>,
}

impl<T: RangeBounds<f64> + Clone> RplidarSensor<T> {
    pub fn new(serial_port: &str, relative_pose: Pose, baud_rate: Option<u32>) -> RplidarSensor<Range<f64>> {
        let mut sensor = Self::with_range(serial_port, relative_pose, None, baud_rate);
        let typical_mode = sensor.device.get_typical_scan_mode().unwrap();
        let sense_range = sensor.device
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

    pub fn with_range(serial_port: &str, relative_pose: Pose, sense_range: Option<T>, baud_rate: Option<u32>) -> RplidarSensor<T> {
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
        let device = RplidarDevice::new(channel);
        RplidarSensor {
            device,
            latest_scan: vec![],
            relative_pose,
            sense_range,
        }
    }
}

impl<T: RangeBounds<f64> + Clone> Sensor for RplidarSensor<T> {
    type Output = Vec<Point>;

    fn update(&mut self) {
        self.latest_scan = self.device.grab_scan().unwrap(); // TODO: pass error upward
    }

    fn sense(&self) -> Self::Output {
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

impl<T: RangeBounds<f64> + Clone> LimitedSensor<T> for RplidarSensor<T> {
    fn range(&self) -> Option<T> {
        self.sense_range.clone()
    }
}
