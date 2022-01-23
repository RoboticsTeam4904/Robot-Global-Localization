use crate::{
    robot::sensors::{LimitedSensor, Sensor},
    utility::{Point, Pose},
};
use rplidar_drv::RplidarDevice;
use serialport::prelude::*;
use std::{
    ops::Range,
    sync::{Arc, Mutex},
    time::Duration,
};

const DEFAULT_BUAD_RATE: u32 = 115200;
const DEFAULT_RANGE: Option<Range<f64>> = Some(150.0..8000.);

/// An Rplidar wrapper.
/// More info found here: https://www.robotshop.com/en/rplidar-a2m8-360-laser-scanner.html.
pub struct RplidarSensor {
    pub device: Arc<Mutex<RplidarDevice<Box<dyn serialport::SerialPort>>>>,
    pub latest_scan: Vec<Point>,
    pub relative_pose: Pose,
    pub sense_range: Option<Range<f64>>,
}

unsafe impl Send for RplidarSensor {}
unsafe impl Sync for RplidarSensor {}

impl RplidarSensor {
    pub fn new(serial_port: &str, relative_pose: Pose, baud_rate: Option<u32>) -> RplidarSensor {
        Self::with_range(serial_port, relative_pose, DEFAULT_RANGE, baud_rate)
        // TODO: automatically grab range from lidar instead of using DEFAULT_RANGE (sdk's implementation of getting range from firmware doesn't always work)
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
            device: Arc::new(Mutex::new(device)),
            latest_scan: vec![],
            relative_pose,
            sense_range,
        }
    }
}

impl Sensor for RplidarSensor {
    type Output = Vec<Point>;

    fn update(&mut self) {
        match self.device.lock().unwrap().grab_scan() {
            Ok(scan) => {
                self.latest_scan = scan
                    .iter()
                    .map(|rp_point| {
                        Point::polar(rp_point.raw_angle() as f64, rp_point.raw_distance() as f64)
                    })
                    .collect()
            }
            Err(e) => println!("{:?}", e),
        };
    }

    fn sense(&self) -> Self::Output {
        self.latest_scan.clone()
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
