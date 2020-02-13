extern crate sysfs_gpio;

use super::Sensor;
use sysfs_gpio::{Direction, Pin};
use std::time::{Instant, Duration};
use std::thread::sleep;

/// General trait for any component that uses gpio
pub trait GPIOComponent {
    /// Perform general cleanup. Should unexport all pins in use.
    fn cleanup(&self) -> Result<()>;
}

pub struct HCSR04Ultrasonic {
    trig_pin: Pin,
    echo_pin: Pin,
}

impl HCSR04Ultrasonic {
    fn new(trig_pin_num: u64, echo_pin_num: u64) -> Self {
        let trig_pin = Pin::new(trig_pin_num);
        let echo_pin = Pin::new(echo_pin_num);
        trig_pin.set_direction(Direction::Out);
        echo_pin.set_direction(Direction::In);
        Self {
            trig_pin,
            echo_pin,
        }
    }
}

impl GPIOComponent for HCSR04Ultrasonic {
    fn cleanup(&self) -> Result<()> {
        self.trig_pin.unexport()?;
        self.echo_pin.unexport()?;
    }
}

impl Sensor<Option<f64>> for HCSR04Ultrasonic {
    /// Returns distance in cm.
    // https://docs.google.com/document/d/1Y-yZnNhMYy7rwhAgyL_pfa39RsB-x2qR4vP8saG73rE/edit#bookmark=id.at3zjj9qch45
    fn sense(&self) -> Result<Option<f64>> {
        self.trig_pin.set_value(1);
        sleep(Duration::new(0, 10000)); // 10 microseconds
        self.trig_pin.set_value(0);
        while self.echo_pin.get_value()? == 0 {} // TODO: use tokio instead and make this whole thing async
        let start_time = Instant::now();
        while self.echo_pin.get_value()? == 1 {}
        let pulse_width = start_time.elapsed();
        if pulse_width.as_millis >= 38 {
            None
        } else {
            Some(pulse_width.as_micros() as f64 / 58.)
        }
    }
}
