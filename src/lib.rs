pub mod ai;
pub mod map;
#[cfg(feature = "network")]
pub mod networktables;
pub mod replay;
pub mod sensors;
pub mod utility;

#[cfg(test)]
mod tests {
    use super::sensors::{io::IOSensor, *};
    use std::fs::{remove_file, copy, OpenOptions};

    #[test]
    fn test_file_sensor_sink() {
        const PATH: &'static str = "test_resources/hello_mut.txt";
        const RESET_PATH: &'static str = "test_resources/hello.txt";
        remove_file(PATH).unwrap();
        copy(RESET_PATH, PATH).unwrap();
        {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .open(PATH)
                .unwrap();
            let mut sensor_sink = IOSensor::new(file)
                .map(|data: Vec<u8>| String::from_utf8(data).unwrap())
                .map_sink(|data: String| data.as_bytes().to_vec());
            sensor_sink.update();
            sensor_sink.update_sink();
            assert_eq!("hello".to_owned(), sensor_sink.sense());
            sensor_sink.push("hi".to_owned());
        }

        {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .open(PATH)
                .unwrap();
            let mut sensor_sink = IOSensor::new(file)
                .map(|data: Vec<u8>| String::from_utf8(data).unwrap())
                .map_sink(|data: String| data.as_bytes().to_vec());
            sensor_sink.update();
            sensor_sink.update_sink();
            assert_eq!("hellohi".to_owned(), sensor_sink.sense());
        }
        remove_file(PATH).unwrap();
        copy(RESET_PATH, PATH).unwrap();
    }
}
