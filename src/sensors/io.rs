use super::{Sensor, SensorSink};
use std::io::{Read, Write};

/// An IO Sensor is a simple wrapper of something which implements
/// `std::io::Read` and/or `std::io::Write`.
///
/// If the wrapped source implements `std::io::Read`, then `IOSensor` implements `Sensor<Output = Vec<u8>>`.
/// The output is attained by calling `read_to_end`.
///
/// If the wrapped source implements `std::io::Write`, then `IOSensor` implements `SensorSink<Input = Vec<u8>>`.
/// The input is pushed by calling `write_all`.
///
/// NOTE: pushed data to `IOSensor` as a `SensorSink` will not be immediately read by
/// `sense`, as `sense` uses `Read::read_to_end` which starts from the cursor position and
/// `push` uses `Write::write_all` which moves the cursor position to the end of the
/// written data.
///
/// It is recomended to use this sensor in conjuction with the `map` and `override_limit`
/// methods to build a sensor which fully meets your needs.
///
/// Although an `IOSensor` can technically wrap any object, even if it does not implement read or write,
/// it is not recommended as it will not accomplish anything.
pub struct IOSensor<IO> {
    io: IO,
    latest_data: Vec<u8>,
}

impl<IO> IOSensor<IO> {
    pub fn new(io: IO) -> Self {
        Self {
            io,
            latest_data: vec![],
        }
    }

    pub fn with_starting_data(io: IO, starting_data: Vec<u8>) -> Self {
        Self {
            io,
            latest_data: starting_data,
        }
    }
}

impl<I> Sensor for IOSensor<I>
where
    I: Read,
{
    type Output = Vec<u8>;

    fn update(&mut self) {
        let mut buf = vec![];
        self.io.read_to_end(&mut buf).unwrap();
        self.latest_data = buf;
    }

    fn sense(&self) -> Self::Output {
        self.latest_data.clone()
    }
}

impl<O> SensorSink for IOSensor<O>
where
    O: Write,
{
    type Input = Vec<u8>;

    fn push(&mut self, input: Self::Input) {
        self.io.write_all(&input).unwrap();
        self.io.flush().unwrap();
    }
}

#[cfg(feature = "asyncio")]
pub use asyncio::*;
#[cfg(feature = "asyncio")]
pub mod asyncio {
    use super::{Sensor, SensorSink};
    use core_futures_io::{AsyncRead, AsyncWrite};
    use futures::{
        channel::mpsc::{unbounded, UnboundedSender},
        executor::ThreadPool,
        stream::StreamExt,
    };
    use mincodec::{AsyncReader, AsyncWriter, Deserialize, MinCodecRead, MinCodecWrite, Serialize};
    use std::{
        fmt::Debug,
        sync::{Arc, Mutex},
    };

    // TODO: Could split this up, provide more convenience,
    // etc. but really, I don't ever want to write down
    // as many trait bounds as are below
    // so...
    pub struct AsyncIOSensor<T, U>
    where
        T: MinCodecRead + Clone + Send,
        U: MinCodecWrite + Send,
    {
        latest_recieved_data: Arc<Mutex<T>>,
        sender: UnboundedSender<U>,
    }

    impl<T, U> AsyncIOSensor<T, U>
    where
        T: MinCodecRead + Clone + Send,
        U: MinCodecWrite + Send,
    {
        /// Creates an `AsyncIOSensor` that reads from
        /// `i` and writes to `o`.
        /// It initially writes `starting_sink_data` to `o`
        /// and reads `starting_sensor_data` from `i`.
        /// 
        /// To get `i` and `o` from a single source, use
        /// `.split()` on that source to attain them.
        pub fn with_starting_data<I, O>(
            i: I,
            o: O,
            starting_sensor_data: T,
            starting_sink_data: U,
        ) -> Self
        where
            I: AsyncRead + Send, // Bounds got me like
            O: AsyncWrite + Send,
            I: 'static + Unpin + Send,
            <I as AsyncRead>::Error: Debug,
            T: 'static + Send,
            T::Deserialize: Unpin + Send,
            <T::Deserialize as Deserialize>::Error: Debug,
            U: 'static + Send,
            U::Serialize: Unpin + Send,
            <U::Serialize as Serialize>::Error: Debug,
            O: 'static + Unpin + Send,
            <O as AsyncWrite>::WriteError: Debug,
        {
            let latest_recieved_data = Arc::new(Mutex::new(starting_sensor_data));
            let (sender, mut recver) = unbounded();

            let data_buf = latest_recieved_data.clone();
            let pool = ThreadPool::new().unwrap();
            pool.spawn_ok(async move {
                let mut reader = AsyncReader::<_, T>::new(i);
                loop {
                    *data_buf.lock().unwrap() =
                        (&mut reader).await.expect("Could not read async reader");
                    reader.reset().expect("Could not reset async reader");
                }
            });
            pool.spawn_ok(async move {
                let mut writer = AsyncWriter::new(o, starting_sink_data);
                (&mut writer)
                    .await
                    .expect("Could not write to async writer");
                while let Some(data) = recver.next().await {
                    writer.reset(data).expect("Could not reset async writer");
                    (&mut writer)
                        .await
                        .expect("Could not write to async writer");
                }
            });
            Self {
                latest_recieved_data,
                sender,
            }
        }
    }

    impl<T, U> Sensor for AsyncIOSensor<T, U>
    where
        T: MinCodecRead + Clone + Send,
        U: MinCodecWrite + Send,
    {
        type Output = T;

        fn sense(&self) -> Self::Output {
            self.latest_recieved_data.lock().unwrap().clone()
        }
    }

    impl<T, U> SensorSink for AsyncIOSensor<T, U>
    where
        T: MinCodecRead + Clone + Send,
        U: MinCodecWrite + Send + Debug,
    {
        type Input = U;

        fn push(&mut self, input: Self::Input) {
            self.sender.unbounded_send(input).unwrap();
        }
    }
}
