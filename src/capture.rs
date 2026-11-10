//! A camera's capture thread: opens the camera, sets its format, frame rate, exposure, gain and
//! controls, and streams frames into the camera's [`Latest`] slot, each with the exposure and gain
//! in effect. On any error it closes the camera and tries again every 300 ms, so an unplugged
//! camera comes back by itself, its settings set again.
//!
//! The camera is a [`Source`]: a V4L2 device ([`V4l2`]), memory-mapped, or a test's.

use std::io;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use log::{info, warn};
use v4l::buffer::{Flags, Metadata, Type};
use v4l::fraction::Fraction;
use v4l::io::mmap::Stream;
use v4l::io::traits::CaptureStream;
use v4l::video::Capture;
use v4l::video::capture::Parameters;
use v4l::{Device, Format, FourCC};

use crate::clock::Clocks;
use crate::config::{Camera, PixelFormat};
use crate::latest::{Captured, Latest};
use crate::settings::{self, Control, Controls, InEffect, Said};

/// How long a camera that failed waits before it's opened again.
pub const RETRY: Duration = Duration::from_millis(300);
/// No frame for this long counts as a lost camera.
const STALL: Duration = Duration::from_secs(1);
const BUFFERS: u32 = 4;

/// Where a camera's frames come from: a V4L2 device, or a test's camera.
pub trait Source {
    /// The camera once it's open.
    type Opened<'a>: Opened
    where
        Self: 'a;

    /// Opens the camera at `device`, or says why it can't: not plugged in, or busy.
    fn open(&mut self, device: &Path) -> io::Result<Self::Opened<'_>>;
}

/// An open camera: its formats and controls, then its frames. It's closed when dropped.
pub trait Opened: Controls {
    /// The pixel formats it offers, each by its four characters.
    fn formats(&self) -> io::Result<Vec<[u8; 4]>>;
    /// Asks for a size and pixel format, and gives what the camera took instead, if it did.
    fn set_format(&self, wanted: &Format) -> io::Result<Format>;
    /// Asks for a frame rate, and gives the time per frame the camera took.
    fn set_fps(&self, fps: u32) -> io::Result<Fraction>;
    /// Starts it streaming.
    fn start(&mut self) -> io::Result<()>;
    /// The next frame's bytes and the driver's description of them. An error once none has come
    /// for a second, or when the camera has gone.
    fn next(&mut self) -> io::Result<(&[u8], &Metadata)>;
}

/// Captures from a V4L2 device until `stop` is set, reconnecting as needed.
pub fn run(camera: &Camera, latest: &Latest, clocks: &impl Clocks, stop: &AtomicBool) {
    run_from(&mut V4l2, camera, latest, clocks, stop, RETRY);
}

/// Captures from `source` until `stop` is set, opening the camera again `retry` after it fails.
pub fn run_from(
    source: &mut impl Source,
    camera: &Camera,
    latest: &Latest,
    clocks: &impl Clocks,
    stop: &AtomicBool,
    retry: Duration,
) {
    let mut last_error = None;
    // Across reopens, so a camera that keeps coming back says each warning once.
    let mut said = Said::default();
    while !stop.load(Ordering::Relaxed) {
        match capture(
            source,
            camera,
            latest,
            clocks,
            stop,
            &mut last_error,
            &mut said,
        ) {
            Ok(()) => return,
            Err(error) => {
                // Said once per cause, not every 300 ms.
                let text = error.to_string();
                if last_error.as_deref() != Some(text.as_str()) {
                    warn!(
                        "{}: {text}; trying again every {} ms",
                        camera.name,
                        retry.as_millis()
                    );
                    last_error = Some(text);
                }
            }
        }
        thread::sleep(retry);
    }
}

fn capture(
    source: &mut impl Source,
    camera: &Camera,
    latest: &Latest,
    clocks: &impl Clocks,
    stop: &AtomicBool,
    last_error: &mut Option<String>,
    said: &mut Said,
) -> io::Result<()> {
    let mut opened = source
        .open(&camera.device)
        .map_err(|e| io::Error::new(e.kind(), format!("{}: {e}", camera.device.display())))?;
    let (format, pixels) = set_format(&opened, camera)?;
    let interval = opened.set_fps(camera.fps)?;
    let applied = settings::apply(&opened, camera)?;
    for warning in &applied.warnings {
        if said.first_time(warning) {
            warn!("{}: {warning}", camera.name);
        }
    }
    let in_effect = applied.in_effect;
    info!("{}: {}", camera.name, described(in_effect));
    opened.start()?;
    info!(
        "{}: streaming {}x{} {} at {} s per frame",
        camera.name, format.width, format.height, format.fourcc, interval
    );
    *last_error = None;

    let mut said_time_source = false;
    while !stop.load(Ordering::Relaxed) {
        let (data, meta) = opened.next()?;
        if meta.flags.contains(Flags::ERROR) {
            latest.broken();
            continue;
        }
        let stamped = meta.flags.contains(Flags::TIMESTAMP_MONOTONIC);
        let monotonic_nanos = if stamped {
            meta.timestamp.sec * 1_000_000_000 + meta.timestamp.usec * 1_000
        } else {
            clocks.monotonic_nanos()
        };
        if !said_time_source {
            info!(
                "{}: capture times from {}",
                camera.name,
                time_source(meta.flags)
            );
            said_time_source = true;
        }
        let used = &data[..(meta.bytesused as usize).min(data.len())];
        let mut image = latest.spare(camera.width, camera.height);
        let copied = match pixels {
            PixelFormat::Grey => image.copy_grey(used, format.stride as usize),
            PixelFormat::Yuyv => image.copy_yuyv_luma(used, format.stride as usize),
        };
        match copied {
            Ok(()) => latest.put(Captured {
                image,
                monotonic_nanos,
                dequeue_time: !stamped,
                settings: in_effect,
            }),
            // A short frame is skipped, not a reason to reconnect, and counted for the detect
            // thread's report rather than logged: a struggling camera can send dozens a second.
            Err(error) => {
                latest.short(error.to_string());
                latest.recycle(image);
            }
        }
    }
    Ok(())
}

/// The cameras V4L2 gives, their frames memory-mapped.
pub struct V4l2;

impl Source for V4l2 {
    type Opened<'a> = V4l2Camera;

    fn open(&mut self, device: &Path) -> io::Result<V4l2Camera> {
        Ok(V4l2Camera {
            stream: None,
            device: Device::with_path(device)?,
        })
    }
}

/// An open V4L2 device, and its stream once started.
pub struct V4l2Camera {
    // Before the device, so its buffers are given back before the device is closed.
    stream: Option<Stream<'static>>,
    device: Device,
}

impl Controls for V4l2Camera {
    fn list(&self) -> io::Result<Vec<Control>> {
        self.device.list()
    }

    fn get(&self, control: &Control) -> io::Result<i64> {
        Controls::get(&self.device, control)
    }

    fn set(&self, control: &Control, value: i64) -> io::Result<()> {
        Controls::set(&self.device, control, value)
    }
}

impl Opened for V4l2Camera {
    fn formats(&self) -> io::Result<Vec<[u8; 4]>> {
        Ok(self
            .device
            .enum_formats()?
            .iter()
            .map(|f| f.fourcc.repr)
            .collect())
    }

    fn set_format(&self, wanted: &Format) -> io::Result<Format> {
        Capture::set_format(&self.device, wanted)
    }

    fn set_fps(&self, fps: u32) -> io::Result<Fraction> {
        Ok(self.device.set_params(&Parameters::with_fps(fps))?.interval)
    }

    fn start(&mut self) -> io::Result<()> {
        let mut stream = Stream::with_buffers(&self.device, Type::VideoCapture, BUFFERS)?;
        stream.set_timeout(STALL);
        self.stream = Some(stream);
        Ok(())
    }

    fn next(&mut self) -> io::Result<(&[u8], &Metadata)> {
        let Some(stream) = self.stream.as_mut() else {
            return Err(io::Error::other("the camera isn't streaming"));
        };
        stream.next().map_err(|error| match error.kind() {
            // What the stream says when its wait runs out: VIDIOC_DQBUF, the call that waited.
            io::ErrorKind::TimedOut => io::Error::new(
                io::ErrorKind::TimedOut,
                format!("no frame for {} s", STALL.as_secs()),
            ),
            _ => error,
        })
    }
}

/// Sets the configured size and pixel format, and checks the camera took them.
fn set_format(device: &impl Opened, camera: &Camera) -> io::Result<(Format, PixelFormat)> {
    let offered = device.formats()?;
    let pixels = choose_format(&offered, camera.format)
        .map_err(|why| io::Error::new(io::ErrorKind::Unsupported, why))?;
    let wanted = Format::new(camera.width, camera.height, FourCC::new(&pixels.fourcc()));
    let actual = device.set_format(&wanted)?;
    if (actual.width, actual.height, actual.fourcc) != (wanted.width, wanted.height, wanted.fourcc)
    {
        return Err(io::Error::new(
            io::ErrorKind::Unsupported,
            format!(
                "asked for {}x{} {}, the camera gives {}x{} {}",
                wanted.width,
                wanted.height,
                wanted.fourcc,
                actual.width,
                actual.height,
                actual.fourcc
            ),
        ));
    }
    Ok((actual, pixels))
}

/// The pixel format to use, from those the camera offers: the configured one, or else `GREY` if
/// offered, then `YUYV`.
pub fn choose_format(
    offered: &[[u8; 4]],
    wanted: Option<PixelFormat>,
) -> Result<PixelFormat, String> {
    let candidates = match wanted {
        Some(format) => vec![format],
        None => vec![PixelFormat::Grey, PixelFormat::Yuyv],
    };
    candidates
        .into_iter()
        .find(|format| offered.contains(&format.fourcc()))
        .ok_or_else(|| {
            let names: Vec<String> = offered
                .iter()
                .map(|f| String::from_utf8_lossy(f).into_owned())
                .collect();
            let asked = match wanted {
                Some(format) => String::from_utf8_lossy(&format.fourcc()).into_owned(),
                None => "GREY or YUYV".into(),
            };
            format!("the camera offers {}, not {asked}", names.join(", "))
        })
}

/// The exposure and gain in effect, as the log says it once the camera is open.
fn described(in_effect: InEffect) -> String {
    let exposure = match (in_effect.auto_exposure, in_effect.exposure_us) {
        (Some(true), _) => "the camera's automatic exposure".to_string(),
        (_, Some(us)) => format!("exposure {:.1} ms", f64::from(us) / 1000.0),
        (_, None) => "exposure unknown".to_string(),
    };
    match in_effect.gain {
        Some(gain) => format!("{exposure}, gain {gain}"),
        None => format!("{exposure}, no gain control"),
    }
}

fn time_source(flags: Flags) -> &'static str {
    if !flags.contains(Flags::TIMESTAMP_MONOTONIC) {
        return "when Visor got each frame (the driver gave no timestamp)";
    }
    if flags.contains(Flags::TSTAMP_SRC_SOE) {
        "the driver, at the start of exposure"
    } else {
        "the driver, at the end of the frame"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::{Cell, RefCell};
    use std::collections::VecDeque;
    use std::path::PathBuf;
    use std::sync::Mutex;

    use v4l::timestamp::Timestamp;

    use crate::settings::{AUTO_EXPOSURE, EXPOSURE_TIME, Kind};

    /// The test camera's size, in GREY: a frame is 32 bytes.
    const WIDTH: u32 = 8;
    const HEIGHT: u32 = 4;
    const GREY: &[[u8; 4]] = &[*b"GREY"];
    const MJPG: &[[u8; 4]] = &[*b"MJPG"];

    /// What the test's camera does each time it's opened.
    enum Plug {
        /// It isn't there: unplugged, or nothing in its port.
        Absent,
        /// It offers only these pixel formats.
        Offers(&'static [[u8; 4]]),
        /// It gives another size than the one asked for.
        Gives(u32, u32),
        /// It refuses the value of a control the configuration names: busy, say.
        RefusesControl,
        /// It streams these, in order.
        Streams(Vec<Event>),
    }

    /// What a streaming camera gives for one frame. Each is numbered, from 1 for the camera's first.
    #[derive(Clone, Copy)]
    enum Event {
        /// A whole frame.
        Frame,
        /// A frame shorter than its size.
        Short,
        /// A frame the driver marked bad.
        Errored,
        /// No frame for a second: the stream's wait runs out.
        Stall,
        /// Nothing: the camera was unplugged mid-stream.
        Unplugged,
    }

    /// A camera that does what its plugs say, one each time it's opened, and counts what's done
    /// with it. Its last act sets `stop`, so the capture loop ends with the plugs.
    struct Fake<'a> {
        plugs: VecDeque<Plug>,
        latest: &'a Latest,
        stop: &'a AtomicBool,
        opened: u32,
        /// How many times its exposure was set: once for each open that got that far.
        exposures_set: Cell<u32>,
        /// How many events it has given.
        given: u32,
        /// The number of each frame that reached the slot, taken as the detect thread would.
        taken: Vec<i64>,
    }

    impl Fake<'_> {
        /// Takes what's in the slot, as the detect thread would.
        fn take(&mut self) {
            while let Some(frame) = self.latest.take(Duration::ZERO) {
                self.taken.push(frame.monotonic_nanos / 1000);
                self.latest.recycle(frame.image);
            }
        }
    }

    /// The test's camera, open.
    struct FakeCamera<'s, 'a> {
        fake: &'s mut Fake<'a>,
        offers: &'static [[u8; 4]],
        gives: (u32, u32),
        events: VecDeque<Event>,
        /// Whether it refuses its brightness.
        refuses_control: bool,
        values: RefCell<Vec<(u32, i64)>>,
        frame: Vec<u8>,
        meta: Metadata,
    }

    impl<'a> Source for Fake<'a> {
        type Opened<'s>
            = FakeCamera<'s, 'a>
        where
            Self: 's;

        fn open(&mut self, _device: &Path) -> io::Result<FakeCamera<'_, 'a>> {
            let plug = self
                .plugs
                .pop_front()
                .expect("the loop stops with the plugs");
            if self.plugs.is_empty() && !matches!(plug, Plug::Streams(_)) {
                self.stop.store(true, Ordering::Relaxed);
            }
            let refuses_control = matches!(plug, Plug::RefusesControl);
            let (offers, gives, events) = match plug {
                Plug::Absent => return Err(io::Error::from_raw_os_error(libc::ENOENT)),
                Plug::Offers(offers) => (offers, (WIDTH, HEIGHT), Vec::new()),
                Plug::Gives(width, height) => (GREY, (width, height), Vec::new()),
                Plug::RefusesControl => (GREY, (WIDTH, HEIGHT), Vec::new()),
                Plug::Streams(events) => (GREY, (WIDTH, HEIGHT), events),
            };
            self.opened += 1;
            Ok(FakeCamera {
                fake: self,
                offers,
                gives,
                events: events.into(),
                refuses_control,
                // Its controls at their defaults, as one just plugged in is.
                values: RefCell::new(vec![
                    (AUTO_EXPOSURE, 3),
                    (EXPOSURE_TIME, 156),
                    (BRIGHTNESS, 0),
                ]),
                frame: vec![0; (WIDTH * HEIGHT) as usize],
                meta: Metadata::default(),
            })
        }
    }

    /// Its brightness control's ID, V4L2's.
    const BRIGHTNESS: u32 = 0x0098_0900;

    /// Manual and automatic exposure, an exposure time and a brightness: no gain control.
    impl Controls for FakeCamera<'_, '_> {
        fn list(&self) -> io::Result<Vec<Control>> {
            let control = |id, name: &str, kind, maximum, default| Control {
                id,
                name: name.into(),
                kind,
                minimum: 1,
                maximum,
                step: 1,
                default,
                menu: Vec::new(),
                inactive: false,
            };
            Ok(vec![
                control(AUTO_EXPOSURE, "auto_exposure", Kind::Menu, 3, 3),
                control(
                    EXPOSURE_TIME,
                    "exposure_time_absolute",
                    Kind::Integer,
                    5000,
                    156,
                ),
                control(BRIGHTNESS, "brightness", Kind::Integer, 64, 0),
            ])
        }

        fn get(&self, control: &Control) -> io::Result<i64> {
            let values = self.values.borrow();
            Ok(values.iter().find(|(id, _)| *id == control.id).unwrap().1)
        }

        fn set(&self, control: &Control, value: i64) -> io::Result<()> {
            if control.id == BRIGHTNESS && self.refuses_control {
                return Err(io::Error::from_raw_os_error(libc::EBUSY));
            }
            if control.id == EXPOSURE_TIME {
                self.fake
                    .exposures_set
                    .set(self.fake.exposures_set.get() + 1);
            }
            let mut values = self.values.borrow_mut();
            values
                .iter_mut()
                .find(|(id, _)| *id == control.id)
                .unwrap()
                .1 = value;
            Ok(())
        }
    }

    impl Opened for FakeCamera<'_, '_> {
        fn formats(&self) -> io::Result<Vec<[u8; 4]>> {
            Ok(self.offers.to_vec())
        }

        fn set_format(&self, wanted: &Format) -> io::Result<Format> {
            let (width, height) = self.gives;
            Ok(Format {
                stride: width,
                ..Format::new(width, height, wanted.fourcc)
            })
        }

        fn set_fps(&self, fps: u32) -> io::Result<Fraction> {
            Ok(Fraction::new(1, fps))
        }

        fn start(&mut self) -> io::Result<()> {
            Ok(())
        }

        fn next(&mut self) -> io::Result<(&[u8], &Metadata)> {
            self.fake.take();
            let event = self.events.pop_front().expect("a stream ends in an error");
            if self.events.is_empty() && self.fake.plugs.is_empty() {
                self.fake.stop.store(true, Ordering::Relaxed);
            }
            self.fake.given += 1;
            let whole = WIDTH * HEIGHT;
            let stamped = Flags::TIMESTAMP_MONOTONIC;
            let (bytesused, flags) = match event {
                Event::Frame => (whole, stamped),
                Event::Short => (whole / 2, stamped),
                Event::Errored => (whole, stamped | Flags::ERROR),
                Event::Stall => {
                    return Err(io::Error::new(io::ErrorKind::TimedOut, "no frame for 1 s"));
                }
                Event::Unplugged => return Err(io::Error::from_raw_os_error(libc::ENODEV)),
            };
            // Stamped with its number, in microseconds, so the slot's frames say which they were.
            self.meta = Metadata {
                bytesused,
                flags,
                timestamp: Timestamp {
                    sec: 0,
                    usec: self.fake.given as libc::time_t,
                },
                ..Metadata::default()
            };
            Ok((&self.frame, &self.meta))
        }
    }

    /// Clocks for frames the driver stamps: never read.
    struct Unread;

    impl Clocks for Unread {
        fn monotonic_nanos(&self) -> i64 {
            unreachable!("every frame is stamped")
        }
        fn wall_nanos(&self) -> i64 {
            unreachable!()
        }
        fn synced(&self) -> bool {
            unreachable!()
        }
    }

    /// The warnings logged in this test program, so a row can check what its camera said.
    struct Recorder(Mutex<Vec<String>>);

    impl log::Log for Recorder {
        fn enabled(&self, metadata: &log::Metadata) -> bool {
            metadata.level() <= log::Level::Warn
        }
        fn log(&self, record: &log::Record) {
            if self.enabled(record.metadata()) {
                self.0.lock().unwrap().push(record.args().to_string());
            }
        }
        fn flush(&self) {}
    }

    static RECORDER: Recorder = Recorder(Mutex::new(Vec::new()));

    /// What the camera named `name` warned of, its name taken off.
    fn warnings_of(name: &str) -> Vec<String> {
        let prefix = format!("{name}: ");
        let logged = RECORDER.0.lock().unwrap();
        logged
            .iter()
            .filter_map(|line| line.strip_prefix(&prefix))
            .map(str::to_string)
            .collect()
    }

    fn camera(name: &str, gain: Option<i64>) -> Camera {
        Camera {
            name: name.into(),
            index: 0,
            device: PathBuf::from("/dev/v4l/by-path/test-usb-0:1:1.0-video-index0"),
            width: WIDTH,
            height: HEIGHT,
            fps: 60,
            format: None,
            min_tag_px: 16,
            time_offset_nanos: 0,
            exposure_us: Some(1000),
            gain,
            controls: Vec::new(),
            lens: None,
            tag_size_m: 0.1651,
            stream: crate::config::Stream::default(),
            config_id: 0,
        }
    }

    /// What the camera does each time it's opened, the configured gain (it has no gain control)
    /// and named controls; then the frames that reached the slot by number, the short frames, the frames
    /// dropped (short or broken), how many times it was opened, how many times it was set up to
    /// stream, and part of each warning, in order.
    struct Case {
        name: &'static str,
        plugs: Vec<Plug>,
        gain: Option<i64>,
        named: Vec<(&'static str, i64)>,
        taken: Vec<i64>,
        short: u32,
        dropped: u64,
        opened: u32,
        set_up: u32,
        warnings: Vec<&'static str>,
    }

    #[test]
    fn the_capture_loop_streams_skips_and_reopens_as_the_camera_does() {
        let _ = log::set_logger(&RECORDER);
        log::set_max_level(log::LevelFilter::Warn);
        use Event::*;
        let cases = [
            Case {
                name: "a short frame is skipped and counted, and the camera streams on",
                plugs: vec![Plug::Streams(vec![Frame, Short, Frame])],
                gain: None,
                named: vec![],
                taken: vec![1, 3],
                short: 1,
                dropped: 1,
                opened: 1,
                set_up: 1,
                warnings: vec![],
            },
            Case {
                name: "a frame the driver marks bad is counted as dropped, and the camera streams on",
                plugs: vec![Plug::Streams(vec![Frame, Errored, Frame])],
                gain: None,
                named: vec![],
                taken: vec![1, 3],
                short: 0,
                dropped: 1,
                opened: 1,
                set_up: 1,
                warnings: vec![],
            },
            Case {
                name: "a stall: the camera is opened again, and set up again",
                plugs: vec![
                    Plug::Streams(vec![Frame, Stall]),
                    Plug::Streams(vec![Frame]),
                ],
                gain: None,
                named: vec![],
                taken: vec![1, 3],
                short: 0,
                dropped: 0,
                opened: 2,
                set_up: 2,
                warnings: vec!["no frame for 1 s; trying again every 1 ms"],
            },
            Case {
                name: "a format the camera doesn't offer: said once, tried until it's offered",
                plugs: vec![
                    Plug::Offers(MJPG),
                    Plug::Offers(MJPG),
                    Plug::Streams(vec![Frame]),
                ],
                gain: None,
                named: vec![],
                taken: vec![1],
                short: 0,
                dropped: 0,
                opened: 3,
                set_up: 1,
                warnings: vec!["the camera offers MJPG, not GREY or YUYV; trying again"],
            },
            Case {
                name: "a size the camera won't give: said, and tried again",
                plugs: vec![Plug::Gives(4, 4), Plug::Streams(vec![Frame])],
                gain: None,
                named: vec![],
                taken: vec![1],
                short: 0,
                dropped: 0,
                opened: 2,
                set_up: 1,
                warnings: vec!["asked for 8x4 GREY, the camera gives 4x4 GREY; trying again"],
            },
            Case {
                name: "a camera absent for three tries, then plugged in: said once, then frames",
                plugs: vec![
                    Plug::Absent,
                    Plug::Absent,
                    Plug::Absent,
                    Plug::Streams(vec![Frame, Frame]),
                ],
                gain: None,
                named: vec![],
                taken: vec![1, 2],
                short: 0,
                dropped: 0,
                opened: 1,
                set_up: 1,
                warnings: vec![
                    "/dev/v4l/by-path/test-usb-0:1:1.0-video-index0: No such file or directory",
                ],
            },
            Case {
                name: "a named control the camera refuses: the open fails, said once, and is tried \
                       until the camera takes it",
                plugs: vec![
                    Plug::RefusesControl,
                    Plug::RefusesControl,
                    Plug::Streams(vec![Frame]),
                ],
                gain: None,
                named: vec![("brightness", 10)],
                taken: vec![1],
                short: 0,
                dropped: 0,
                opened: 3,
                set_up: 3,
                warnings: vec!["setting brightness: Device or resource busy"],
            },
            Case {
                name: "unplugged and plugged back in: set up again, each cause said once, and the \
                       camera's own warning only the first time",
                plugs: vec![
                    Plug::Streams(vec![Frame, Unplugged]),
                    Plug::Absent,
                    Plug::Absent,
                    Plug::Streams(vec![Frame]),
                ],
                gain: Some(40),
                named: vec![],
                taken: vec![1, 3],
                short: 0,
                dropped: 0,
                opened: 2,
                set_up: 2,
                warnings: vec![
                    "the camera has no gain control",
                    "No such device",
                    "No such file or directory",
                ],
            },
        ];
        for (row, case) in cases.into_iter().enumerate() {
            let name = format!("camera-{row}");
            let mut camera = camera(&name, case.gain);
            camera.controls = case
                .named
                .iter()
                .map(|(n, v)| (n.to_string(), *v))
                .collect();
            let latest = Latest::new();
            let stop = AtomicBool::new(false);
            let mut fake = Fake {
                plugs: case.plugs.into(),
                latest: &latest,
                stop: &stop,
                opened: 0,
                exposures_set: Cell::new(0),
                given: 0,
                taken: Vec::new(),
            };
            let retry = Duration::from_millis(1);
            run_from(&mut fake, &camera, &latest, &Unread, &stop, retry);
            fake.take();
            assert_eq!(fake.taken, case.taken, "{}", case.name);
            assert_eq!(latest.take_short().count, case.short, "{}", case.name);
            assert_eq!(latest.dropped(), case.dropped, "{}", case.name);
            assert_eq!(fake.opened, case.opened, "{}", case.name);
            assert_eq!(fake.exposures_set.get(), case.set_up, "{}", case.name);
            let warned = warnings_of(&name);
            assert_eq!(
                warned.len(),
                case.warnings.len(),
                "{}: {warned:?}",
                case.name
            );
            for (warning, part) in warned.iter().zip(&case.warnings) {
                assert!(warning.contains(part), "{}: {warning}", case.name);
            }
        }
    }

    /// What the camera offers, what's configured, and the choice or part of the refusal.
    type FormatCase<'a> = (
        &'a [[u8; 4]],
        Option<PixelFormat>,
        Result<PixelFormat, &'a str>,
    );

    #[test]
    fn the_pixel_format_is_chosen_from_what_the_camera_offers() {
        let grey_and_yuyv = [*b"MJPG", *b"YUYV", *b"GREY"];
        let only_yuyv = [*b"YUYV"];
        let only_mjpg = [*b"MJPG"];
        let cases: [FormatCase; 5] = [
            (&grey_and_yuyv, None, Ok(PixelFormat::Grey)),
            (&only_yuyv, None, Ok(PixelFormat::Yuyv)),
            (
                &grey_and_yuyv,
                Some(PixelFormat::Yuyv),
                Ok(PixelFormat::Yuyv),
            ),
            (
                &only_yuyv,
                Some(PixelFormat::Grey),
                Err("offers YUYV, not GREY"),
            ),
            (&only_mjpg, None, Err("offers MJPG, not GREY or YUYV")),
        ];
        for (offered, wanted, expected) in cases {
            match (choose_format(offered, wanted), expected) {
                (Ok(chosen), Ok(format)) => assert_eq!(chosen, format),
                (Err(why), Err(part)) => assert!(why.contains(part), "{why}"),
                (got, expected) => panic!("{got:?}, expected {expected:?}"),
            }
        }
    }
}
