//! The capture loop against a camera of the test's, scripted open by open and frame by frame.

use super::*;
use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::Mutex;

use v4l::timestamp::Timestamp;

use crate::usb::{Link, Version};

use crate::settings::{AUTO_EXPOSURE, Control, EXPOSURE_TIME, Kind};

/// The test camera's size, in GREY: a frame is 32 bytes.
const WIDTH: u32 = 8;
const HEIGHT: u32 = 4;
/// A format the camera offers, with its one frame rate at the test's size.
type Offer = ([u8; 4], f64);
const GREY: &[Offer] = &[(*b"GREY", 60.0)];
const H264: &[Offer] = &[(*b"H264", 60.0)];

/// What the test's camera does each time it's opened.
enum Plug {
    /// It isn't there: unplugged, or nothing in its port.
    Absent,
    /// It offers only these pixel formats.
    Offers(&'static [Offer]),
    /// It gives another size than the one asked for.
    Gives(u32, u32),
    /// It refuses the value of a control the configuration names: busy, say.
    RefusesControl,
    /// It streams these, in order.
    Streams(Vec<Event>),
    /// It offers these formats, and streams these in whichever it's asked for.
    StreamsFrom(&'static [Offer], Vec<Event>),
}

/// What a streaming camera gives for one frame. Each is numbered, from 1 for the camera's first.
#[derive(Clone, Copy)]
enum Event {
    /// A whole frame.
    Frame,
    /// A frame shorter than its size.
    Short,
    /// A frame of the right size whose bytes are nonsense: a compressed frame that won't decode.
    Garbled,
    /// A frame the driver marked bad.
    Errored,
    /// A frame the driver numbered and threw away: the camera sent it incomplete. Nothing is
    /// given for it; the next frame's number shows it.
    Withheld,
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
    /// The exposure each of those frames says it was taken at.
    exposures: Vec<Option<u32>>,
    /// The link state each of those frames says.
    links: Vec<LinkState>,
    /// Whether its port's power can be switched.
    port: Port,
    /// The USB link it's on, which a reset, and then a power cycle, changes to the next of
    /// `resets`.
    link: Option<Link>,
    resets: VecDeque<Link>,
    /// The stall time each stream started with.
    stalls: Vec<Duration>,
    /// A frame of it as a JPEG.
    jpeg: Vec<u8>,
}

impl<'a> Fake<'a> {
    /// A camera that does what `plugs` say, on no known link, with a JPEG of its frame ready.
    fn new(plugs: Vec<Plug>, latest: &'a Latest, stop: &'a AtomicBool) -> Fake<'a> {
        Fake {
            plugs: plugs.into(),
            latest,
            stop,
            opened: 0,
            exposures_set: Cell::new(0),
            given: 0,
            taken: Vec::new(),
            exposures: Vec::new(),
            links: Vec::new(),
            link: None,
            port: Port::default(),
            resets: VecDeque::new(),
            stalls: Vec::new(),
            jpeg: crate::image::fixtures::jpeg(WIDTH as u16, HEIGHT as u16, |_, _| 128),
        }
    }

    /// Takes what's in the slot, as the detect thread would.
    fn take(&mut self) {
        while let Some(frame) = self.latest.take(Duration::ZERO) {
            self.taken.push(frame.monotonic_nanos / 1000);
            self.exposures.push(frame.settings.exposure_us);
            self.links.push(frame.link);
            self.latest.recycle(frame.image);
        }
    }
}

/// The test's camera, open.
struct FakeCamera<'s, 'a> {
    fake: &'s mut Fake<'a>,
    offers: &'static [Offer],
    /// The format it was asked for.
    asked: Cell<[u8; 4]>,
    gives: (u32, u32),
    events: VecDeque<Event>,
    /// Whether it refuses its brightness.
    refuses_control: bool,
    values: RefCell<Vec<(u32, i64)>>,
    frame: Vec<u8>,
    meta: Metadata,
    /// The driver's number for its next frame: from 0 each time it's opened, as V4L2's is.
    sequence: u32,
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
        if self.plugs.is_empty() && !matches!(plug, Plug::Streams(_) | Plug::StreamsFrom(..)) {
            self.stop.store(true, Ordering::Relaxed);
        }
        let refuses_control = matches!(plug, Plug::RefusesControl);
        let (offers, gives, events) = match plug {
            Plug::Absent => return Err(io::Error::from_raw_os_error(libc::ENOENT)),
            Plug::Offers(offers) => (offers, (WIDTH, HEIGHT), Vec::new()),
            Plug::Gives(width, height) => (GREY, (width, height), Vec::new()),
            Plug::RefusesControl => (GREY, (WIDTH, HEIGHT), Vec::new()),
            Plug::Streams(events) => (GREY, (WIDTH, HEIGHT), events),
            Plug::StreamsFrom(offers, events) => (offers, (WIDTH, HEIGHT), events),
        };
        self.opened += 1;
        Ok(FakeCamera {
            fake: self,
            offers,
            asked: Cell::new(*b"GREY"),
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
            sequence: 0,
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

/// Whether the power of the port the test's camera is on can be switched.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
enum Port {
    #[default]
    Fixed,
    Switchable,
}

/// The camera's USB side: its link, and a reset, then a power cycle where its port can be
/// switched, that each move it to the next scripted link.
impl Usb for Fake<'_> {
    fn link(&mut self, _device: &Path) -> io::Result<Option<Link>> {
        Ok(self.link.clone())
    }

    fn reset(&mut self, _device: &Path, _link: &Link) -> io::Result<crate::usb::Back> {
        self.link = Some(self.resets.pop_front().expect("a reset the row expects"));
        Ok(crate::usb::Back::Returned)
    }

    fn find(&mut self, _identity: &crate::usb::Identity) -> io::Result<crate::usb::Found> {
        Ok(crate::usb::Found::Nothing)
    }

    fn power_port(&mut self, _link: &Link) -> io::Result<crate::usb::PowerPort> {
        Ok(match self.port {
            Port::Switchable => crate::usb::PowerPort::Switchable("a test's".into()),
            Port::Fixed => crate::usb::PowerPort::Fixed("a test's".into()),
        })
    }

    fn power_cycle(&mut self, _device: &Path, _port: &Path) -> io::Result<crate::usb::Back> {
        assert_eq!(self.port, Port::Switchable, "a port that can't be switched");
        self.link = Some(
            self.resets
                .pop_front()
                .expect("a power cycle the row expects"),
        );
        Ok(crate::usb::Back::Returned)
    }

    fn now(&mut self) -> Duration {
        Duration::ZERO
    }
}

impl Opened for FakeCamera<'_, '_> {
    fn formats(&self) -> io::Result<Vec<[u8; 4]>> {
        Ok(self.offers.iter().map(|(fourcc, _)| *fourcc).collect())
    }

    fn rates(&self, fourcc: [u8; 4], width: u32, height: u32) -> Rates {
        if (width, height) != (WIDTH, HEIGHT) {
            return Rates::NotAtThisSize;
        }
        let offered = self.offers.iter().find(|(each, _)| *each == fourcc);
        Rates::Offered(offered.map(|(_, fps)| *fps).into_iter().collect())
    }

    fn set_format(&self, wanted: &Format) -> io::Result<Format> {
        self.asked.set(wanted.fourcc.repr);
        let (width, height) = self.gives;
        Ok(Format {
            stride: width,
            ..Format::new(width, height, wanted.fourcc)
        })
    }

    /// The rate it offers in the format asked for, whatever the rate asked.
    fn set_fps(&self, _fps: u32) -> io::Result<Fraction> {
        let asked = self.asked.get();
        let (_, fps) = self.offers.iter().find(|(each, _)| *each == asked).unwrap();
        Ok(Fraction::new(1, *fps as u32))
    }

    fn start(&mut self, stall: Duration) -> io::Result<()> {
        self.fake.stalls.push(stall);
        Ok(())
    }

    fn next(&mut self) -> io::Result<(&[u8], &Metadata)> {
        self.fake.take();
        let mut event = self.events.pop_front().expect("a stream ends in an error");
        // A frame the driver threw away takes its number, and the camera's, and isn't given.
        while matches!(event, Event::Withheld) {
            self.fake.given += 1;
            self.sequence += 1;
            event = self
                .events
                .pop_front()
                .expect("a frame after those withheld");
        }
        if self.events.is_empty() && self.fake.plugs.is_empty() {
            self.fake.stop.store(true, Ordering::Relaxed);
        }
        self.fake.given += 1;
        let sequence = self.sequence;
        self.sequence += 1;
        if self.asked.get() == *b"MJPG" {
            self.frame = self.fake.jpeg.clone();
        }
        if let Event::Garbled = event {
            self.frame.fill(0x55);
        }
        let whole = self.frame.len() as u32;
        let stamped = Flags::TIMESTAMP_MONOTONIC;
        let (bytesused, flags) = match event {
            Event::Frame | Event::Garbled => (whole, stamped),
            Event::Short => (whole / 2, stamped),
            Event::Errored => (whole, stamped | Flags::ERROR),
            Event::Withheld => unreachable!("never given"),
            Event::Stall => return Err(io::Error::from(io::ErrorKind::TimedOut)),
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
            sequence,
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
/// dropped (short, broken, or never given by the driver), how many times it was opened, how
/// many times it was set up to stream, and part of each warning, in order.
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
            name: "frames the driver numbers and never gives are counted as dropped, and the \
                   camera streams on",
            plugs: vec![Plug::Streams(vec![
                Frame, Withheld, Frame, Frame, Withheld, Withheld, Frame,
            ])],
            gain: None,
            named: vec![],
            taken: vec![1, 3, 4, 7],
            short: 0,
            dropped: 3,
            opened: 1,
            set_up: 1,
            warnings: vec![],
        },
        Case {
            name: "a frame withheld before a broken one: each is counted once",
            plugs: vec![Plug::Streams(vec![Frame, Withheld, Errored, Frame])],
            gain: None,
            named: vec![],
            taken: vec![1, 4],
            short: 0,
            dropped: 2,
            opened: 1,
            set_up: 1,
            warnings: vec![],
        },
        Case {
            name: "frames withheld before a stream's first: nothing came before to count from, \
                   so none; and the numbers start again when the camera is opened again",
            plugs: vec![
                Plug::Streams(vec![Withheld, Frame, Frame, Frame, Stall]),
                Plug::Streams(vec![Withheld, Withheld, Frame]),
            ],
            gain: None,
            named: vec![],
            taken: vec![2, 3, 4, 8],
            short: 0,
            dropped: 0,
            opened: 2,
            set_up: 2,
            warnings: vec!["no frame for 1 s; trying again every 1 ms"],
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
                Plug::Offers(H264),
                Plug::Offers(H264),
                Plug::Streams(vec![Frame]),
            ],
            gain: None,
            named: vec![],
            taken: vec![1],
            short: 0,
            dropped: 0,
            opened: 3,
            set_up: 1,
            warnings: vec!["the camera offers H264, not GREY, YUYV or MJPG; trying again"],
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
        let mut fake = Fake::new(case.plugs, &latest, &stop);
        let retry = Duration::from_millis(1);
        run_from(
            &mut fake,
            &camera,
            &latest,
            &Unread,
            &Trial::default(),
            &stop,
            retry,
        );
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

#[test]
fn an_exposure_tried_is_set_before_the_next_frame_and_gone_when_the_camera_is_opened_again() {
    use Event::*;
    let camera = camera("tried", None);
    let latest = Latest::new();
    let stop = AtomicBool::new(false);
    // A frame, a stall, and the camera opened again for one more.
    let plugs = vec![
        Plug::Streams(vec![Frame, Stall]),
        Plug::Streams(vec![Frame]),
    ];
    let mut fake = Fake::new(plugs, &latest, &stop);
    let trial = Trial::default();
    trial.ask(settings::Tried {
        exposure_us: Some(2000),
        gain: None,
    });
    let retry = Duration::from_millis(1);
    run_from(&mut fake, &camera, &latest, &Unread, &trial, &stop, retry);
    fake.take();
    // The first frame is at the exposure tried; the camera opened again is at the
    // configuration's, and nothing is tried twice.
    assert_eq!(fake.exposures, [Some(2000), Some(1000)]);
    assert_eq!(fake.opened, 2);
    assert_eq!(fake.exposures_set.get(), 3, "each opening, and the trial");
    assert_eq!(trial.take(), None);
}

#[test]
fn the_frames_the_driver_never_gave_are_the_numbers_it_skipped() {
    // (the number of the frame before, this frame's, the frames between that never came)
    let cases: [(&str, Option<u32>, u32, u32); 7] = [
        ("a stream's first frame", None, 0, 0),
        ("a stream's first frame, numbered late", None, 5, 0),
        ("the next frame", Some(5), 6, 0),
        ("one skipped", Some(5), 7, 1),
        ("a second's worth skipped", Some(5), 66, 60),
        ("a driver that numbers every frame 0", Some(0), 0, 0),
        ("numbers that go back", Some(9), 2, 0),
    ];
    for (name, last, now, expected) in cases {
        assert_eq!(withheld(last, now), expected, "{name}");
    }
}

/// The camera stuck at USB 2, as a USB 3 camera reports it there, and back at SuperSpeed; and a
/// USB 2 camera.
fn on(mbps: u32, major: u8, minor: u8, address: u32) -> Link {
    Link {
        name: "7-1".into(),
        mbps,
        version: Version { major, minor },
        bus: 7,
        address,
        controller: None,
        identity: crate::usb::Identity::default(),
    }
}

/// What a USB 3 camera offers at its size on USB 2 and at SuperSpeed, and a camera with its size
/// raw only, slowly.
const ON_USB_2: &[Offer] = &[(*b"MJPG", 60.0), (*b"GREY", 5.0)];
const ON_USB_3: &[Offer] = &[(*b"MJPG", 60.0), (*b"GREY", 60.0), (*b"YUYV", 60.0)];
const RAW_SLOWLY: &[Offer] = &[(*b"GREY", 5.0)];

/// A camera on a link and a port, what its reset and then its power cycle do, what it does each
/// time it's opened; then the frames
/// that reached the slot by number, those that didn't decode, the link state they carry, each
/// stream's stall time, and part of each warning, in order.
struct LinkCase {
    name: &'static str,
    link: Option<Link>,
    port: Port,
    resets: Vec<Link>,
    plugs: Vec<Plug>,
    taken: Vec<i64>,
    undecoded: u32,
    state: LinkState,
    stalls: Vec<Duration>,
    warnings: Vec<&'static str>,
}

#[test]
fn the_format_follows_the_link_mjpeg_is_decoded_and_a_slow_camera_waits_longer() {
    use crate::link_state::{Headroom, Speed};
    use crate::recovery::Recovered;
    use Event::*;
    let _ = log::set_logger(&RECORDER);
    log::set_max_level(log::LevelFilter::Warn);
    let second = Duration::from_secs(1);
    let cases = [
        LinkCase {
            name: "stuck at USB 2 on a port whose power can't be switched: never reset, MJPG at \
                   the full rate, decoded, a frame that won't decode counted, and why said once",
            link: Some(on(480, 3, 10, 2)),
            port: Port::Fixed,
            resets: vec![],
            plugs: vec![Plug::StreamsFrom(ON_USB_2, vec![Frame, Garbled, Frame])],
            taken: vec![1, 3],
            undecoded: 1,
            state: LinkState {
                speed: Speed::High,
                format: Some(PixelFormat::Mjpg),
                headroom: Headroom::MayDoBetter,
                recovered: Recovered::NotTried,
            },
            stalls: vec![second],
            warnings: vec![
                "on USB 480 Mb/s (a USB 3.10 device, 7-1), below the USB 3 it can do, and Visor \
                 can't switch its port's power, so it sends no USB reset",
            ],
        },
        LinkCase {
            name: "stuck at USB 2 on a switchable port, still there after its reset and its \
                   power cycle: MJPG at the full rate, and what happened said once",
            link: Some(on(480, 3, 10, 2)),
            port: Port::Switchable,
            resets: vec![on(480, 3, 10, 3), on(480, 3, 10, 4)],
            plugs: vec![Plug::StreamsFrom(ON_USB_2, vec![Frame, Frame])],
            taken: vec![1, 2],
            undecoded: 0,
            state: LinkState {
                speed: Speed::High,
                format: Some(PixelFormat::Mjpg),
                headroom: Headroom::MayDoBetter,
                recovered: Recovered::TriedStillSlow,
            },
            stalls: vec![second],
            warnings: vec![
                "still on USB 480 Mb/s (a USB 3.10 device, 7-1) after a USB reset and a power \
                 cycle of its port, so its USB 3 contacts are probably not connecting: reseat its \
                 plug or change its cable. Visor runs it as it is, 8x4 MJPG at 1/60 s per frame,",
            ],
        },
        LinkCase {
            name: "stuck at USB 2, at SuperSpeed after its reset: GREY, and nothing to warn of",
            link: Some(on(480, 3, 10, 2)),
            port: Port::Switchable,
            resets: vec![on(5000, 3, 10, 3)],
            plugs: vec![Plug::StreamsFrom(ON_USB_3, vec![Frame, Frame])],
            taken: vec![1, 2],
            undecoded: 0,
            state: LinkState {
                speed: Speed::Super,
                format: Some(PixelFormat::Grey),
                headroom: Headroom::NoneKnown,
                recovered: Recovered::NotTried,
            },
            stalls: vec![second],
            warnings: vec![],
        },
        LinkCase {
            name: "a USB 2 camera at its best: never reset",
            link: Some(on(480, 2, 0, 2)),
            port: Port::Fixed,
            resets: vec![],
            plugs: vec![Plug::StreamsFrom(ON_USB_2, vec![Frame])],
            taken: vec![1],
            undecoded: 0,
            state: LinkState {
                speed: Speed::High,
                format: Some(PixelFormat::Mjpg),
                headroom: Headroom::NoneKnown,
                recovered: Recovered::NotTried,
            },
            stalls: vec![second],
            warnings: vec![],
        },
        LinkCase {
            name: "its size only raw at 5 fps: GREY, the fastest, said once; it waits 15 frame \
                   times, 3 s, before a stall",
            link: None,
            port: Port::Fixed,
            resets: vec![],
            plugs: vec![
                Plug::StreamsFrom(RAW_SLOWLY, vec![Frame, Stall]),
                Plug::StreamsFrom(RAW_SLOWLY, vec![Frame]),
            ],
            taken: vec![1, 3],
            undecoded: 0,
            state: LinkState {
                format: Some(PixelFormat::Grey),
                ..LinkState::default()
            },
            stalls: vec![3 * second, 3 * second],
            warnings: vec![
                "no format gives 8x4 at 60 fps: GREY, the fastest at that size, at 5.0 fps",
                "no frame for 3 s; trying again every 1 ms",
            ],
        },
    ];
    for (row, case) in cases.into_iter().enumerate() {
        let name = format!("linked-{row}");
        let camera = camera(&name, None);
        let latest = Latest::new();
        let stop = AtomicBool::new(false);
        let mut fake = Fake::new(case.plugs, &latest, &stop);
        fake.link = case.link;
        fake.port = case.port;
        fake.resets = case.resets.into();
        let retry = Duration::from_millis(1);
        run_from(
            &mut fake,
            &camera,
            &latest,
            &Unread,
            &Trial::default(),
            &stop,
            retry,
        );
        fake.take();
        assert_eq!(fake.taken, case.taken, "{}", case.name);
        assert_eq!(latest.take_undecoded(), case.undecoded, "{}", case.name);
        assert_eq!(latest.dropped(), u64::from(case.undecoded), "{}", case.name);
        assert!(fake.resets.is_empty(), "{}: every recovery", case.name);
        assert!(
            fake.links.iter().all(|state| *state == case.state),
            "{}: {:?}",
            case.name,
            fake.links
        );
        assert_eq!(fake.stalls, case.stalls, "{}", case.name);
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

#[test]
fn a_camera_granted_a_slow_rate_waits_for_its_frames_before_it_s_lost() {
    // (the time per frame granted, the stall time)
    let cases = [
        (Fraction::new(1, 60), Duration::from_secs(1)),
        (Fraction::new(1, 30), Duration::from_secs(1)),
        (Fraction::new(1, 15), Duration::from_secs(1)),
        (Fraction::new(1, 10), Duration::from_millis(1500)),
        (Fraction::new(1, 5), Duration::from_secs(3)),
        (Fraction::new(2, 5), Duration::from_secs(6)),
        (Fraction::new(1, 0), Duration::from_secs(1)),
    ];
    for (interval, stall) in cases {
        assert_eq!(stall_after(interval), stall, "{interval}");
    }
    let said = stalled(
        io::Error::from(io::ErrorKind::TimedOut),
        Duration::from_millis(1500),
    );
    assert_eq!(said.to_string(), "no frame for 1.5 s");
    let gone = stalled(
        io::Error::from_raw_os_error(libc::ENODEV),
        Duration::from_secs(1),
    );
    assert_eq!(gone.raw_os_error(), Some(libc::ENODEV));
}
