//! The service's wiring end to end: two cameras, each on its own scene, through capture, detection
//! and the robot's socket, with the live view's port free and with it taken.

mod common;

use std::collections::BTreeMap;
use std::io;
use std::net::{SocketAddr, TcpListener, UdpSocket};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use v4l::Format;
use v4l::buffer::{Flags, Metadata};
use v4l::fraction::Fraction;
use v4l::timestamp::Timestamp;
use visor::capture::{Opened, Source};
use visor::clock::{Clocks, EPOCH_NANOS};
use visor::config::{Camera, Stream};
use visor::image::GrayImage;
use visor::latest::Latest;
use visor::napa::Detector;
use visor::service::{self, Wired};
use visor::settings::{Control, Controls};
use visor::stream::server::Options;
use visor::wire::Frame;

/// The robot's clock, synced.
struct Synced;

impl Clocks for Synced {
    fn monotonic_nanos(&self) -> i64 {
        5_000_000_000_000
    }
    fn wall_nanos(&self) -> i64 {
        EPOCH_NANOS + 4_000_000_000_000
    }
    fn synced(&self) -> bool {
        true
    }
}

/// A camera that sees one rendered scene, at 30 frames a second, and has no controls.
struct Scene {
    image: GrayImage,
}

impl Source for Scene {
    type Opened<'a> = SceneCamera<'a>;

    fn open(&mut self, _device: &Path) -> io::Result<SceneCamera<'_>> {
        Ok(SceneCamera {
            image: &self.image,
            frames: 0,
            meta: Metadata::default(),
        })
    }
}

struct SceneCamera<'a> {
    image: &'a GrayImage,
    frames: i64,
    meta: Metadata,
}

const PERIOD: Duration = Duration::from_micros(33_333);

impl Controls for SceneCamera<'_> {
    fn list(&self) -> io::Result<Vec<Control>> {
        Ok(Vec::new())
    }
    fn get(&self, control: &Control) -> io::Result<i64> {
        unreachable!("{} isn't listed", control.name)
    }
    fn set(&self, control: &Control, _value: i64) -> io::Result<()> {
        unreachable!("{} isn't listed", control.name)
    }
}

impl Opened for SceneCamera<'_> {
    fn formats(&self) -> io::Result<Vec<[u8; 4]>> {
        Ok(vec![*b"GREY"])
    }

    fn set_format(&self, wanted: &Format) -> io::Result<Format> {
        let size = (self.image.width, self.image.height);
        assert_eq!((wanted.width, wanted.height), size, "the scene's size");
        Ok(Format {
            stride: self.image.width,
            ..Format::new(wanted.width, wanted.height, wanted.fourcc)
        })
    }

    fn set_fps(&self, _fps: u32) -> io::Result<Fraction> {
        Ok(Fraction::new(1, 30))
    }

    fn start(&mut self) -> io::Result<()> {
        Ok(())
    }

    fn next(&mut self) -> io::Result<(&[u8], &Metadata)> {
        thread::sleep(PERIOD);
        self.frames += 1;
        let nanos = self.frames * PERIOD.as_nanos() as i64;
        self.meta = Metadata {
            bytesused: self.image.pixels.len() as u32,
            flags: Flags::TIMESTAMP_MONOTONIC,
            timestamp: Timestamp {
                sec: (nanos / 1_000_000_000) as libc::time_t,
                usec: (nanos % 1_000_000_000 / 1000) as libc::time_t,
            },
            ..Metadata::default()
        };
        Ok((&self.image.pixels, &self.meta))
    }
}

fn camera(name: &str, index: u8) -> Camera {
    Camera {
        name: name.into(),
        index,
        device: PathBuf::from(format!("/dev/v4l/by-path/test-{index}-video-index0")),
        width: 1600,
        height: 1304,
        fps: 30,
        format: None,
        min_tag_px: 16,
        time_offset_nanos: 0,
        exposure_us: None,
        gain: None,
        controls: Vec::new(),
        lens: None,
        tag_size_m: 0.1651,
        stream: Stream::default(),
        config_id: 0,
    }
}

/// Whether the live view's port is free for the service, or taken by another program.
enum Port {
    Free,
    Taken,
}

#[test]
fn each_camera_s_frames_go_out_under_its_own_index_whether_or_not_the_view_has_its_port() {
    // Each camera, its index, its scene, and the one tag in that scene.
    let cameras = [
        (camera("front", 3), "one-tag", 7),
        (camera("back left", 8), "steep-far", 21),
    ];
    let expected: BTreeMap<u8, u16> = cameras
        .iter()
        .map(|(camera, _, tag)| (camera.index, *tag))
        .collect();
    let rows = [
        (
            "the view's port is free: the page lists both cameras",
            Port::Free,
        ),
        (
            "the view's port is taken: detection goes on without it",
            Port::Taken,
        ),
    ];
    for (name, port) in rows {
        let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
        robot
            .set_read_timeout(Some(Duration::from_secs(1)))
            .unwrap();
        let taken = TcpListener::bind("127.0.0.1:0").unwrap();
        let listener = match port {
            Port::Free => {
                let listener = service::listen("127.0.0.1:0".parse().unwrap());
                Some(listener.expect("a free port"))
            }
            Port::Taken => {
                let listener = service::listen(taken.local_addr().unwrap());
                assert!(listener.is_none(), "{name}: the port is another's");
                None
            }
        };
        let view: Option<SocketAddr> = listener.as_ref().map(|l| l.local_addr().unwrap());
        let wired: Vec<Wired<Scene>> = cameras
            .iter()
            .map(|(camera, scene, _)| Wired {
                camera,
                source: Scene {
                    image: common::scene(scene).image,
                },
                detector: Detector::new(camera.min_tag_px).unwrap(),
                socket: UdpSocket::bind("127.0.0.1:0").unwrap(),
                latest: Latest::new(),
            })
            .collect();
        let stop = AtomicBool::new(false);
        let robot_address = robot.local_addr().unwrap();
        let mut frames: BTreeMap<u8, Vec<Frame>> = BTreeMap::new();
        let mut measured = None;
        thread::scope(|scope| {
            let stop = &stop;
            scope.spawn(move || {
                service::run(
                    wired,
                    robot_address,
                    listener,
                    Options::default(),
                    &Synced,
                    stop,
                )
            });
            let deadline = Instant::now() + Duration::from_secs(15);
            let mut buffer = [0u8; 2048];
            while frames.len() < 2 || frames.values().any(|each| each.len() < 5) {
                assert!(Instant::now() < deadline, "{name}: {frames:?}");
                if let Ok(n) = robot.recv(&mut buffer) {
                    let frame = Frame::decode(&buffer[..n]).expect("a frame");
                    frames.entry(frame.camera).or_default().push(frame);
                }
            }
            if let Some(view) = view {
                let page = ureq::get(format!("http://{view}/"))
                    .call()
                    .unwrap()
                    .body_mut()
                    .read_to_string()
                    .unwrap();
                for (camera, _, _) in &cameras {
                    assert!(
                        page.contains(&format!(">{}</a>", camera.name)),
                        "{name}: {page}"
                    );
                }
            }
            if view.is_some() {
                measured = Some(Measured::now());
            }
            stop.store(true, Ordering::Relaxed);
        });
        assert_eq!(
            frames.keys().copied().collect::<Vec<u8>>(),
            expected.keys().copied().collect::<Vec<u8>>(),
            "{name}"
        );
        for (index, received) in &frames {
            for frame in received {
                let ids: Vec<u16> = frame.tags.iter().map(|tag| tag.id).collect();
                assert_eq!(ids, [expected[index]], "{name}: camera {index}");
            }
        }
        if let Some(measured) = measured {
            measured.fits_the_unit(name);
        }
    }
}

/// What this test's process holds while both cameras run and the view is served: the service's
/// files and memory, and the test's own few beside them.
struct Measured {
    files: usize,
    /// The most resident memory so far, in KiB.
    peak_kib: u64,
}

impl Measured {
    fn now() -> Measured {
        let files = std::fs::read_dir("/proc/self/fd").unwrap().count();
        let status = std::fs::read_to_string("/proc/self/status").unwrap();
        let peak = status
            .lines()
            .find_map(|line| line.strip_prefix("VmHWM:"))
            .expect("the peak resident size");
        let peak_kib = peak.trim().trim_end_matches(" kB").parse().unwrap();
        Measured { files, peak_kib }
    }

    /// That the unit's limits (deploy/visor.service) keep their margins over what's measured: twice
    /// the files with every connection the view allows open, and four times the memory.
    fn fits_the_unit(&self, name: &str) {
        let unit = std::fs::read_to_string(
            Path::new(env!("CARGO_MANIFEST_DIR")).join("deploy/visor.service"),
        )
        .unwrap();
        let setting = |key: &str| -> u64 {
            let line = unit.lines().find_map(|line| line.strip_prefix(key));
            let value = line.unwrap_or_else(|| panic!("deploy/visor.service sets no {key}"));
            match value.strip_suffix('M') {
                Some(mib) => mib.parse::<u64>().unwrap() * 1024,
                None => value.parse().unwrap(),
            }
        };
        let options = Options::default();
        let connections = options.most_connections + options.kept_for_controller;
        let files = (self.files + connections) as u64;
        assert!(
            files * 2 <= setting("LimitNOFILE="),
            "{name}: {} files open, and {connections} connections",
            self.files
        );
        assert!(
            self.peak_kib * 4 <= setting("MemoryMax="),
            "{name}: {} KiB resident at most",
            self.peak_kib
        );
    }
}
