//! The live view's page served from rendered scenes, with no cameras: for looking at the page in a
//! browser, and checking it there. Three cameras, each a still scene from `testdata/` sent as frames
//! at its rate through detection, the views and the server, as Visor runs them:
//!
//! ```text
//! cargo run --example page -- [port]      (default 5809, on 127.0.0.1 only)
//! ```
//!
//! Then open `http://127.0.0.1:5809/`. Settings can be set from it (`try_settings` on, as a
//! bench's), and each scene answers them as a camera would: brighter or darker with exposure times
//! gain, against the exposure and gain it was "configured" with.

use std::net::{IpAddr, Ipv4Addr, TcpListener};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::thread;
use std::time::{Duration, Instant};

use visor::clock::{Clocks, EPOCH_NANOS};
use visor::config::{Camera, Stream};
use visor::image::GrayImage;
use visor::latest::{Captured, Latest};
use visor::lens::Lens;
use visor::link_state::{LinkState, Speed};
use visor::napa::Detector;
use visor::pipeline;
use visor::settings::{InEffect, Range, Ranges, Trial, Wanted};
use visor::stream::feed::Feed;
use visor::stream::server::{self, Options};
use visor::stream::view::{self, RealTime, View};

/// A board's clock, synced to the robot's.
struct Synced(Instant);

impl Clocks for Synced {
    fn monotonic_nanos(&self) -> i64 {
        self.0.elapsed().as_nanos() as i64
    }
    fn wall_nanos(&self) -> i64 {
        EPOCH_NANOS + self.monotonic_nanos()
    }
    fn synced(&self) -> bool {
        true
    }
}

/// One rendered camera: its name, scene, how bright the scene is drawn at its configured exposure
/// and gain, whether it has a lens calibration, and its link.
struct Scene {
    name: &'static str,
    file: &'static str,
    brightness: f64,
    lens: bool,
    speed: Speed,
}

const SCENES: [Scene; 3] = [
    Scene {
        name: "front left",
        file: "one-tag",
        brightness: 0.6,
        lens: true,
        speed: Speed::Super,
    },
    Scene {
        name: "front right",
        file: "two-tags",
        brightness: 1.0,
        lens: false,
        speed: Speed::High,
    },
    Scene {
        name: "back",
        file: "empty",
        brightness: 0.6,
        lens: true,
        speed: Speed::Super,
    },
];

/// Every rendered camera's configured exposure and gain, and their ranges, as a ThriftyCam's.
const EXPOSURE_US: u32 = 2000;
const GAIN: i64 = 40;
const FPS: u32 = 30;
const RANGES: Ranges = Ranges {
    exposure_us: Some(Range {
        least: 100,
        most: 819_200,
    }),
    gain: Some(Range {
        least: 16,
        most: 255,
    }),
};

fn testdata(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("testdata")
        .join(format!("{name}.png"))
}

fn camera(index: usize, scene: &Scene) -> Camera {
    let focal = 1600f64.hypot(1304.0) / 2.0 / 40f64.to_radians().tan();
    Camera {
        name: scene.name.into(),
        index: index as u8,
        device: PathBuf::from(format!("/dev/v4l/by-id/rendered-{}", scene.file)),
        width: 1600,
        height: 1304,
        fps: FPS,
        format: None,
        min_tag_px: 16,
        time_offset_nanos: 0,
        exposure_us: Some(EXPOSURE_US),
        gain: Some(GAIN),
        controls: Vec::new(),
        lens: scene
            .lens
            .then(|| Lens::new(focal, focal, 799.5, 651.5, &[]).expect("a lens")),
        tag_size_m: 0.1651,
        stream: Stream::default(),
        config_id: 0,
    }
}

/// The scene as the camera sees it at `in_effect`: brighter or darker with exposure times gain.
fn exposed(scene: &GrayImage, base: f64, in_effect: InEffect) -> GrayImage {
    let light = |e: InEffect| {
        f64::from(e.exposure_us.unwrap_or(EXPOSURE_US)) * f64::from(e.gain.unwrap_or(GAIN as i32))
    };
    let configured = InEffect {
        exposure_us: Some(EXPOSURE_US),
        gain: Some(GAIN as i32),
        auto_exposure: Some(false),
    };
    let factor = base * light(in_effect) / light(configured);
    let mut out = scene.clone();
    for pixel in &mut out.pixels {
        *pixel = (f64::from(*pixel) * factor).round().clamp(0.0, 255.0) as u8;
    }
    out
}

/// A camera's frames at its rate, answering what's asked of it through `trial` as a capture thread
/// does.
fn capture(scene: &Scene, latest: &Latest, trial: &Trial, clocks: &Synced, stop: &AtomicBool) {
    let png = image::open(testdata(scene.file))
        .expect("a rendered frame")
        .to_luma8();
    let image = GrayImage {
        width: png.width(),
        height: png.height(),
        pixels: png.into_raw(),
    };
    let mut in_effect = InEffect {
        exposure_us: Some(EXPOSURE_US),
        gain: Some(GAIN as i32),
        auto_exposure: Some(false),
    };
    trial.offer(RANGES);
    trial.offer_serial(Some(format!("RENDERED{}", scene.file.len())));
    trial.offer_configured(in_effect);
    let configured = in_effect;
    let mut seen = exposed(&image, scene.brightness, in_effect);
    let period = Duration::from_secs(1) / FPS;
    let started = Instant::now();
    let mut put = 0u32;
    while !stop.load(std::sync::atomic::Ordering::Relaxed) {
        if let Some(wanted) = trial.take() {
            let hold = |r: Option<Range>, v: i64| r.map_or(v, |r| r.hold(v));
            match wanted {
                Wanted::Values(asked) => {
                    if let Some(us) = asked.exposure_us {
                        in_effect.exposure_us =
                            Some(hold(RANGES.exposure_us, i64::from(us)) as u32);
                    }
                    if let Some(gain) = asked.gain {
                        in_effect.gain = Some(hold(RANGES.gain, gain) as i32);
                    }
                }
                Wanted::Configuration => in_effect = configured,
            }
            seen = exposed(&image, scene.brightness, in_effect);
        }
        if let Some(rest) = (period * put).checked_sub(started.elapsed()) {
            thread::sleep(rest);
        }
        let mut frame = latest.spare(1600, 1304);
        frame.pixels.copy_from_slice(&seen.pixels);
        latest.put(Captured {
            image: frame,
            monotonic_nanos: clocks.monotonic_nanos(),
            dequeue_time: false,
            settings: in_effect,
            link: LinkState {
                speed: scene.speed,
                ..LinkState::default()
            },
        });
        put += 1;
    }
}

fn main() {
    let port: u16 = std::env::args()
        .nth(1)
        .map_or(5809, |p| p.parse().expect("a port number"));
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, port)).expect("the port is free");
    println!("http://{}/", listener.local_addr().unwrap());
    let cameras: Vec<Camera> = SCENES
        .iter()
        .enumerate()
        .map(|(i, s)| camera(i, s))
        .collect();
    let views = Arc::new(cameras.iter().map(View::new).collect::<Vec<_>>());
    let latest: Vec<Latest> = cameras.iter().map(|_| Latest::new()).collect();
    let feeds: Vec<Feed> = cameras.iter().map(|_| Feed::new()).collect();
    let clocks = Synced(Instant::now());
    let stop = AtomicBool::new(false);
    let options = Options {
        try_settings: true,
        ..Options::default()
    };
    thread::scope(|scope| {
        let served = Arc::clone(&views);
        let stop = &stop;
        scope.spawn(move || {
            server::run(
                listener,
                served,
                options,
                IpAddr::V4(Ipv4Addr::LOCALHOST),
                stop,
            )
        });
        for (i, camera) in cameras.iter().enumerate() {
            let (latest, feed, view, clocks) = (&latest[i], &feeds[i], &views[i], &clocks);
            let scene = &SCENES[i];
            scope.spawn(move || capture(scene, latest, &view.trial, clocks, stop));
            scope.spawn(move || view::run(camera, feed, latest, view, &RealTime, stop));
            scope.spawn(move || {
                let mut detector = Detector::new(camera.min_tag_px).expect("NAPA");
                // Nowhere to send frames to: there's no robot.
                let mut send = |_: &[u8]| Ok(());
                pipeline::run(camera, latest, &mut detector, clocks, &mut send, feed, stop);
            });
        }
    });
}
