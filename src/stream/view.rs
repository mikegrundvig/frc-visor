//! A camera's live view: its thread, which makes a picture at the view's rate while anyone watches
//! and nothing at all while no one does, and the latest picture, which each watcher's connection
//! takes as it's ready for the next. The same thread makes a frame at full size when one is asked
//! for ([`Still`]): the pixels detection read, and what it found in them. Its thread runs at a lower
//! priority than capture and detection, so on a busy board the view slows, never detection.

use std::collections::VecDeque;
use std::fmt::Write as _;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use axum::body::Bytes;
use log::warn;
use tokio::sync::watch;

use crate::config::Camera;
use crate::latest::Latest;
use crate::settings::Trial;
use crate::stream::feed::{Detected, Feed};
use crate::stream::picture::{self, Painter};

/// How often an unwatched view looks for a watcher.
const IDLE_CHECK: Duration = Duration::from_millis(100);

/// How long the view waits for a frame before asking again: a camera that's down sends none.
const FRAME_WAIT: Duration = Duration::from_millis(500);

/// One camera's latest picture, and its watchers.
pub struct View {
    /// The camera's name, as its address has it.
    pub name: String,
    pictures: watch::Sender<Option<Bytes>>,
    made: AtomicU64,
    stills: watch::Sender<Option<Arc<Still>>>,
    still_asked: AtomicBool,
    stills_made: AtomicU64,
    /// An exposure and gain asked for from the live view, which the camera's capture thread sets.
    pub trial: Trial,
}

/// One frame at full size, as detection had it: not made smaller, not compressed, nothing drawn on
/// it.
#[derive(Debug)]
pub struct Still {
    /// The frame as a binary PGM (`P5`, 8-bit), which `tools/calibrate.py` and `visor detect` read.
    pub pgm: Bytes,
    /// What it was taken at and what Visor found in it, as one line of JSON in ASCII
    /// (docs/live-view.md, "A frame at full size").
    pub about: String,
}

impl Still {
    fn of(camera: &Camera, frame: &Detected, focus: f64) -> Still {
        let image = &frame.image;
        let mut pgm = format!("P5\n{} {}\n255\n", image.width, image.height).into_bytes();
        pgm.extend_from_slice(&image.pixels);
        Still {
            pgm: Bytes::from(pgm),
            about: about(camera, frame, focus),
        }
    }
}

/// A frame's facts as JSON: the camera, the frame's number as sent to the robot, its size, the
/// exposure and gain in effect, its brightness and focus, and each tag with its corners.
fn about(camera: &Camera, frame: &Detected, focus: f64) -> String {
    let optional = |value: Option<String>| value.unwrap_or_else(|| "null".to_string());
    let settings = frame.settings;
    let mut json = String::from("{\"camera\":");
    json_text(&mut json, &camera.name);
    let _ = write!(
        json,
        ",\"sequence\":{},\"width\":{},\"height\":{},\"exposure_us\":{},\"gain\":{},\
         \"auto_exposure\":{},\"brightness_median\":{},\"brightness_high\":{},\
         \"clipped_percent\":{},\"focus\":{},\"tags\":[",
        frame.sequence,
        frame.image.width,
        frame.image.height,
        optional(settings.exposure_us.map(|us| us.to_string())),
        optional(settings.gain.map(|gain| gain.to_string())),
        optional(settings.auto_exposure.map(|auto| auto.to_string())),
        frame.brightness.median,
        frame.brightness.high,
        f64::from(frame.brightness.clipped) / 100.0,
        number(focus),
    );
    for (i, tag) in frame.tags.iter().enumerate() {
        let corners: Vec<String> = tag.corners.iter().map(|c| number(f64::from(*c))).collect();
        let (distance, error) = match tag.poses {
            Some([best, _]) => {
                let [x, y, z] = best.tvec.map(f64::from);
                (
                    number((x * x + y * y + z * z).sqrt()),
                    number(f64::from(best.error_px)),
                )
            }
            None => ("null".to_string(), "null".to_string()),
        };
        let _ = write!(
            json,
            "{}{{\"id\":{},\"hamming\":{},\"decision_margin\":{},\"corners\":[{}],\
             \"distance_m\":{distance},\"pose_error_px\":{error}}}",
            if i == 0 { "" } else { "," },
            tag.id,
            tag.hamming,
            number(f64::from(tag.decision_margin)),
            corners.join(","),
        );
    }
    json.push_str("]}");
    json
}

/// A number as JSON has it, to a thousandth; one that isn't finite is `null`.
fn number(value: f64) -> String {
    if value.is_finite() {
        format!("{:.3}", value)
    } else {
        "null".to_string()
    }
}

/// Text as a JSON string in ASCII alone, so it can go in an HTTP header: everything else escaped.
fn json_text(json: &mut String, text: &str) {
    json.push('"');
    for unit in text.encode_utf16() {
        match char::from_u32(u32::from(unit)) {
            Some('"') => json.push_str("\\\""),
            Some('\\') => json.push_str("\\\\"),
            Some(c) if c.is_ascii() && !c.is_ascii_control() => json.push(c),
            _ => {
                let _ = write!(json, "\\u{unit:04x}");
            }
        }
    }
    json.push('"');
}

impl View {
    pub fn new(name: &str) -> View {
        View {
            name: name.to_string(),
            pictures: watch::Sender::new(None),
            made: AtomicU64::new(0),
            stills: watch::Sender::new(None),
            still_asked: AtomicBool::new(false),
            stills_made: AtomicU64::new(0),
            trial: Trial::default(),
        }
    }

    /// How many pictures it has made since Visor started: none while no one watches.
    pub fn made(&self) -> u64 {
        self.made.load(Ordering::Relaxed)
    }

    /// A new watcher: it sees each picture made from now on, the newest when it's ready for one.
    pub fn watch(&self) -> watch::Receiver<Option<Bytes>> {
        self.pictures.subscribe()
    }

    /// How many watch now.
    pub fn watchers(&self) -> usize {
        self.pictures.receiver_count()
    }

    fn show(&self, picture: Option<Bytes>) {
        self.pictures.send_replace(picture);
    }

    /// Asks for the camera's next frame at full size. `changed()` on what comes back waits for it;
    /// say when it's had, or given up on, with [`View::forget_still`].
    pub fn ask_still(&self) -> watch::Receiver<Option<Arc<Still>>> {
        let stills = self.stills.subscribe();
        self.still_asked.store(true, Ordering::Release);
        stills
    }

    /// Takes back an ask, and lets go of the frame made for it: two megabytes for a ThriftyCam.
    pub fn forget_still(&self) {
        self.still_asked.store(false, Ordering::Release);
        self.stills.send_replace(None);
    }

    /// How many full-size frames it has made since Visor started: none unless asked.
    pub fn stills_made(&self) -> u64 {
        self.stills_made.load(Ordering::Relaxed)
    }
}

/// The time the view paces its pictures by: the real one ([`RealTime`]), or a test's.
pub trait Pace {
    fn now(&self) -> Instant;
    fn sleep(&self, how_long: Duration);
}

/// The real time.
pub struct RealTime;

impl Pace for RealTime {
    fn now(&self) -> Instant {
        Instant::now()
    }

    fn sleep(&self, how_long: Duration) {
        thread::sleep(how_long);
    }
}

/// Makes pictures from the camera's frames while anyone watches, and a frame at full size when one
/// is asked for, until `stop` is set: one each period of the view's rate by `pace`. Each frame's image goes back to `latest` once its picture
/// is made.
pub fn run(
    camera: &Camera,
    feed: &Feed,
    latest: &Latest,
    view: &View,
    pace: &impl Pace,
    stop: &AtomicBool,
) {
    super::lower_priority(&format!("{}'s live view", camera.name));
    let period = Duration::from_secs_f64(1.0 / f64::from(camera.stream.fps));
    let mut painter = Painter::new(camera.stream);
    let mut rate = Rate::default();
    let mut said = false;
    while !stop.load(Ordering::Relaxed) {
        let watched = view.watchers() > 0;
        if !watched && rate.started() {
            // Its last picture is no one's now; a new watcher waits for a fresh one.
            view.show(None);
            rate = Rate::default();
        }
        if !watched && !view.still_asked.load(Ordering::Acquire) {
            thread::sleep(IDLE_CHECK);
            continue;
        }
        let started = pace.now();
        if let Some(left) = feed.ask() {
            latest.recycle(left);
        }
        let Some(frame) = feed.take(FRAME_WAIT) else {
            continue;
        };
        let focus = picture::focus(&frame.image);
        // Asked for, and still waited for: an ask given up on makes nothing.
        if view.still_asked.swap(false, Ordering::AcqRel) && view.stills.receiver_count() > 0 {
            let still = Still::of(camera, &frame, focus);
            view.stills_made.fetch_add(1, Ordering::Relaxed);
            view.stills.send_replace(Some(Arc::new(still)));
        }
        let painted = watched.then(|| {
            rate.add(frame.sequence, frame.monotonic_nanos);
            painter.paint(&frame, rate.fps(), focus)
        });
        latest.recycle(frame.image);
        match painted {
            Some(Ok(jpeg)) => {
                view.made.fetch_add(1, Ordering::Relaxed);
                view.show(Some(Bytes::from(jpeg)));
            }
            Some(Err(why)) if !said => {
                warn!("{}: the live view's picture: {why}", camera.name);
                said = true;
            }
            _ => {}
        }
        // The next a period after this one began; when this one took longer, straight away.
        if let Some(rest) = period.checked_sub(pace.now() - started) {
            pace.sleep(rest);
        }
    }
}

/// Detection's frame rate, from the frames' numbers and capture times over about the last second.
#[derive(Default)]
struct Rate {
    seen: VecDeque<(u32, i64)>,
}

/// The span the rate is measured over.
const RATE_OVER_NANOS: i64 = 1_000_000_000;

impl Rate {
    fn started(&self) -> bool {
        !self.seen.is_empty()
    }

    fn add(&mut self, sequence: u32, monotonic_nanos: i64) {
        if let Some(&(last, _)) = self.seen.back()
            && sequence < last
        {
            self.seen.clear(); // Visor's numbers don't go back: a test's frames, or a wrap
        }
        self.seen.push_back((sequence, monotonic_nanos));
        while self.seen.len() > 2 && monotonic_nanos - self.seen[1].1 >= RATE_OVER_NANOS {
            self.seen.pop_front();
        }
    }

    /// Frames detected per second, once two pictures have been made.
    fn fps(&self) -> Option<f64> {
        let (&(first, from), &(last, to)) = (self.seen.front()?, self.seen.back()?);
        if to <= from {
            return None;
        }
        Some(f64::from(last - first) * 1e9 / (to - from) as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_rate_is_detection_s_over_about_the_last_second() {
        let mut rate = Rate::default();
        assert_eq!(rate.fps(), None);
        // Pictures at 10 a second of a camera detecting 60.
        for picture in 0..30 {
            rate.add(picture * 6, i64::from(picture) * 100_000_000);
            if picture == 0 {
                assert_eq!(rate.fps(), None, "one picture says no rate");
            }
        }
        assert!((rate.fps().unwrap() - 60.0).abs() < 1e-9);
        assert!(rate.seen.len() <= 12, "about a second: {}", rate.seen.len());
        // Detection slows to 30.
        for picture in 30..60 {
            rate.add(180 + (picture - 30) * 3, i64::from(picture) * 100_000_000);
        }
        assert!((rate.fps().unwrap() - 30.0).abs() < 1e-9);
    }

    #[test]
    fn a_still_is_the_frame_s_pixels_as_a_pgm_and_its_facts_as_ascii_json() {
        use crate::brightness::Brightness;
        use crate::image::GrayImage;
        use crate::settings::InEffect;
        use crate::wire::{Pose, Tag};

        let camera = Camera {
            name: "front \"lef\u{e9}\"".into(),
            index: 0,
            device: "/dev/null".into(),
            width: 3,
            height: 2,
            fps: 60,
            format: None,
            min_tag_px: 16,
            time_offset_nanos: 0,
            exposure_us: Some(1000),
            gain: None,
            controls: Vec::new(),
            lens: None,
            tag_size_m: 0.1651,
            stream: crate::config::Stream::default(),
            config_id: 0,
        };
        let mut image = GrayImage::new(3, 2);
        image.pixels.copy_from_slice(&[1, 2, 3, 4, 5, 6]);
        let solved = Pose {
            rvec: [0.0; 3],
            tvec: [0.0, 3.0, 4.0],
            error_px: 0.25,
        };
        let frame = Detected {
            image,
            tags: vec![
                Tag {
                    id: 7,
                    hamming: 1,
                    decision_margin: 81.5,
                    corners: [10.0, 20.5, 30.0, 20.5, 30.0, 40.0, 10.0, 40.0],
                    poses: Some([solved, solved]),
                },
                Tag {
                    id: 9,
                    hamming: 0,
                    decision_margin: f32::NAN,
                    corners: [0.0; 8],
                    poses: None,
                },
            ],
            settings: InEffect {
                exposure_us: Some(1000),
                gain: None,
                auto_exposure: Some(false),
            },
            brightness: Brightness {
                median: 37,
                high: 201,
                clipped: 125,
            },
            sequence: 42,
            monotonic_nanos: 0,
        };
        let still = Still::of(&camera, &frame, 12.3456);
        assert_eq!(&still.pgm[..], b"P5\n3 2\n255\n\x01\x02\x03\x04\x05\x06");
        assert!(still.about.is_ascii(), "{}", still.about);
        assert!(!still.about.contains('\n'));
        let about: serde_json::Value = serde_json::from_str(&still.about).unwrap();
        assert_eq!(about["camera"], "front \"lef\u{e9}\"");
        assert_eq!(about["sequence"], 42);
        assert_eq!((&about["width"], &about["height"]), (&3.into(), &2.into()));
        assert_eq!(about["exposure_us"], 1000);
        assert!(about["gain"].is_null(), "a camera with no gain control");
        assert_eq!(about["auto_exposure"], false);
        assert_eq!(about["brightness_median"], 37);
        assert_eq!(about["brightness_high"], 201);
        assert_eq!(about["clipped_percent"], 1.25);
        assert_eq!(about["focus"], 12.346);
        let tags = about["tags"].as_array().unwrap();
        assert_eq!(tags.len(), 2);
        assert_eq!(tags[0]["id"], 7);
        assert_eq!(tags[0]["hamming"], 1);
        assert_eq!(tags[0]["decision_margin"], 81.5);
        assert_eq!(tags[0]["corners"][1], 20.5);
        assert_eq!(tags[0]["corners"].as_array().unwrap().len(), 8);
        assert_eq!(tags[0]["distance_m"], 5.0);
        assert_eq!(tags[0]["pose_error_px"], 0.25);
        // No lens, so no pose; and a number that isn't one is null, never invalid JSON.
        assert!(tags[1]["distance_m"].is_null());
        assert!(tags[1]["decision_margin"].is_null());
    }

    #[test]
    fn a_still_is_made_only_while_someone_waits_for_it() {
        let view = View::new("front");
        assert!(!view.still_asked.load(Ordering::Acquire));
        let waiting = view.ask_still();
        assert!(view.still_asked.load(Ordering::Acquire));
        assert_eq!(view.stills.receiver_count(), 1);
        drop(waiting);
        view.forget_still();
        assert!(!view.still_asked.load(Ordering::Acquire));
        assert_eq!(view.stills.receiver_count(), 0);
        assert!(view.stills.borrow().is_none());
    }

    #[test]
    fn a_view_counts_its_watchers() {
        let view = View::new("front");
        assert_eq!(view.watchers(), 0);
        let one = view.watch();
        let two = view.watch();
        assert_eq!(view.watchers(), 2);
        drop(one);
        assert_eq!(view.watchers(), 1);
        drop(two);
        assert_eq!(view.watchers(), 0);
    }
}
