//! A camera's live view: its thread, which makes a picture at the view's rate while anyone watches
//! and nothing at all while no one does, and the latest picture, which each watcher's connection
//! takes as it's ready for the next. Its thread runs at a lower priority than capture and detection,
//! so on a busy board the view slows, never detection.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use axum::body::Bytes;
use log::warn;
use tokio::sync::watch;

use crate::config::Camera;
use crate::latest::Latest;
use crate::stream::feed::Feed;
use crate::stream::picture::Painter;

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
}

impl View {
    pub fn new(name: &str) -> View {
        View {
            name: name.to_string(),
            pictures: watch::Sender::new(None),
            made: AtomicU64::new(0),
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

/// Makes pictures from the camera's frames while anyone watches, until `stop` is set, one each
/// period of the view's rate by `pace`. Each frame's image goes back to `latest` once its picture
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
        if view.watchers() == 0 {
            if rate.started() {
                // Its last picture is no one's now; a new watcher waits for a fresh one.
                view.show(None);
                rate = Rate::default();
            }
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
        rate.add(frame.sequence, frame.monotonic_nanos);
        let painted = painter.paint(&frame, rate.fps());
        latest.recycle(frame.image);
        match painted {
            Ok(jpeg) => {
                view.made.fetch_add(1, Ordering::Relaxed);
                view.show(Some(Bytes::from(jpeg)));
            }
            Err(why) if !said => {
                warn!("{}: the live view's picture: {why}", camera.name);
                said = true;
            }
            Err(_) => {}
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
