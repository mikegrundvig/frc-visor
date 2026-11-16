//! The hand-off between a camera's capture thread and its detect thread: one slot holding the
//! newest frame. The capture thread replaces whatever is waiting, so a slow detection drops frames
//! rather than falling behind. Images are recycled, so frames don't allocate. The slot counts the
//! frames that never went on: replaced before they were taken, broken, never given by the driver,
//! or compressed and undecodable; and the times the detect thread asked for one, which say it's
//! running ([`crate::watchdog`]).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Condvar, Mutex};
use std::time::Duration;

use crate::image::GrayImage;
use crate::link_state::LinkState;
use crate::settings::InEffect;

/// A frame as the camera gave it.
#[derive(Debug)]
pub struct Captured {
    pub image: GrayImage,
    /// When it was captured, on `CLOCK_MONOTONIC`.
    pub monotonic_nanos: i64,
    /// Whether that's when Visor got it, because the driver gave no timestamp.
    pub dequeue_time: bool,
    /// The exposure and gain it was taken at.
    pub settings: InEffect,
    /// The camera's link and pixel format when it was taken.
    pub link: LinkState,
}

/// The slot.
#[derive(Default)]
pub struct Latest {
    state: Mutex<State>,
    ready: Condvar,
    /// The times the detect thread asked for a frame, since the program started.
    asked: AtomicU64,
}

#[derive(Default)]
struct State {
    waiting: Option<Captured>,
    spares: Vec<GrayImage>,
    /// Since the program started.
    dropped: u64,
    /// Since the detect thread last asked.
    short: ShortFrames,
    /// Since the detect thread last asked.
    withheld: u32,
    /// Since the detect thread last asked.
    undecoded: u32,
}

/// The frames the camera gave too short to use, since the detect thread last asked: counted and
/// reported every ten seconds, not logged one by one.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ShortFrames {
    pub count: u32,
    /// Why the last one was refused.
    pub last: Option<String>,
}

impl Latest {
    pub fn new() -> Latest {
        Latest::default()
    }

    /// An image to fill: a recycled one of the right size, or a new one.
    pub fn spare(&self, width: u32, height: u32) -> GrayImage {
        let mut state = self.state.lock().unwrap();
        while let Some(image) = state.spares.pop() {
            if image.width == width && image.height == height {
                return image;
            }
        }
        GrayImage::new(width, height)
    }

    /// Puts a frame in the slot, replacing (and counting as dropped) one nobody took.
    pub fn put(&self, frame: Captured) {
        let mut state = self.state.lock().unwrap();
        if let Some(old) = state.waiting.replace(frame) {
            state.dropped += 1;
            state.spares.push(old.image);
        }
        self.ready.notify_one();
    }

    /// The newest frame, waiting up to `timeout` for one.
    pub fn take(&self, timeout: Duration) -> Option<Captured> {
        self.asked.fetch_add(1, Ordering::Relaxed);
        let state = self.state.lock().unwrap();
        let (mut state, _) = self
            .ready
            .wait_timeout_while(state, timeout, |state| state.waiting.is_none())
            .unwrap();
        state.waiting.take()
    }

    /// The times the detect thread has asked for a frame ([`Latest::take`]): each frame it took,
    /// and each wait for one that ran out. While it runs, this goes up at least each wait.
    pub fn asked(&self) -> u64 {
        self.asked.load(Ordering::Relaxed)
    }

    /// Counts a frame the camera gave that can't be used: the driver marked it bad, or it was short.
    pub fn broken(&self) {
        self.state.lock().unwrap().dropped += 1;
    }

    /// Counts a frame too short for the image, as broken, and keeps why for the report.
    pub fn short(&self, why: String) {
        let mut state = self.state.lock().unwrap();
        state.dropped += 1;
        state.short.count = state.short.count.saturating_add(1);
        state.short.last = Some(why);
    }

    /// The short frames since the last call, which starts the count again.
    pub fn take_short(&self) -> ShortFrames {
        std::mem::take(&mut self.state.lock().unwrap().short)
    }

    /// Counts `frames` the driver numbered and never gave, as dropped, and keeps how many for the
    /// report. The driver numbers every frame the camera starts and gives only the whole ones: one
    /// the camera sent incomplete or flagged bad it throws away, as it does one it has no buffer
    /// for.
    pub fn withheld(&self, frames: u32) {
        let mut state = self.state.lock().unwrap();
        state.dropped += u64::from(frames);
        state.withheld = state.withheld.saturating_add(frames);
    }

    /// The frames the driver never gave since the last call, which starts the count again.
    pub fn take_withheld(&self) -> u32 {
        std::mem::take(&mut self.state.lock().unwrap().withheld)
    }

    /// Counts a compressed frame that didn't decode, as dropped, and keeps how many for the report:
    /// the compressed counterpart of a frame the driver never gave, since the driver passes a
    /// compressed frame on whole or not.
    pub fn undecoded(&self) {
        let mut state = self.state.lock().unwrap();
        state.dropped += 1;
        state.undecoded = state.undecoded.saturating_add(1);
    }

    /// The frames that didn't decode since the last call, which starts the count again.
    pub fn take_undecoded(&self) -> u32 {
        std::mem::take(&mut self.state.lock().unwrap().undecoded)
    }

    /// Gives back a taken frame's image for reuse.
    pub fn recycle(&self, image: GrayImage) {
        let mut state = self.state.lock().unwrap();
        // Two spares cover one in the camera's hands and one waiting; the third, one in the live
        // view's.
        if state.spares.len() < 3 {
            state.spares.push(image);
        }
    }

    /// How many frames were dropped since the program started: replaced before they were taken,
    /// broken, or never given by the driver.
    pub fn dropped(&self) -> u64 {
        self.state.lock().unwrap().dropped
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;

    fn frame(latest: &Latest, at: i64) -> Captured {
        Captured {
            image: latest.spare(4, 2),
            monotonic_nanos: at,
            dequeue_time: false,
            settings: InEffect::default(),
            link: LinkState::default(),
        }
    }

    #[test]
    fn the_newest_frame_wins_and_the_rest_count_as_dropped() {
        let latest = Latest::new();
        for at in [1, 2, 3] {
            latest.put(frame(&latest, at));
        }
        let taken = latest.take(Duration::ZERO).unwrap();
        assert_eq!(taken.monotonic_nanos, 3);
        assert_eq!(latest.dropped(), 2);
        assert!(latest.take(Duration::ZERO).is_none());
        // A broken frame counts too, and the count runs on from the start.
        latest.broken();
        latest.put(frame(&latest, 4));
        latest.take(Duration::ZERO).unwrap();
        assert_eq!(latest.dropped(), 3);
        // So does a short one, which is also kept for the report until it's taken.
        latest.short("too small".into());
        latest.short("too small again".into());
        assert_eq!(latest.dropped(), 5);
        assert_eq!(
            latest.take_short(),
            ShortFrames {
                count: 2,
                last: Some("too small again".into())
            }
        );
        assert_eq!(latest.take_short(), ShortFrames::default());
        assert_eq!(latest.dropped(), 5);
        // So do frames the driver never gave, kept for the report the same way.
        latest.withheld(2);
        latest.withheld(1);
        assert_eq!(latest.dropped(), 8);
        assert_eq!(latest.take_withheld(), 3);
        assert_eq!(latest.take_withheld(), 0);
        assert_eq!(latest.dropped(), 8);
        // And compressed frames that didn't decode.
        latest.undecoded();
        latest.undecoded();
        assert_eq!(latest.dropped(), 10);
        assert_eq!(latest.take_undecoded(), 2);
        assert_eq!(latest.take_undecoded(), 0);
    }

    #[test]
    fn images_are_recycled_by_size() {
        let latest = Latest::new();
        let mut image = latest.spare(4, 2);
        image.pixels[0] = 9;
        latest.recycle(image);
        assert_eq!(latest.spare(4, 2).pixels[0], 9, "the recycled image");
        latest.recycle(GrayImage::new(8, 8));
        assert_eq!(
            latest.spare(4, 2).pixels.len(),
            8,
            "a new one: the spare was the wrong size"
        );
    }

    #[test]
    fn take_waits_for_a_frame() {
        let latest = Arc::new(Latest::new());
        let putter = Arc::clone(&latest);
        let handle = thread::spawn(move || {
            thread::sleep(Duration::from_millis(20));
            putter.put(frame(&putter, 7));
        });
        let taken = latest
            .take(Duration::from_secs(5))
            .expect("a frame arrives");
        assert_eq!(taken.monotonic_nanos, 7);
        handle.join().unwrap();
        assert!(
            latest.take(Duration::from_millis(10)).is_none(),
            "then times out"
        );
    }
}
