//! The hand-off from a camera's detect thread to its live view: one slot, filled only when the view
//! asks. The detect thread checks a flag after sending each frame, and hands the frame over only
//! when the view has asked for one, so with no one watching it does nothing more than that check.
//! Handing over never waits: if the view holds the slot at that instant, the frame goes back to be
//! recycled, and the next frame is handed over instead.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Condvar, Mutex};
use std::time::Duration;

use crate::brightness::Brightness;
use crate::image::GrayImage;
use crate::settings::InEffect;
use crate::wire::Tag;

/// A frame as detection left it: its image, the tags found in it, and what it was taken at.
#[derive(Debug)]
pub struct Detected {
    pub image: GrayImage,
    pub tags: Vec<Tag>,
    pub settings: InEffect,
    pub brightness: Brightness,
    /// Its number, as sent to the robot.
    pub sequence: u32,
    /// When it was captured, on `CLOCK_MONOTONIC`.
    pub monotonic_nanos: i64,
}

/// The slot, and whether the view wants a frame.
#[derive(Default)]
pub struct Feed {
    wanted: AtomicBool,
    slot: Mutex<Option<Detected>>,
    ready: Condvar,
}

impl Feed {
    pub fn new() -> Feed {
        Feed::default()
    }

    /// For the detect thread: whether the view wants this frame. Saying yes takes back the ask, so
    /// one frame is handed over for each.
    pub fn wants(&self) -> bool {
        // A load first, so a frame nobody watches costs no write.
        self.wanted.load(Ordering::Relaxed) && self.wanted.swap(false, Ordering::AcqRel)
    }

    /// For the detect thread: hands over the frame the view asked for. Never waits. What comes back
    /// is an image to recycle: this frame's, when the view holds the slot at this instant (it's
    /// asked again for the next), or one left in the slot from an ask that ran out of time.
    pub fn give(&self, frame: Detected) -> Option<GrayImage> {
        match self.slot.try_lock() {
            Ok(mut slot) => {
                let left = slot.replace(frame);
                self.ready.notify_one();
                left.map(|frame| frame.image)
            }
            Err(_) => {
                self.wanted.store(true, Ordering::Release);
                Some(frame.image)
            }
        }
    }

    /// For the view: asks for the next frame. What comes back is an image to recycle, from a frame
    /// handed over after an earlier ask ran out of time.
    pub fn ask(&self) -> Option<GrayImage> {
        let left = self.slot.lock().unwrap().take();
        self.wanted.store(true, Ordering::Release);
        left.map(|frame| frame.image)
    }

    /// For the view: the frame it asked for, waiting up to `timeout` for one. None when the camera
    /// sent none in that time, and the ask is taken back.
    pub fn take(&self, timeout: Duration) -> Option<Detected> {
        let slot = self.slot.lock().unwrap();
        let (mut slot, _) = self
            .ready
            .wait_timeout_while(slot, timeout, |slot| slot.is_none())
            .unwrap();
        let taken = slot.take();
        if taken.is_none() {
            self.wanted.store(false, Ordering::Release);
        }
        taken
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;

    fn detected(sequence: u32) -> Detected {
        Detected {
            image: GrayImage::new(4, 2),
            tags: Vec::new(),
            settings: InEffect::default(),
            brightness: Brightness::default(),
            sequence,
            monotonic_nanos: 0,
        }
    }

    #[test]
    fn nothing_is_handed_over_until_the_view_asks_then_one_frame_for_each_ask() {
        let feed = Feed::new();
        assert!(!feed.wants(), "nobody asked");
        assert!(feed.ask().is_none());
        assert!(feed.wants());
        assert!(!feed.wants(), "one frame for one ask");
        assert!(feed.give(detected(7)).is_none());
        assert_eq!(feed.take(Duration::ZERO).unwrap().sequence, 7);
    }

    #[test]
    fn an_ask_that_runs_out_of_time_is_taken_back_and_a_late_frame_recycled() {
        let feed = Feed::new();
        feed.ask();
        assert!(feed.take(Duration::from_millis(5)).is_none());
        assert!(!feed.wants(), "taken back");
        // The detect thread said yes just before the view gave up: its frame waits in the slot,
        // and the next ask hands its image back to recycle.
        feed.ask();
        assert!(feed.wants());
        feed.wanted.store(false, Ordering::Release);
        assert!(feed.give(detected(1)).is_none());
        assert!(feed.ask().is_some(), "left over, to recycle");
        assert!(feed.wants());
    }

    #[test]
    fn handing_over_never_waits_for_the_view() {
        let feed = Feed::new();
        feed.ask();
        assert!(feed.wants());
        let held = feed.slot.lock().unwrap();
        // The view holds the slot this instant: the frame comes straight back, and the next is
        // asked for instead.
        assert!(feed.give(detected(2)).is_some());
        drop(held);
        assert!(feed.wants());
    }

    #[test]
    fn the_view_waits_for_the_frame_it_asked_for() {
        let feed = Arc::new(Feed::new());
        feed.ask();
        let detector = Arc::clone(&feed);
        let handle = thread::spawn(move || {
            while !detector.wants() {
                thread::sleep(Duration::from_millis(1));
            }
            thread::sleep(Duration::from_millis(20));
            detector.give(detected(9));
        });
        let taken = feed
            .take(Duration::from_secs(5))
            .expect("the frame arrives");
        assert_eq!(taken.sequence, 9);
        handle.join().unwrap();
    }
}
