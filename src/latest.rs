//! The hand-off between a camera's capture thread and its detect thread: one slot holding the
//! newest frame. The capture thread replaces whatever is waiting, so a slow detection drops frames
//! rather than falling behind. Images are recycled, so frames don't allocate.

use std::sync::{Condvar, Mutex};
use std::time::Duration;

use crate::image::GrayImage;

/// A frame as the camera gave it.
#[derive(Debug)]
pub struct Captured {
    pub image: GrayImage,
    /// When it was captured, on `CLOCK_MONOTONIC`.
    pub monotonic_nanos: i64,
    /// Whether that's when Visor got it, because the driver gave no timestamp.
    pub dequeue_time: bool,
}

/// The slot.
#[derive(Default)]
pub struct Latest {
    state: Mutex<State>,
    ready: Condvar,
}

#[derive(Default)]
struct State {
    waiting: Option<Captured>,
    spares: Vec<GrayImage>,
    dropped: u64,
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
        let state = self.state.lock().unwrap();
        let (mut state, _) = self
            .ready
            .wait_timeout_while(state, timeout, |state| state.waiting.is_none())
            .unwrap();
        state.waiting.take()
    }

    /// Gives back a taken frame's image for reuse.
    pub fn recycle(&self, image: GrayImage) {
        let mut state = self.state.lock().unwrap();
        // Two spares cover one in the camera's hands and one waiting.
        if state.spares.len() < 2 {
            state.spares.push(image);
        }
    }

    /// How many frames were dropped since the last call.
    pub fn take_dropped(&self) -> u64 {
        std::mem::take(&mut self.state.lock().unwrap().dropped)
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
        assert_eq!(latest.take_dropped(), 2);
        assert_eq!(latest.take_dropped(), 0);
        assert!(latest.take(Duration::ZERO).is_none());
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
