//! A camera's detect thread: takes the newest frame, finds its tags, stamps it with robot time and
//! sends it. Every frame is sent, with or without tags, so silence means the camera is down.

use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use log::{info, warn};

use crate::clock::{Clocks, monotonic_to_robot};
use crate::config::Camera;
use crate::latest::Latest;
use crate::napa::{Detection, Detector};
use crate::wire::{self, Frame, MAX_TAGS, Tag};

/// How often each camera logs its numbers.
const REPORT_EVERY: Duration = Duration::from_secs(10);

/// Detects and sends until `stop` is set. `send` gets each frame's bytes.
pub fn run(
    camera: &Camera,
    latest: &Latest,
    detector: &mut Detector,
    clocks: &impl Clocks,
    send: &mut impl FnMut(&[u8]) -> io::Result<()>,
    stop: &AtomicBool,
) {
    let mut sequence: u32 = 0;
    let mut bytes = Vec::with_capacity(wire::HEADER_BYTES + MAX_TAGS * wire::TAG_BYTES);
    let mut numbers = Numbers::new();
    while !stop.load(Ordering::Relaxed) {
        if numbers.started.elapsed() >= REPORT_EVERY {
            numbers.report(&camera.name, latest.take_dropped());
            numbers = Numbers::new();
        }
        let Some(captured) = latest.take(Duration::from_millis(100)) else {
            continue;
        };
        let started = Instant::now();
        let tags = match detector.detect(&captured.image) {
            Ok(found) => tags_of(found),
            Err(error) => {
                warn!("{}: {error}", camera.name);
                Vec::new()
            }
        };
        let detect_time = started.elapsed();
        let frame = Frame {
            version_id: wire::VERSION_ID,
            sequence,
            width: captured.image.width as u16,
            height: captured.image.height as u16,
            capture_nanos: captured.monotonic_nanos
                + monotonic_to_robot(clocks)
                + camera.time_offset_nanos,
            camera: camera.index,
            clock_synced: clocks.synced(),
            dequeue_time: captured.dequeue_time,
            tags,
        };
        latest.recycle(captured.image);
        frame.encode(&mut bytes);
        let sent = send(&bytes).is_ok();
        numbers.add(detect_time, frame.tags.len(), sent, frame.clock_synced);
        sequence = sequence.wrapping_add(1);
    }
}

/// NAPA's detections as the wire carries them. If there are ever more than a frame holds, the
/// clearest are kept.
fn tags_of(found: &[Detection]) -> Vec<Tag> {
    let mut tags: Vec<Tag> = found
        .iter()
        .map(|d| Tag {
            id: d.id as u16,
            hamming: d.hamming as u8,
            decision_margin: d.decision_margin,
            corners: d.corners.map(|v| v as f32),
        })
        .collect();
    if tags.len() > MAX_TAGS {
        tags.sort_by(|a, b| b.decision_margin.total_cmp(&a.decision_margin));
        tags.truncate(MAX_TAGS);
    }
    tags
}

/// What a camera logs every ten seconds.
struct Numbers {
    started: Instant,
    frames: u32,
    tags: usize,
    detect_total: Duration,
    detect_worst: Duration,
    send_failures: u32,
    unsynced: u32,
}

impl Numbers {
    fn new() -> Numbers {
        Numbers {
            started: Instant::now(),
            frames: 0,
            tags: 0,
            detect_total: Duration::ZERO,
            detect_worst: Duration::ZERO,
            send_failures: 0,
            unsynced: 0,
        }
    }

    fn add(&mut self, detect_time: Duration, tags: usize, sent: bool, synced: bool) {
        self.frames += 1;
        self.tags += tags;
        self.detect_total += detect_time;
        self.detect_worst = self.detect_worst.max(detect_time);
        self.send_failures += u32::from(!sent);
        self.unsynced += u32::from(!synced);
    }

    fn report(&self, camera: &str, dropped: u64) {
        let seconds = self.started.elapsed().as_secs_f64();
        if self.frames == 0 {
            warn!("{camera}: no frames for {seconds:.0} s");
            return;
        }
        let frames = f64::from(self.frames);
        info!(
            "{camera}: {:.1} fps, {dropped} dropped, detect {:.1} ms mean, {:.1} ms worst, \
             {:.2} tags per frame",
            frames / seconds,
            self.detect_total.as_secs_f64() * 1000.0 / frames,
            self.detect_worst.as_secs_f64() * 1000.0,
            self.tags as f64 / frames,
        );
        if self.send_failures > 0 {
            warn!(
                "{camera}: {} frames not sent (is the robot program running?)",
                self.send_failures
            );
        }
        if self.unsynced > 0 {
            warn!(
                "{camera}: {} frames with the clock not synced to the robot (chrony)",
                self.unsynced
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn detection(id: i32, margin: f32) -> Detection {
        Detection {
            id,
            hamming: 1,
            decision_margin: margin,
            corners: [0.25; 8],
        }
    }

    #[test]
    fn detections_become_wire_tags() {
        let tags = tags_of(&[detection(586, 40.0)]);
        assert_eq!(
            tags,
            [Tag {
                id: 586,
                hamming: 1,
                decision_margin: 40.0,
                corners: [0.25; 8]
            }]
        );
    }

    #[test]
    fn past_a_frame_s_room_the_clearest_tags_are_kept() {
        let found: Vec<Detection> = (0..40).map(|i| detection(i, i as f32)).collect();
        let tags = tags_of(&found);
        assert_eq!(tags.len(), MAX_TAGS);
        assert!(tags.iter().all(|t| t.decision_margin >= 8.0));
    }
}
