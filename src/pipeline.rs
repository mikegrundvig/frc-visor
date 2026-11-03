//! A camera's detect thread: takes the newest frame, finds its tags, stamps it with robot time and
//! sends it. Every frame is sent, with or without tags, so silence means the camera is down.

use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use log::{info, warn};

use crate::clock::{Clocks, monotonic_to_robot};
use crate::config::Camera;
use crate::latest::Latest;
use crate::lens::Lens;
use crate::napa::{Detection, Detector};
use crate::pose::{TagPose, solve_square};
use crate::wire::{self, Frame, MAX_TAGS, Pose, Tag};

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
            Ok(found) => tags_of(found, camera.lens.as_ref(), camera.tag_size_m),
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

/// NAPA's detections as the wire carries them, each with its two poses when the camera has a lens
/// calibration. If there are ever more than a frame holds, the clearest are kept.
fn tags_of(found: &[Detection], lens: Option<&Lens>, tag_size_m: f64) -> Vec<Tag> {
    let mut kept: Vec<&Detection> = found.iter().collect();
    if kept.len() > MAX_TAGS {
        kept.sort_by(|a, b| b.decision_margin.total_cmp(&a.decision_margin));
        kept.truncate(MAX_TAGS);
    }
    kept.into_iter()
        .map(|d| Tag {
            id: d.id as u16,
            hamming: d.hamming as u8,
            decision_margin: d.decision_margin,
            corners: d.corners.map(|v| v as f32),
            poses: lens
                .and_then(|lens| solve_square(&d.corners, lens, tag_size_m))
                .map(|poses| poses.map(|p| wire_pose(&p))),
        })
        .collect()
}

fn wire_pose(pose: &TagPose) -> Pose {
    Pose {
        rvec: [pose.rvec.x as f32, pose.rvec.y as f32, pose.rvec.z as f32],
        tvec: [pose.tvec.x as f32, pose.tvec.y as f32, pose.tvec.z as f32],
        error_px: pose.error_px as f32,
    }
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
    fn detections_become_wire_tags_with_poses_only_given_a_lens() {
        let lens = Lens::new(1000.0, 1000.0, 800.0, 650.0, &[]).unwrap();
        // A 0.2 m tag 2 m straight ahead, square to the camera: 100 px across, centred.
        let mut seen = detection(586, 40.0);
        seen.corners = [750.0, 600.0, 850.0, 600.0, 850.0, 700.0, 750.0, 700.0];
        let without = tags_of(&[seen], None, 0.2);
        assert_eq!(
            without,
            [Tag {
                id: 586,
                hamming: 1,
                decision_margin: 40.0,
                corners: [750.0, 600.0, 850.0, 600.0, 850.0, 700.0, 750.0, 700.0],
                poses: None,
            }]
        );
        let with = tags_of(&[seen], Some(&lens), 0.2);
        let [best, _] = with[0].poses.expect("solved");
        let [x, y, z] = best.tvec;
        assert!(
            x.abs() < 1e-4 && y.abs() < 1e-4 && (z - 2.0).abs() < 1e-4,
            "{:?}",
            best.tvec
        );
        // A degenerate square is sent with its corners alone.
        assert_eq!(
            tags_of(&[detection(1, 9.0)], Some(&lens), 0.2)[0].poses,
            None
        );
    }

    #[test]
    fn past_a_frame_s_room_the_clearest_tags_are_kept() {
        let found: Vec<Detection> = (0..40).map(|i| detection(i, i as f32)).collect();
        let tags = tags_of(&found, None, 0.1651);
        assert_eq!(tags.len(), MAX_TAGS);
        assert!(tags.iter().all(|t| t.decision_margin >= 8.0));
    }
}
