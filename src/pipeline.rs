//! A camera's detect thread: takes the newest frame, finds its tags, measures its brightness, stamps
//! it with robot time and sends it, with the exposure and gain it was taken at. Every frame is sent,
//! with or without tags, so silence means the camera is down. Each frame also says how long its
//! detection took and how many of the camera's frames were dropped so far, from which the robot
//! works out the camera's frame rate, drops and load.

use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use log::{info, warn};

use crate::brightness::{self, Brightness};
use crate::clock::{Clocks, monotonic_to_robot};
use crate::config::Camera;
use crate::latest::Latest;
use crate::lens::Lens;
use crate::napa::{Detection, Detector};
use crate::pose::{TagPose, solve_square};
use crate::settings::InEffect;
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
    let mut numbers = Numbers::new(latest.dropped());
    while !stop.load(Ordering::Relaxed) {
        if numbers.started.elapsed() >= REPORT_EVERY {
            let dropped = latest.dropped();
            numbers.report(&camera.name, dropped - numbers.dropped_before);
            numbers = Numbers::new(dropped);
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
        let brightness = Brightness::of(&captured.image);
        let settings = captured.settings;
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
            exposure_us: settings.exposure_us,
            gain: settings.gain,
            auto_exposure: settings.auto_exposure == Some(true),
            brightness,
            detect_us: u32::try_from(detect_time.as_micros()).unwrap_or(u32::MAX),
            // Wraps, as the robot reads it: only the difference between two frames' counts matters.
            dropped: latest.dropped() as u32,
            tags,
        };
        latest.recycle(captured.image);
        frame.encode(&mut bytes);
        let sent = send(&bytes).is_ok();
        numbers.add(detect_time, frame.tags.len(), sent, frame.clock_synced);
        numbers.light(brightness, settings);
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

/// The exposure and gain as a warning names them.
fn settings_shown(settings: InEffect) -> String {
    let exposure = match (settings.auto_exposure, settings.exposure_us) {
        (Some(true), _) => "the camera's automatic exposure".to_string(),
        (_, Some(us)) => format!("exposure {:.1} ms", f64::from(us) / 1000.0),
        (_, None) => "an unknown exposure".to_string(),
    };
    match settings.gain {
        Some(gain) => format!("{exposure} and gain {gain}"),
        None => exposure,
    }
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
    /// The camera's dropped frames when these numbers started.
    dropped_before: u64,
    frames: u32,
    tags: usize,
    detect_total: Duration,
    detect_worst: Duration,
    send_failures: u32,
    unsynced: u32,
    dark: u32,
    clipping: u32,
    /// The last frame's, for the report.
    brightness: Brightness,
    settings: InEffect,
}

impl Numbers {
    fn new(dropped_before: u64) -> Numbers {
        Numbers {
            started: Instant::now(),
            dropped_before,
            frames: 0,
            tags: 0,
            detect_total: Duration::ZERO,
            detect_worst: Duration::ZERO,
            send_failures: 0,
            unsynced: 0,
            dark: 0,
            clipping: 0,
            brightness: Brightness::default(),
            settings: InEffect::default(),
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

    /// A frame's brightness, and the exposure and gain it was taken at.
    fn light(&mut self, brightness: Brightness, settings: InEffect) {
        self.dark += u32::from(brightness.is_dark());
        self.clipping += u32::from(brightness.clips());
        self.brightness = brightness;
        self.settings = settings;
    }

    /// What's wrong with the light, if most frames since the last report were too dark or had
    /// their whites clipping: each a warning naming what to change.
    fn light_warnings(&self, camera: &str) -> Vec<String> {
        let mut warnings = Vec::new();
        let at = settings_shown(self.settings);
        if self.frames > 0 && self.dark * 2 > self.frames {
            warnings.push(format!(
                "{camera}: {} of {} frames too dark to read tags far (the brightest 1% under {} \
                 of 255; median {}), at {at}: lengthen the exposure, raise the gain, or add light",
                self.dark,
                self.frames,
                brightness::DARK_BELOW,
                self.brightness.median
            ));
        }
        if self.frames > 0 && self.clipping * 2 > self.frames {
            warnings.push(format!(
                "{camera}: {} of {} frames with whites clipping ({:.1}% of the frame at full \
                 white, over {}%), at {at}: shorten the exposure or lower the gain, or turn the \
                 camera from a bright light",
                self.clipping,
                self.frames,
                f64::from(self.brightness.clipped) / 100.0,
                f64::from(brightness::CLIPPED_ABOVE) / 100.0
            ));
        }
        warnings
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
             {:.2} tags per frame, brightness {} median and {} at 99%",
            frames / seconds,
            self.detect_total.as_secs_f64() * 1000.0 / frames,
            self.detect_worst.as_secs_f64() * 1000.0,
            self.tags as f64 / frames,
            self.brightness.median,
            self.brightness.high,
        );
        for warning in self.light_warnings(camera) {
            warn!("{warning}");
        }
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
    fn most_frames_too_dark_or_clipping_say_what_to_change() {
        let fixed = InEffect {
            exposure_us: Some(1000),
            gain: Some(40),
            auto_exposure: Some(false),
        };
        let dark = Brightness {
            median: 8,
            high: 30,
            clipped: 0,
        };
        let lit = Brightness {
            median: 60,
            high: 190,
            clipped: 0,
        };
        let clipping = Brightness {
            median: 140,
            high: 255,
            clipped: 1250,
        };
        // The frames since the last report, and part of each warning.
        let cases: [(&str, Vec<Brightness>, InEffect, Vec<&str>); 5] = [
            ("lit", vec![lit; 10], fixed, vec![]),
            (
                "dark",
                vec![dark; 10],
                fixed,
                vec![
                    "10 of 10 frames too dark to read tags far (the brightest 1% under 64 of 255; \
                     median 8), at exposure 1.0 ms and gain 40: lengthen the exposure, raise the \
                     gain, or add light",
                ],
            ),
            (
                "a few dark frames among lit ones",
                [vec![dark; 4], vec![lit; 6]].concat(),
                fixed,
                vec![],
            ),
            (
                "clipping, under the camera's own exposure",
                vec![clipping; 10],
                InEffect {
                    exposure_us: None,
                    gain: None,
                    auto_exposure: Some(true),
                },
                vec![
                    "whites clipping (12.5% of the frame at full white, over 1%), at the camera's \
                     automatic exposure: shorten the exposure or lower the gain",
                ],
            ),
            ("no frames", vec![], fixed, vec![]),
        ];
        for (name, frames, settings, expected) in cases {
            let mut numbers = Numbers::new(0);
            for brightness in frames {
                numbers.add(Duration::from_millis(3), 0, true, true);
                numbers.light(brightness, settings);
            }
            let warnings = numbers.light_warnings("front-left");
            assert_eq!(warnings.len(), expected.len(), "{name}: {warnings:?}");
            for (warning, part) in warnings.iter().zip(&expected) {
                assert!(warning.starts_with("front-left: "), "{name}: {warning}");
                assert!(warning.contains(part), "{name}: {warning}");
            }
        }
    }

    #[test]
    fn past_a_frame_s_room_the_clearest_tags_are_kept() {
        let found: Vec<Detection> = (0..40).map(|i| detection(i, i as f32)).collect();
        let tags = tags_of(&found, None, 0.1651);
        assert_eq!(tags.len(), MAX_TAGS);
        assert!(tags.iter().all(|t| t.decision_margin >= 8.0));
    }
}
