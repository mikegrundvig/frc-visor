//! The detect thread end to end: frames into a camera's slot, datagrams out over UDP on loopback,
//! decoded as the robot would decode them.

mod common;

use std::net::UdpSocket;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use visor::brightness::Brightness;
use visor::clock::{Clocks, EPOCH_NANOS};
use visor::config::Camera;
use visor::latest::{Captured, Latest};
use visor::napa::Detector;
use visor::pipeline;
use visor::settings::InEffect;
use visor::stream::feed::Feed;
use visor::wire::{Frame, VERSION_ID};

/// Clocks with the robot exactly 1,000 s behind the monotonic clock, synced or not.
struct Clocks1000 {
    synced: bool,
}

const ROBOT_BEHIND_NANOS: i64 = 1_000_000_000_000;

impl Clocks for Clocks1000 {
    fn monotonic_nanos(&self) -> i64 {
        5_000_000_000_000
    }
    fn wall_nanos(&self) -> i64 {
        EPOCH_NANOS + 5_000_000_000_000 - ROBOT_BEHIND_NANOS
    }
    fn synced(&self) -> bool {
        self.synced
    }
}

fn camera() -> Camera {
    Camera {
        name: "test".into(),
        index: 5,
        device: PathBuf::from("/dev/null"),
        width: 1600,
        height: 1304,
        fps: 60,
        format: None,
        min_tag_px: 16,
        time_offset_nanos: -2_000_000,
        exposure_us: Some(1000),
        gain: Some(40),
        controls: Vec::new(),
        lens: Some(common::rendered_lens()),
        tag_size_m: 0.1651,
        stream: visor::config::Stream::default(),
    }
}

/// Runs the detect thread on `frames`, one at a time, and returns what the robot received.
fn send_through(frames: Vec<Captured>, synced: bool) -> Vec<Frame> {
    let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
    robot
        .set_read_timeout(Some(Duration::from_secs(10)))
        .unwrap();
    let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
    sender.connect(robot.local_addr().unwrap()).unwrap();
    let camera = camera();
    let latest = Latest::new();
    let stop = AtomicBool::new(false);
    let count = frames.len();
    let mut received = Vec::new();
    thread::scope(|scope| {
        scope.spawn(|| {
            let mut detector = Detector::new(camera.min_tag_px).unwrap();
            let mut send = |bytes: &[u8]| sender.send(bytes).map(|_| ());
            pipeline::run(
                &camera,
                &latest,
                &mut detector,
                &Clocks1000 { synced },
                &mut send,
                &Feed::new(),
                &stop,
            );
        });
        let mut buffer = [0u8; 2048];
        for frame in frames {
            latest.put(frame);
            // One at a time, so none is dropped as stale.
            let n = robot.recv(&mut buffer).expect("a datagram for each frame");
            received.push(Frame::decode(&buffer[..n]).expect("a frame"));
        }
        stop.store(true, Ordering::Relaxed);
    });
    assert_eq!(received.len(), count);
    received
}

fn captured(scene: &common::Scene, at: i64, dequeue_time: bool) -> Captured {
    Captured {
        image: scene.image.clone(),
        monotonic_nanos: at,
        dequeue_time,
        settings: AT_1_MS,
    }
}

/// The capture thread's settings, as read back: 1 ms and gain 40, the camera's automatic exposure
/// off.
const AT_1_MS: InEffect = InEffect {
    exposure_us: Some(1000),
    gain: Some(40),
    auto_exposure: Some(false),
};

#[test]
fn each_frame_reaches_the_robot_with_its_tags_and_robot_time() {
    let scenes: Vec<common::Scene> = common::SCENES.iter().map(|n| common::scene(n)).collect();
    let frames = scenes
        .iter()
        .enumerate()
        .map(|(i, s)| captured(s, 4_000_000_000_000 + i as i64 * 16_666_667, i == 3))
        .collect();
    let received = send_through(frames, true);
    for (i, (frame, scene)) in received.iter().zip(&scenes).enumerate() {
        assert_eq!(frame.version_id, VERSION_ID);
        assert_eq!(frame.sequence, i as u32, "sequence counts frames");
        assert_eq!((frame.width, frame.height, frame.camera), (1600, 1304, 5));
        // Captured on the monotonic clock, moved to the robot's, plus the camera's -2 ms offset.
        let monotonic = 4_000_000_000_000 + i as i64 * 16_666_667;
        assert_eq!(
            frame.capture_nanos,
            monotonic - ROBOT_BEHIND_NANOS - 2_000_000
        );
        assert!(frame.clock_synced);
        assert_eq!(frame.dequeue_time, i == 3);
        // How long its detection took, and none dropped: each was taken before the next came.
        assert!(frame.detect_us > 0, "{}", scene.name);
        assert_eq!(frame.dropped, 0);
        // The exposure and gain it was taken at, and how bright it is, measured from the image.
        assert_eq!(
            (frame.exposure_us, frame.gain, frame.auto_exposure),
            (Some(1000), Some(40), false)
        );
        assert_eq!(
            frame.brightness,
            Brightness::of(&scene.image),
            "{}",
            scene.name
        );
        assert_eq!(
            frame.tags.len(),
            scene.tags.len(),
            "{}: tags sent",
            scene.name
        );
        for known in &scene.tags {
            let tag = frame
                .tags
                .iter()
                .find(|t| i32::from(t.id) == known.id)
                .unwrap();
            let corners = tag.corners.map(f64::from);
            assert!(
                common::worst_corner(&corners, &known.corners) < 0.1,
                "{}",
                scene.name
            );
            assert!(tag.decision_margin > 0.0);
            // Solved: the best pose puts the tag where the renderer did, to half a percent.
            let [best, alt] = tag.poses.expect("poses, with a lens");
            let z = f64::from(best.tvec[2]);
            assert!(
                (z - known.distance_m).abs() < 0.005 * known.distance_m,
                "{} tag {}: {z} m, rendered at {} m",
                scene.name,
                known.id,
                known.distance_m
            );
            assert!(best.error_px <= alt.error_px);
            // NAPA's corner order and the solve's tag frame agree: a printed tag seen from the
            // front has its z axis (out of its face) pointing back toward the camera.
            let rvec = nalgebra::Vector3::from(best.rvec.map(f64::from));
            let tvec = nalgebra::Vector3::from(best.tvec.map(f64::from));
            let face = nalgebra::Rotation3::new(rvec) * nalgebra::Vector3::z();
            assert!(
                face.dot(&tvec) < 0.0,
                "{} tag {}: facing away",
                scene.name,
                known.id
            );
        }
    }
}

#[test]
fn frames_replaced_before_detection_got_to_them_are_counted_in_the_next() {
    let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
    robot
        .set_read_timeout(Some(Duration::from_secs(10)))
        .unwrap();
    let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
    sender.connect(robot.local_addr().unwrap()).unwrap();
    let camera = camera();
    let latest = Latest::new();
    let scene = common::scene("empty");
    // Three frames wait before the detect thread starts: the newest replaced two, and a broken one
    // came after it.
    for at in 0..3 {
        latest.put(captured(&scene, at, false));
    }
    latest.broken();
    let stop = AtomicBool::new(false);
    let mut buffer = [0u8; 2048];
    let received = thread::scope(|scope| {
        scope.spawn(|| {
            let mut detector = Detector::new(camera.min_tag_px).unwrap();
            let mut send = |bytes: &[u8]| sender.send(bytes).map(|_| ());
            pipeline::run(
                &camera,
                &latest,
                &mut detector,
                &Clocks1000 { synced: true },
                &mut send,
                &Feed::new(),
                &stop,
            );
        });
        let n = robot.recv(&mut buffer).expect("the newest frame");
        stop.store(true, Ordering::Relaxed);
        Frame::decode(&buffer[..n]).expect("a frame")
    });
    assert_eq!(received.sequence, 0, "the first frame sent");
    assert_eq!(received.dropped, 3);
}

#[test]
fn an_unsynced_clock_is_flagged() {
    let scene = common::scene("empty");
    let received = send_through(vec![captured(&scene, 0, false)], false);
    assert!(!received[0].clock_synced);
    assert!(
        received[0].tags.is_empty(),
        "a frame without tags is still sent"
    );
}
