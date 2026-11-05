//! The live view end to end: a camera's frames into its slot as capture puts them, the detect thread
//! sending each to the robot over UDP on loopback, the camera's live view, and the server on a free
//! port, fetched over HTTP as a browser would. Watchers that read slowly, stall or vanish never hold
//! detection up.

mod common;

use std::io::{BufRead, BufReader, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, UdpSocket};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use visor::clock::{Clocks, EPOCH_NANOS};
use visor::config::{Camera, Stream};
use visor::latest::{Captured, Latest};
use visor::napa::Detector;
use visor::pipeline;
use visor::settings::InEffect;
use visor::stream::feed::Feed;
use visor::stream::server::{self, Options};
use visor::stream::view::{self, View};
use visor::wire::Frame;

/// The robot's clock, synced, as the pipeline's tests have it.
struct Synced;

impl Clocks for Synced {
    fn monotonic_nanos(&self) -> i64 {
        5_000_000_000_000
    }
    fn wall_nanos(&self) -> i64 {
        EPOCH_NANOS + 4_000_000_000_000
    }
    fn synced(&self) -> bool {
        true
    }
}

/// A camera whose name needs escaping in an address.
fn camera() -> Camera {
    Camera {
        name: "front left".into(),
        index: 2,
        device: PathBuf::from("/dev/null"),
        width: 1600,
        height: 1304,
        fps: 60,
        format: None,
        min_tag_px: 16,
        time_offset_nanos: 0,
        exposure_us: Some(1000),
        gain: Some(40),
        controls: Vec::new(),
        lens: Some(common::rendered_lens()),
        tag_size_m: 0.1651,
        stream: Stream::default(),
    }
}

const STREAM: &str = "/stream/front%20left";

/// What the rig gives a test while it runs.
struct Rig<'a> {
    address: SocketAddr,
    view: &'a View,
}

/// What the robot received while the test ran.
struct Received {
    frames: Vec<Frame>,
    /// Frames whose datagram didn't arrive within a second: detection was held up.
    late: usize,
    /// Visor's own count of frames replaced before detection took them.
    dropped: u64,
}

/// Runs a camera end to end while `test` runs, and for a second at least: one frame at a time,
/// every 16 ms, each waited for at the robot, so any frame detection is held up for shows as late.
fn with_camera(options: Options, test: impl FnOnce(&Rig)) -> Received {
    let scene = common::scene("one-tag");
    let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
    robot
        .set_read_timeout(Some(Duration::from_secs(1)))
        .unwrap();
    let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
    sender.connect(robot.local_addr().unwrap()).unwrap();
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = listener.local_addr().unwrap();
    let camera = camera();
    let latest = Latest::new();
    let feed = Feed::new();
    let views = Arc::new(vec![View::new(&camera.name)]);
    let stop = AtomicBool::new(false);
    let done = AtomicBool::new(false);
    thread::scope(|scope| {
        let (camera, latest, feed, stop, views) = (&camera, &latest, &feed, &stop, &views);
        let served = Arc::clone(views);
        scope.spawn(move || server::run(listener, served, options, stop));
        scope.spawn(move || view::run(camera, feed, latest, &views[0], stop));
        scope.spawn(|| {
            let mut detector = Detector::new(camera.min_tag_px).unwrap();
            let mut send = |bytes: &[u8]| sender.send(bytes).map(|_| ());
            pipeline::run(
                camera,
                latest,
                &mut detector,
                &Synced,
                &mut send,
                feed,
                stop,
            );
        });
        let capture = scope.spawn(|| {
            let mut frames = Vec::new();
            let mut late = 0;
            let mut buffer = [0u8; 2048];
            let mut at = 0;
            while !done.load(Ordering::Relaxed) {
                let mut image = latest.spare(1600, 1304);
                image.pixels.copy_from_slice(&scene.image.pixels);
                latest.put(Captured {
                    image,
                    monotonic_nanos: at,
                    dequeue_time: false,
                    settings: InEffect {
                        exposure_us: Some(1000),
                        gain: Some(40),
                        auto_exposure: Some(false),
                    },
                });
                match robot.recv(&mut buffer) {
                    Ok(n) => frames.push(Frame::decode(&buffer[..n]).expect("a frame")),
                    Err(_) => late += 1,
                }
                at += 16_666_667;
                thread::sleep(Duration::from_millis(16));
            }
            (frames, late)
        });
        let started = Instant::now();
        test(&Rig {
            address,
            view: &views[0],
        });
        // At least a second of frames, for what the robot received to say anything.
        if let Some(rest) = Duration::from_secs(1).checked_sub(started.elapsed()) {
            thread::sleep(rest);
        }
        done.store(true, Ordering::Relaxed);
        let (frames, late) = capture.join().unwrap();
        stop.store(true, Ordering::Relaxed);
        Received {
            frames,
            late,
            dropped: latest.dropped(),
        }
    })
}

fn agent() -> ureq::Agent {
    ureq::Agent::config_builder()
        .timeout_per_read(Some(Duration::from_secs(5)))
        .http_status_as_error(false)
        .build()
        .into()
}

/// One picture of a stream: its part's headers, then its JPEG.
fn next_picture(reader: &mut impl BufRead) -> Vec<u8> {
    let mut line = String::new();
    loop {
        line.clear();
        assert!(reader.read_line(&mut line).unwrap() > 0, "the stream ended");
        if line.trim_end() == "--frame" {
            break;
        }
    }
    let mut length = None;
    loop {
        line.clear();
        reader.read_line(&mut line).unwrap();
        let header = line.trim_end();
        if header.is_empty() {
            break;
        }
        if let Some(value) = header.strip_prefix("Content-Length: ") {
            length = Some(value.parse::<usize>().unwrap());
        }
        if let Some(value) = header.strip_prefix("Content-Type: ") {
            assert_eq!(value, "image/jpeg");
        }
    }
    let mut jpeg = vec![0; length.expect("each part says its length")];
    reader.read_exact(&mut jpeg).unwrap();
    jpeg
}

fn wait_until(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(15);
    while !done() {
        assert!(Instant::now() < deadline, "never: {what}");
        thread::sleep(Duration::from_millis(20));
    }
}

/// Nothing held detection up: every frame reached the robot in time, and none was dropped.
fn assert_detection_kept_up(received: &Received) {
    assert!(
        received.frames.len() > 20,
        "{} frames",
        received.frames.len()
    );
    assert_eq!(received.late, 0, "frames detection was held up for");
    assert_eq!(received.dropped, 0);
    assert!(received.frames.iter().all(|f| f.dropped == 0));
    assert!(received.frames.iter().all(|f| f.tags.len() == 1));
}

#[test]
fn a_watcher_sees_the_tags_outlined_at_the_views_size_and_rate() {
    let scene = common::scene("one-tag");
    let received = with_camera(Options::default(), |rig| {
        let agent = agent();
        let mut index = agent
            .get(format!("http://{}/", rig.address))
            .call()
            .unwrap();
        let page = index.body_mut().read_to_string().unwrap();
        assert!(page.contains(&format!("<img src=\"{STREAM}\"")), "{page}");
        assert!(page.contains(">front left</a>"), "{page}");

        let mut response = agent
            .get(format!("http://{}{STREAM}", rig.address))
            .call()
            .unwrap();
        assert_eq!(response.status(), 200);
        assert_eq!(
            response.headers()["content-type"],
            "multipart/x-mixed-replace; boundary=frame"
        );
        let mut reader = BufReader::new(response.body_mut().as_reader());
        next_picture(&mut reader);
        let started = Instant::now();
        let mut jpeg = Vec::new();
        for _ in 0..10 {
            jpeg = next_picture(&mut reader);
        }
        let seconds = started.elapsed().as_secs_f64();
        // Ten pictures at 10 a second, of a camera detecting about 60.
        assert!((0.8..2.0).contains(&seconds), "{seconds} s");
        let picture = image::load_from_memory(&jpeg).unwrap().to_luma8();
        assert_eq!(picture.dimensions(), (800, 652));
        // The tag's outline runs between its corners, halved: bright on the line, whatever the
        // tag's edge is there.
        let tag = &scene.tags[0];
        let corner = |i: usize| {
            let half = |v: f64| (v + 0.5) * 0.5 - 0.5;
            (half(tag.corners[2 * i]), half(tag.corners[2 * i + 1]))
        };
        for side in 0..4 {
            let ((x0, y0), (x1, y1)) = (corner(side), corner((side + 1) % 4));
            let (x, y) = ((x0 + x1) / 2.0, (y0 + y1) / 2.0);
            let brightest = (-1..=1)
                .flat_map(|dx| (-1..=1).map(move |dy| (dx, dy)))
                .map(|(dx, dy)| {
                    picture.get_pixel(
                        (x.round() as i32 + dx) as u32,
                        (y.round() as i32 + dy) as u32,
                    )[0]
                })
                .max()
                .unwrap();
            assert!(
                brightest > 170,
                "side {side} at ({x:.0}, {y:.0}): {brightest}"
            );
        }
        drop(reader);
        wait_until("the watcher's gone", || rig.view.watchers() == 0);
    });
    assert_detection_kept_up(&received);
}

#[test]
fn nobody_watching_means_no_pictures_made() {
    let received = with_camera(Options::default(), |rig| {
        thread::sleep(Duration::from_millis(600));
        assert_eq!(rig.view.made(), 0);
        // Asking for a camera Visor doesn't have, or for the list, makes none either.
        let agent = agent();
        let missing = agent
            .get(format!("http://{}/stream/back", rig.address))
            .call()
            .unwrap();
        assert_eq!(missing.status(), 404);
        assert_eq!(
            missing.into_body().read_to_string().unwrap(),
            "no camera \"back\": Visor has front left\n"
        );
        thread::sleep(Duration::from_millis(300));
        assert_eq!(rig.view.made(), 0);
    });
    assert_detection_kept_up(&received);
}

/// A watcher that asks for the stream, then never reads again: its socket's receive buffer is as
/// small as the kernel allows, so the server's writes to it soon wait.
fn stalled(address: SocketAddr) -> socket2::Socket {
    let socket = socket2::Socket::new(socket2::Domain::IPV4, socket2::Type::STREAM, None).unwrap();
    socket.set_recv_buffer_size(1).unwrap();
    socket.connect(&address.into()).unwrap();
    let mut stream: TcpStream = socket.try_clone().unwrap().into();
    write!(stream, "GET {STREAM} HTTP/1.1\r\nHost: visor\r\n\r\n").unwrap();
    socket
}

#[test]
fn slow_stalled_and_vanishing_watchers_never_hold_detection_up() {
    let options = Options {
        write_timeout: Duration::from_secs(1),
        most_watchers: 8,
        send_buffer: 16 * 1024,
    };
    let received = with_camera(options, |rig| {
        let address = rig.address;
        // A watcher that reads a picture every half second, slower than they're made.
        let slow = thread::spawn(move || {
            let mut response = agent()
                .get(format!("http://{address}{STREAM}"))
                .call()
                .unwrap();
            let mut reader = BufReader::new(response.body_mut().as_reader());
            let mut pictures = 0;
            for _ in 0..6 {
                next_picture(&mut reader);
                pictures += 1;
                thread::sleep(Duration::from_millis(500));
            }
            pictures
        });
        let stalled = stalled(address);
        // Watchers that vanish: one mid-picture, one before reading anything, again and again.
        for _ in 0..5 {
            let mut response = agent()
                .get(format!("http://{address}{STREAM}"))
                .call()
                .unwrap();
            let mut some = [0u8; 1000];
            response
                .body_mut()
                .as_reader()
                .read_exact(&mut some)
                .unwrap();
            drop(response);
            let gone = TcpStream::connect(address).unwrap();
            write!(&gone, "GET {STREAM} HTTP/1.1\r\nHost: visor\r\n\r\n").unwrap();
            drop(gone);
            thread::sleep(Duration::from_millis(100));
        }
        // The stalled watcher's connection is closed once a write to it has waited a second, and
        // only the slow one is left.
        wait_until("the stalled watcher's closed", || rig.view.watchers() == 1);
        let made = rig.view.made();
        assert_eq!(slow.join().unwrap(), 6);
        assert!(rig.view.made() > made, "pictures went on being made");
        drop(stalled);
        wait_until("every watcher's gone", || rig.view.watchers() == 0);
    });
    assert_detection_kept_up(&received);
}

#[test]
fn watchers_past_the_most_are_turned_away() {
    let options = Options {
        most_watchers: 2,
        ..Options::default()
    };
    let received = with_camera(options, |rig| {
        let agent = agent();
        let url = format!("http://{}{STREAM}", rig.address);
        let first = agent.get(&url).call().unwrap();
        let second = agent.get(&url).call().unwrap();
        assert_eq!(
            (first.status().as_u16(), second.status().as_u16()),
            (200, 200)
        );
        let third = agent.get(&url).call().unwrap();
        assert_eq!(third.status(), 503);
        assert_eq!(
            third.into_body().read_to_string().unwrap(),
            "2 watchers already, the most Visor serves: close another view\n"
        );
        // One leaves, and there's room again.
        drop(first);
        wait_until("a seat's free", || {
            agent.get(&url).call().unwrap().status() == 200
        });
        drop(second);
    });
    assert_detection_kept_up(&received);
}
