//! The live view end to end: a camera's frames into its slot at the camera's rate, the detect thread
//! sending each to the robot over UDP on loopback, the camera's live view, and the server on a free
//! port, fetched over HTTP as a browser would. Watchers that read slowly, stall or vanish never hold
//! detection up: measured against the same camera with no one watching.

mod common;

use std::io::{BufRead, BufReader, Read, Write};
use std::net::{IpAddr, SocketAddr, TcpListener, TcpStream, UdpSocket};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
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
use visor::stream::view::{self, Pace, RealTime, View};
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
        config_id: 0,
    }
}

const STREAM: &str = "/stream/front%20left";

/// The robot controller's address, as the rig's server keeps places for it: another loopback
/// address than the watchers', which connect from 127.0.0.1.
const CONTROLLER: IpAddr = IpAddr::V4(std::net::Ipv4Addr::new(127, 0, 0, 2));

/// The stream tests time detection, so they run one at a time, whatever the test runner's threads.
static SERIAL: Mutex<()> = Mutex::new(());

fn one_at_a_time() -> MutexGuard<'static, ()> {
    // A test that failed holding it says so itself; the others run on.
    SERIAL
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// What the rig gives a test while it runs.
struct Rig<'a> {
    address: SocketAddr,
    view: &'a View,
}

/// What the robot received while the test ran, and when.
struct Received {
    /// How many frames the camera put in its slot.
    put: usize,
    frames: Vec<Frame>,
    /// How long after it was put in the slot each frame reached the robot.
    delays: Vec<Duration>,
    /// Visor's own count of frames replaced before detection took them.
    dropped: u64,
}

impl Received {
    /// The worst delay from the slot to the robot, bar the worst 1%.
    fn worst_delay(&self) -> Duration {
        nearly_worst(self.delays.clone())
    }

    /// The worst detection, bar the worst 1%.
    fn worst_detect(&self) -> Duration {
        let times = self
            .frames
            .iter()
            .map(|f| Duration::from_micros(u64::from(f.detect_us)));
        nearly_worst(times.collect())
    }
}

/// The worst of `times`, bar the worst 1%. A hiccup of a shared machine's scheduler can hold up a
/// frame or two whatever the view does; a view that held detection up would hold up a frame for
/// each of its pictures, a sixth of them.
fn nearly_worst(mut times: Vec<Duration>) -> Duration {
    times.sort();
    let left_out = times.len() / 100;
    times
        .len()
        .checked_sub(left_out + 1)
        .map_or(Duration::ZERO, |i| times[i])
}

/// What the robot's clock is behind the monotonic clock, by [`Synced`]: a frame's capture time
/// plus this is when it was put in the slot.
const ROBOT_BEHIND_NANOS: i64 = 1_000_000_000_000;

/// Runs a camera end to end while `test` runs, and for a second at least. The camera runs free at
/// its rate, as a real one does, putting each frame in the slot on time whatever detection is
/// doing; the robot notes when each arrives. A frame detection is held up for arrives late, and
/// one held up for longer than a frame is replaced, and counted as dropped. The view paces its
/// pictures by `pace`.
fn with_camera(options: Options, pace: &(impl Pace + Sync), test: impl FnOnce(&Rig)) -> Received {
    let scene = common::scene("one-tag");
    let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
    robot
        .set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();
    let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
    sender.connect(robot.local_addr().unwrap()).unwrap();
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = listener.local_addr().unwrap();
    let camera = camera();
    let period = Duration::from_secs(1) / camera.fps;
    let latest = Latest::new();
    let feed = Feed::new();
    let views = Arc::new(vec![View::new(&camera.name)]);
    let stop = AtomicBool::new(false);
    let done = AtomicBool::new(false);
    // How many frames the robot is to receive, once the camera has stopped.
    let expected = AtomicUsize::new(usize::MAX);
    let started = Instant::now();
    let since_start = || started.elapsed().as_nanos() as i64;
    thread::scope(|scope| {
        let (camera, latest, feed, stop, views) = (&camera, &latest, &feed, &stop, &views);
        let served = Arc::clone(views);
        scope.spawn(move || server::run(listener, served, options, CONTROLLER, stop));
        scope.spawn(move || view::run(camera, feed, latest, &views[0], pace, stop));
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
            let mut put = 0;
            while !done.load(Ordering::Relaxed) {
                if let Some(rest) = (period * put).checked_sub(started.elapsed()) {
                    thread::sleep(rest);
                }
                let mut image = latest.spare(1600, 1304);
                image.pixels.copy_from_slice(&scene.image.pixels);
                latest.put(Captured {
                    image,
                    monotonic_nanos: since_start(),
                    dequeue_time: false,
                    settings: InEffect {
                        exposure_us: Some(1000),
                        gain: Some(40),
                        auto_exposure: Some(false),
                    },
                });
                put += 1;
            }
            put as usize
        });
        let robot_side = scope.spawn(|| {
            let mut frames = Vec::new();
            let mut delays = Vec::new();
            let mut buffer = [0u8; 2048];
            // Once the camera stops, the last frames have a second to arrive.
            let mut deadline = None;
            while frames.len() < expected.load(Ordering::Relaxed) {
                if expected.load(Ordering::Relaxed) != usize::MAX {
                    let at =
                        *deadline.get_or_insert_with(|| Instant::now() + Duration::from_secs(1));
                    if Instant::now() > at {
                        break;
                    }
                }
                let Ok(n) = robot.recv(&mut buffer) else {
                    continue;
                };
                let arrived = since_start();
                let frame = Frame::decode(&buffer[..n]).expect("a frame");
                let put_at = frame.capture_nanos + ROBOT_BEHIND_NANOS;
                delays.push(Duration::from_nanos((arrived - put_at) as u64));
                frames.push(frame);
            }
            (frames, delays)
        });
        // A test that fails stops the rig, so it's reported rather than waited on for good.
        let tested = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            test(&Rig {
                address,
                view: &views[0],
            })
        }));
        if let Err(failed) = tested {
            done.store(true, Ordering::Relaxed);
            expected.store(0, Ordering::Relaxed);
            stop.store(true, Ordering::Relaxed);
            std::panic::resume_unwind(failed);
        }
        // At least a second of frames, for what the robot received to say anything.
        if let Some(rest) = Duration::from_secs(1).checked_sub(started.elapsed()) {
            thread::sleep(rest);
        }
        done.store(true, Ordering::Relaxed);
        let put = capture.join().unwrap();
        expected.store(put, Ordering::Relaxed);
        let (frames, delays) = robot_side.join().unwrap();
        stop.store(true, Ordering::Relaxed);
        Received {
            put,
            frames,
            delays,
            dropped: latest.dropped(),
        }
    })
}

/// The view's time, moving only when the view sleeps between pictures: its pictures can be
/// counted per second of it, however busy the machine.
struct Paced {
    start: Instant,
    slept: Mutex<Duration>,
}

impl Paced {
    fn new() -> Paced {
        Paced {
            start: Instant::now(),
            slept: Mutex::new(Duration::ZERO),
        }
    }

    fn seconds(&self) -> f64 {
        self.slept.lock().unwrap().as_secs_f64()
    }
}

impl Pace for Paced {
    fn now(&self) -> Instant {
        self.start + *self.slept.lock().unwrap()
    }

    fn sleep(&self, how_long: Duration) {
        *self.slept.lock().unwrap() += how_long;
    }
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

/// Nothing held detection up long enough to cost a frame: every frame the camera put in its slot
/// reached the robot, and none was dropped.
fn assert_detection_kept_up(received: &Received) {
    assert!(
        received.frames.len() > 20,
        "{} frames",
        received.frames.len()
    );
    assert_eq!(
        received.frames.len(),
        received.put,
        "frames that never arrived"
    );
    assert_eq!(received.dropped, 0);
    assert!(received.frames.iter().all(|f| f.dropped == 0));
    assert!(received.frames.iter().all(|f| f.tags.len() == 1));
}

#[test]
fn a_watcher_sees_the_tags_outlined_at_the_views_size_and_rate() {
    let _one = one_at_a_time();
    let scene = common::scene("one-tag");
    let pace = Paced::new();
    let received = with_camera(Options::default(), &pace, |rig| {
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
        let (made, seconds) = (rig.view.made(), pace.seconds());
        let mut jpeg = Vec::new();
        for _ in 0..10 {
            jpeg = next_picture(&mut reader);
        }
        // Ten a second of the view's time: one each 100 ms it slept. A picture may be counted
        // before its sleep is, so one either way.
        let (made, seconds) = (rig.view.made() - made, pace.seconds() - seconds);
        assert!(made >= 10, "{made} pictures made for the 10 seen");
        let per_second = made as f64 / seconds;
        assert!(
            (made as f64 - seconds * 10.0).abs() <= 1.0,
            "{made} pictures in {seconds} s: {per_second:.1} a second"
        );
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
    let _one = one_at_a_time();
    let received = with_camera(Options::default(), &RealTime, |rig| {
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

/// How much worse the watched run's may be than the unwatched run's: room for a shared machine's
/// scheduler, and well under a frame (16.7 ms at 60 fps). A detect thread held up a frame's time
/// for each picture makes the next frame late by more than this.
const SLACK: Duration = Duration::from_millis(4);

/// That the watched run's worst is near the unwatched run's: no more than [`SLACK`] over it.
fn assert_near(what: &str, watched: Duration, alone: Duration) {
    assert!(
        watched <= alone + SLACK,
        "{what}: {watched:?} watched, {alone:?} with no one watching"
    );
}

#[test]
fn slow_stalled_and_vanishing_watchers_never_hold_detection_up() {
    let _one = one_at_a_time();
    let options = Options {
        write_timeout: Duration::from_secs(1),
        send_buffer: 16 * 1024,
        ..Options::default()
    };
    // The same camera with no one watching, for as long as the watchers take.
    let alone = with_camera(options, &RealTime, |_| {
        thread::sleep(Duration::from_secs(4))
    });
    assert_detection_kept_up(&alone);
    let watched = with_camera(options, &RealTime, |rig| {
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
    assert_detection_kept_up(&watched);
    assert_near(
        "the worst arrival at the robot",
        watched.worst_delay(),
        alone.worst_delay(),
    );
    assert_near(
        "the worst detection",
        watched.worst_detect(),
        alone.worst_detect(),
    );
}

/// A connection to `address` from `from`, one of the loopback addresses.
fn connect_from(from: IpAddr, address: SocketAddr) -> TcpStream {
    let socket = socket2::Socket::new(socket2::Domain::IPV4, socket2::Type::STREAM, None).unwrap();
    socket.bind(&SocketAddr::new(from, 0).into()).unwrap();
    socket.connect(&address.into()).unwrap();
    let stream: TcpStream = socket.into();
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    stream
}

/// A request for `path` addressed to `host`, from `from`, on a connection of its own: the
/// answer's status, and its body up to the first picture's part.
fn get_from(from: IpAddr, address: SocketAddr, path: &str, host: &str) -> (u16, String) {
    let mut connection = connect_from(from, address);
    write!(
        connection,
        "GET {path} HTTP/1.1\r\nHost: {host}\r\nConnection: close\r\n\r\n"
    )
    .unwrap();
    answer(&mut connection)
}

/// The status of a request for the page from `from`, or none if the connection is closed unanswered:
/// turned away, as a connection past the most is until a place is free.
fn status_from(from: IpAddr, address: SocketAddr, host: &str) -> Option<u16> {
    let mut connection = connect_from(from, address);
    write!(
        connection,
        "GET / HTTP/1.1\r\nHost: {host}\r\nConnection: close\r\n\r\n"
    )
    .ok()?;
    let mut status = String::new();
    BufReader::new(connection).read_line(&mut status).ok()?;
    status.split_whitespace().nth(1)?.parse().ok()
}

/// An answer's status, and its body when it says its length: a stream's never ends, so it's left
/// unread.
fn answer(connection: &mut TcpStream) -> (u16, String) {
    let mut reader = BufReader::new(connection);
    let mut status = String::new();
    reader.read_line(&mut status).unwrap();
    let code = status
        .split_whitespace()
        .nth(1)
        .unwrap_or_else(|| panic!("no status: {status:?}"))
        .parse()
        .unwrap();
    let mut length = None;
    let mut line = String::new();
    while reader.read_line(&mut line).unwrap() > 2 {
        if let Some((name, value)) = line.trim_end().split_once(": ")
            && name.eq_ignore_ascii_case("content-length")
        {
            length = Some(value.parse::<usize>().unwrap());
        }
        line.clear();
    }
    let mut body = vec![0; length.unwrap_or(0)];
    reader.read_exact(&mut body).unwrap();
    (code, String::from_utf8_lossy(&body).into_owned())
}

/// Whether the server closed `connection` within `within`, without answering.
fn closed_within(connection: &mut TcpStream, within: Duration) -> bool {
    connection.set_read_timeout(Some(within)).unwrap();
    let mut byte = [0u8; 1];
    match connection.read(&mut byte) {
        Ok(0) => true,
        Err(error) => error.kind() == std::io::ErrorKind::ConnectionReset,
        Ok(_) => false,
    }
}

#[test]
fn watchers_past_the_most_are_turned_away_but_the_controller_has_its_own() {
    let _one = one_at_a_time();
    let options = Options {
        most_watchers: 2,
        kept_for_controller: 1,
        ..Options::default()
    };
    let received = with_camera(options, &RealTime, |rig| {
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
        // The controller's address still has a seat, and only the one kept for it.
        let host = rig.address.to_string();
        let mut kept = connect_from(CONTROLLER, rig.address);
        write!(kept, "GET {STREAM} HTTP/1.1\r\nHost: {host}\r\n\r\n").unwrap();
        assert_eq!(answer(&mut kept).0, 200, "the controller's seat");
        let (status, _) = get_from(CONTROLLER, rig.address, STREAM, &host);
        assert_eq!(status, 503, "the controller past its kept seat");
        // One leaves, and there's room again.
        drop(first);
        wait_until("a seat's free", || {
            agent.get(&url).call().unwrap().status() == 200
        });
        drop(second);
        drop(kept);
    });
    assert_detection_kept_up(&received);
}

#[test]
fn idle_connections_are_closed_and_those_past_the_most_turned_away_but_the_controller_s() {
    let _one = one_at_a_time();
    let options = Options {
        header_timeout: Duration::from_millis(300),
        most_connections: 2,
        kept_for_controller: 1,
        ..Options::default()
    };
    let received = with_camera(options, &RealTime, |rig| {
        let address = rig.address;
        let host = address.to_string();
        let anyone = IpAddr::V4(std::net::Ipv4Addr::LOCALHOST);
        // (what the connection sends before it waits)
        let idle = [
            ("nothing", String::new()),
            (
                "half a request's headers",
                format!("GET / HTTP/1.1\r\nHost: {host}\r\n"),
            ),
            (
                "a request, then nothing more",
                format!("GET / HTTP/1.1\r\nHost: {host}\r\n\r\n"),
            ),
        ];
        for (sent, bytes) in idle {
            let mut connection = connect_from(anyone, address);
            connection.write_all(bytes.as_bytes()).unwrap();
            if !bytes.ends_with("\r\n\r\n") {
                let started = Instant::now();
                assert!(
                    closed_within(&mut connection, Duration::from_secs(3)),
                    "{sent}"
                );
                assert!(started.elapsed() >= Duration::from_millis(250), "{sent}");
                continue;
            }
            assert_eq!(answer(&mut connection).0, 200, "{sent}");
            assert!(
                closed_within(&mut connection, Duration::from_secs(3)),
                "{sent}"
            );
        }
        // Two connections that send nothing yet are as many as anyone may have: a third is closed
        // as it's accepted, while the controller is still answered.
        let held: Vec<TcpStream> = (0..2).map(|_| connect_from(anyone, address)).collect();
        thread::sleep(Duration::from_millis(50));
        let mut turned_away = connect_from(anyone, address);
        assert!(closed_within(&mut turned_away, Duration::from_millis(200)));
        assert_eq!(get_from(CONTROLLER, address, "/", &host).0, 200);
        // Once they're closed for sending nothing, there's room again.
        drop(held);
        wait_until("a connection's place is free", || {
            status_from(anyone, address, &host) == Some(200)
        });
    });
    assert_detection_kept_up(&received);
}

#[test]
fn a_request_not_addressed_to_the_board_is_refused() {
    let _one = one_at_a_time();
    let received = with_camera(Options::default(), &RealTime, |rig| {
        let anyone = IpAddr::V4(std::net::Ipv4Addr::LOCALHOST);
        let port = rig.address.port();
        // (the request's Host, its status)
        let rows = [
            (format!("10.12.34.11:{port}"), 200),
            ("localhost".to_string(), 200),
            (format!("[::1]:{port}"), 200),
            // A web page's own name pointed at the board.
            (format!("visor.example.com:{port}"), 421),
        ];
        for (host, status) in rows {
            for path in ["/", STREAM] {
                let (got, body) = get_from(anyone, rig.address, path, &host);
                assert_eq!(got, status, "{host} {path}: {body}");
            }
        }
    });
    assert_detection_kept_up(&received);
}
