//! `visor run`, the service: three threads per camera (capture, detect, live view), the live
//! view's server, and the supervisor that tells systemd Visor is alive ([`watchdog`]), until
//! stopped.
//!
//! Each camera's source, detector, socket and slot are handed over together, as one [`Wired`], so
//! one camera's frames can't go out under another's index.

use std::net::{SocketAddr, TcpListener, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::thread;
use std::time::Duration;

use log::error;

use crate::capture::{self, Source};
use crate::clock::Clocks;
use crate::config::Camera;
use crate::latest::Latest;
use crate::napa::Detector;
use crate::pipeline;
use crate::stream::feed::Feed;
use crate::stream::server;
use crate::stream::view::{self, RealTime, View};
use crate::watchdog;

/// One camera and what its threads are given: where its frames come from, its detector, the
/// socket its frames go to the robot from, and the slot between its capture and detect threads.
pub struct Wired<'a, S> {
    pub camera: &'a Camera,
    pub source: S,
    pub detector: Detector,
    pub socket: UdpSocket,
    pub latest: Latest,
}

/// The live view's listener on `address`, or none, said in the log, when the port can't be had:
/// Visor detects all the same.
pub fn listen(address: SocketAddr) -> Option<TcpListener> {
    match TcpListener::bind(address) {
        Ok(listener) => Some(listener),
        Err(why) => {
            error!(
                "the live view can't have port {} ({why}): detection goes on without it",
                address.port()
            );
            None
        }
    }
}

/// Runs every camera's threads, the live view's server on `listener` if there is one, and the
/// supervisor, until `stop` is set; with no cameras, the server and the supervisor alone. Each
/// camera's frames go to `robot`, and systemd is told Visor is alive each `alive_every` while every
/// camera's detect thread runs ([`watchdog::interval`]: none when systemd isn't watching).
///
/// A thread that panics ends the process only because panics abort (Cargo.toml's profiles): the
/// scope would otherwise wait for the other threads, which never finish. systemd then restarts
/// Visor (deploy/visor.service).
pub fn run<S: Source + Send>(
    mut cameras: Vec<Wired<'_, S>>,
    robot: SocketAddr,
    listener: Option<TcpListener>,
    options: server::Options,
    alive_every: Option<Duration>,
    clocks: &(impl Clocks + Sync),
    stop: &AtomicBool,
) {
    let feeds: Vec<Feed> = cameras.iter().map(|_| Feed::new()).collect();
    let views: Arc<Vec<View>> = Arc::new(
        cameras
            .iter()
            .map(|wired| View::new(&wired.camera.name))
            .collect(),
    );
    thread::scope(|scope| {
        if let Some(listener) = listener {
            let views = Arc::clone(&views);
            scope.spawn(move || server::run(listener, views, options, robot.ip(), stop));
        }
        let mut watched: Vec<watchdog::Watched> = Vec::new();
        for ((wired, feed), shown) in cameras.iter_mut().zip(&feeds).zip(views.iter()) {
            let Wired {
                camera,
                source,
                detector,
                socket,
                latest,
            } = wired;
            let (camera, latest, socket) = (*camera, &*latest, &*socket);
            watched.push((camera.name.as_str(), latest));
            scope.spawn(move || {
                capture::run_from(source, camera, latest, clocks, stop, capture::RETRY)
            });
            scope.spawn(move || {
                panic_if_a_test_asks(&camera.name);
                let mut send = |bytes: &[u8]| socket.send_to(bytes, robot).map(|_| ());
                pipeline::run(camera, latest, detector, clocks, &mut send, feed, stop)
            });
            scope.spawn(move || view::run(camera, feed, latest, shown, &RealTime, stop));
        }
        scope.spawn(move || watchdog::supervise(&watched, alive_every, stop));
    });
}

/// A fault switch for tests/run.rs, in debug builds only: the detect thread of the camera that
/// VISOR_TEST_PANIC names panics as it starts, so the test can check that a panic ends the process.
#[cfg(debug_assertions)]
fn panic_if_a_test_asks(camera: &str) {
    let Some(asked) = std::env::var_os("VISOR_TEST_PANIC") else {
        return;
    };
    if asked == camera {
        panic!("{camera}: the panic VISOR_TEST_PANIC asks for");
    }
}

/// A release build has no fault switch.
#[cfg(not(debug_assertions))]
fn panic_if_a_test_asks(_camera: &str) {}
