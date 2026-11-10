//! systemd's watchdog (deploy/visor.service: `Type=notify`, `WatchdogSec=`): Visor says it's ready
//! once its cameras' threads are about to start, and then, from a supervisor thread, that it's
//! alive, but only while every camera's detect thread is. systemd restarts a Visor that stops
//! saying so, as it restarts one that ends.
//!
//! A detect thread asks its camera's slot for a frame on every turn of its loop, and waits at most
//! 100 ms for one ([`Latest::asked`]). So Visor is healthy:
//!
//! - streaming, while every detect thread takes frame after frame;
//! - with a camera absent and its capture thread retrying, as its detect thread still asks each
//!   100 ms it waits; a restart wouldn't plug the camera in, and the robot sees it's absent;
//! - with no camera configured, always: there's no thread to stop.
//!
//! A detect thread that hangs (in the detector, say) stops asking, and Visor stops saying it's
//! alive. Run by hand or in a test, with no `NOTIFY_SOCKET`, nothing is said to anyone.

use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use log::{info, warn};
use sd_notify::NotifyState;

use crate::latest::Latest;

/// How often the supervisor checks whether it's to stop, between looks.
const STOP_CHECK: Duration = Duration::from_millis(100);

/// Tells systemd Visor is ready: its configuration read, its cameras' threads about to start.
pub fn ready() {
    say(NotifyState::Ready);
}

/// How often to tell systemd Visor is alive: half its watchdog's time, as systemd advises; none
/// when systemd isn't watching.
pub fn interval() -> Option<Duration> {
    sd_notify::watchdog_enabled()
        .filter(|watched| !watched.is_zero())
        .map(|watched| watched / 2)
}

/// Says one thing to systemd. None of it matters to detection, so a failure is only logged.
fn say(state: NotifyState) {
    if let Err(error) = sd_notify::notify(&[state]) {
        warn!("can't tell systemd how Visor is: {error}");
    }
}

/// A camera's name and slot: its detect thread's asks of the slot say it runs.
pub type Watched<'a> = (&'a str, &'a Latest);

/// The cameras whose detect threads haven't asked for a frame since the counts `before`: none
/// means healthy.
pub fn stopped<'a>(cameras: &[Watched<'a>], before: &[u64]) -> Vec<&'a str> {
    cameras
        .iter()
        .zip(before)
        .filter(|((_, latest), before)| latest.asked() == **before)
        .map(|((name, _), _)| *name)
        .collect()
}

/// Watches each camera's detect thread until `stop` is set, telling systemd Visor is alive each
/// `every` while all of them ran in it ([`interval`]). With systemd not watching, it only waits
/// for `stop`, which keeps a board with no camera running.
pub fn supervise(cameras: &[Watched<'_>], every: Option<Duration>, stop: &AtomicBool) {
    let Some(every) = every else {
        while !stop.load(Ordering::Relaxed) {
            thread::sleep(STOP_CHECK);
        }
        return;
    };
    info!(
        "telling systemd every {} ms that every detect thread runs",
        every.as_millis()
    );
    watch(cameras, every, &mut || say(NotifyState::Watchdog), stop);
}

/// [`supervise`]'s loop: `alive` as it starts, then after each `every` in which every camera's
/// detect thread ran.
pub fn watch(
    cameras: &[Watched<'_>],
    every: Duration,
    alive: &mut impl FnMut(),
    stop: &AtomicBool,
) {
    let counts = || -> Vec<u64> { cameras.iter().map(|(_, latest)| latest.asked()).collect() };
    let mut before = counts();
    let mut said_stopped = false;
    alive();
    while !stop.load(Ordering::Relaxed) {
        let look = Instant::now() + every;
        while Instant::now() < look && !stop.load(Ordering::Relaxed) {
            thread::sleep(STOP_CHECK.min(every));
        }
        let stopped = stopped(cameras, &before);
        if stopped.is_empty() {
            alive();
            said_stopped = false;
        } else if !said_stopped {
            warn!(
                "detection stopped for {}: systemd restarts Visor unless it goes on",
                stopped.join(", ")
            );
            said_stopped = true;
        }
        before = counts();
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::AtomicUsize;

    use super::*;

    /// How a camera's detect thread does between two looks.
    #[derive(Clone, Copy)]
    enum Detect {
        /// It runs: it takes frames, or waits 100 ms for one and asks again.
        Runs,
        /// It hangs.
        Hangs,
    }

    #[test]
    fn visor_is_healthy_only_while_every_detect_thread_runs() {
        use Detect::*;
        // (the board, how each camera's detect thread does, the cameras said to have stopped)
        let rows: [(&str, &[Detect], &[&str]); 5] = [
            ("streaming, two cameras", &[Runs, Runs], &[]),
            (
                "a camera absent and retrying: its detect thread waits and asks again",
                &[Runs],
                &[],
            ),
            ("no camera configured", &[], &[]),
            ("one detect thread hangs", &[Runs, Hangs], &["camera-1"]),
            ("both hang", &[Hangs, Hangs], &["camera-0", "camera-1"]),
        ];
        for (board, threads, stopped_ones) in rows {
            let slots: Vec<Latest> = threads.iter().map(|_| Latest::new()).collect();
            let names: Vec<String> = (0..threads.len()).map(|i| format!("camera-{i}")).collect();
            let cameras: Vec<Watched> = names.iter().map(String::as_str).zip(&slots).collect();
            let before: Vec<u64> = slots.iter().map(Latest::asked).collect();
            for (thread, latest) in threads.iter().zip(&slots) {
                if let Runs = thread {
                    // No frame comes: an absent camera's, or one between frames.
                    assert!(latest.take(Duration::ZERO).is_none());
                }
            }
            assert_eq!(stopped(&cameras, &before), stopped_ones, "{board}");
        }
    }

    #[test]
    fn the_supervisor_says_alive_while_the_threads_run_and_stops_once_one_hangs() {
        let (front, back) = (Latest::new(), Latest::new());
        let cameras = [("front", &front), ("back", &back)];
        let (stop, hung) = (AtomicBool::new(false), AtomicBool::new(false));
        let said = AtomicUsize::new(0);
        let every = Duration::from_millis(50);
        let (said_before_the_hang, said_after) = thread::scope(|scope| {
            // Both detect threads wait a millisecond for frames that never come, until the back
            // one hangs.
            scope.spawn(|| {
                while !stop.load(Ordering::Relaxed) {
                    front.take(Duration::from_millis(1));
                    if !hung.load(Ordering::Relaxed) {
                        back.take(Duration::ZERO);
                    }
                }
            });
            let watching = scope.spawn(|| {
                let mut alive = || {
                    said.fetch_add(1, Ordering::Relaxed);
                };
                watch(&cameras, every, &mut alive, &stop);
            });
            thread::sleep(every * 6);
            hung.store(true, Ordering::Relaxed);
            let before = said.load(Ordering::Relaxed);
            thread::sleep(every * 6);
            stop.store(true, Ordering::Relaxed);
            watching.join().unwrap();
            (before, said.load(Ordering::Relaxed))
        });
        // As it starts and at a look or more while both ran; then at most at the look under way as
        // the back thread hung, and never again.
        assert!(said_before_the_hang >= 2, "{said_before_the_hang}");
        assert!(
            said_after <= said_before_the_hang + 1,
            "{said_after} after {said_before_the_hang}"
        );
    }
}
