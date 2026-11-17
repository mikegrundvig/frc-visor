//! A camera's live view: its thread, which makes a picture at the view's rate while anyone watches
//! and nothing at all while no one does, and the latest picture, which each watcher's connection
//! takes as it's ready for the next. With each picture the same thread measures focus, judges the
//! brightness and focus, steps "Set exposure and gain" when it's been asked for, makes the page's
//! magnified corner, and keeps what the page's status document says ([`Live`]). It makes a frame
//! at full size when one is asked for ([`Still`]). Its thread runs at a lower priority than capture
//! and detection, so on a busy board the view slows, never detection.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use axum::body::Bytes;
use log::warn;
use tokio::sync::watch;

use crate::config::Camera;
use crate::latest::Latest;
use crate::settings::{InEffect, Trial, Tried};
use crate::stream::corner::Corner;
use crate::stream::dial::{Dial, Seen, Start};
use crate::stream::feed::{Detected, Feed};
use crate::stream::focus::{self, Measured, Shown, Tracker};
use crate::stream::picture::Painter;
use crate::stream::sharpness::{self, Judge, Verdict};
use crate::stream::status::{Live, Opened, Picture, Setting};
pub use crate::stream::still::Still;

/// How often an unwatched view looks for a watcher.
const IDLE_CHECK: Duration = Duration::from_millis(100);

/// How long the view waits for a frame before asking again: a camera that's down sends none.
const FRAME_WAIT: Duration = Duration::from_millis(500);

/// One camera's latest picture, and its watchers.
pub struct View {
    /// The camera's name, as its address has it.
    pub name: String,
    camera: Camera,
    pictures: watch::Sender<Option<Bytes>>,
    made: AtomicU64,
    corner: Mutex<Option<Bytes>>,
    stills: watch::Sender<Option<Arc<Still>>>,
    still_asked: AtomicBool,
    stills_made: AtomicU64,
    forget_focus: AtomicBool,
    focus_forgotten: AtomicU64,
    focus: Mutex<Shown>,
    /// An exposure and gain asked for from the live view, which the camera's capture thread sets.
    pub trial: Trial,
    /// "Set exposure and gain", once asked for: held while what it answers is asked of
    /// [`View::trial`], so an ask from the view's thread and one by hand never cross.
    dial: Mutex<Option<Dial>>,
    live: Mutex<Live>,
}

impl View {
    pub fn new(camera: &Camera) -> View {
        View {
            name: camera.name.clone(),
            camera: camera.clone(),
            pictures: watch::Sender::new(None),
            made: AtomicU64::new(0),
            corner: Mutex::new(None),
            stills: watch::Sender::new(None),
            still_asked: AtomicBool::new(false),
            stills_made: AtomicU64::new(0),
            forget_focus: AtomicBool::new(false),
            focus_forgotten: AtomicU64::new(0),
            focus: Mutex::new(Shown::default()),
            trial: Trial::default(),
            dial: Mutex::new(None),
            live: Mutex::new(Live::default()),
        }
    }

    /// The camera, as configured.
    pub fn camera(&self) -> &Camera {
        &self.camera
    }

    /// How many pictures it has made since Visor started: none while no one watches.
    pub fn made(&self) -> u64 {
        self.made.load(Ordering::Relaxed)
    }

    /// A new watcher: it sees each picture made from now on, the newest when it's ready for one.
    pub fn watch(&self) -> watch::Receiver<Option<Bytes>> {
        self.pictures.subscribe()
    }

    /// How many watch now.
    pub fn watchers(&self) -> usize {
        self.pictures.receiver_count()
    }

    fn show(&self, picture: Option<Bytes>) {
        self.pictures.send_replace(picture);
    }

    /// The newest magnified corner, as a JPEG: none while no one watches.
    pub fn corner(&self) -> Option<Bytes> {
        self.corner.lock().unwrap().clone()
    }

    /// Asks for the camera's next frame at full size. `changed()` on what comes back waits for it;
    /// say when it's had, or given up on, with [`View::forget_still`].
    pub fn ask_still(&self) -> watch::Receiver<Option<Arc<Still>>> {
        let stills = self.stills.subscribe();
        self.still_asked.store(true, Ordering::Release);
        stills
    }

    /// Takes back an ask, and lets go of the frame made for it: two megabytes for a ThriftyCam.
    pub fn forget_still(&self) {
        self.still_asked.store(false, Ordering::Release);
        self.stills.send_replace(None);
    }

    /// How many full-size frames it has made since Visor started: none unless asked.
    pub fn stills_made(&self) -> u64 {
        self.stills_made.load(Ordering::Relaxed)
    }

    /// Forgets the best focus seen, before the next picture: the lens is to be turned afresh.
    pub fn forget_focus(&self) {
        self.forget_focus.store(true, Ordering::Release);
    }

    /// How many times it has forgotten the best focus because it was asked to.
    pub fn focus_forgotten(&self) -> u64 {
        self.focus_forgotten.load(Ordering::Relaxed)
    }

    /// What its newest picture's focus reading is: nothing while no one watches.
    pub fn focus(&self) -> Shown {
        *self.focus.lock().unwrap()
    }

    /// What the page's status document says of it now: nothing of its pictures while no one
    /// watches.
    pub fn live(&self) -> Live {
        *self.live.lock().unwrap()
    }

    /// What the capture thread said of the camera when it last opened it.
    pub fn opened(&self) -> Opened {
        Opened {
            ranges: self.trial.ranges(),
            serial: self.trial.serial(),
            configured: self.trial.configured(),
        }
    }

    /// Starts "Set exposure and gain", afresh if it's running or has run: from the configured
    /// exposure, or the ladder where none is configured. It steps with the view's next picture.
    pub fn set_exposure_and_gain(&self) {
        let start = match self.camera.exposure_us {
            Some(us) => Start::Exposure(us),
            None => Start::Ladder,
        };
        let dial = Dial::new(start);
        let state = dial.state();
        *self.dial.lock().unwrap() = Some(dial);
        self.live.lock().unwrap().setting = Setting::Dial(state);
    }

    /// Asks for an exposure and gain set by hand: "Set exposure and gain", if it's running,
    /// stops, and leaves it be.
    pub fn try_by_hand(&self, tried: Tried) {
        let mut dial = self.dial.lock().unwrap();
        *dial = None;
        self.trial.ask(tried);
        self.live.lock().unwrap().setting = Setting::Idle;
    }

    /// Gives "Set exposure and gain" a frame, if it's been asked for, and asks for what it answers.
    fn step_dial(&self, seen: &Seen) -> Setting {
        let mut dial = self.dial.lock().unwrap();
        let Some(running) = dial.as_mut() else {
            return Setting::Idle;
        };
        let period = Duration::from_secs(1) / self.camera.fps.max(1);
        // The camera not yet opened: nothing to keep to, so nothing is asked.
        if let Some(ranges) = self.trial.ranges() {
            let answer = running.step(seen, &ranges, period);
            if let Some(tried) = answer.ask {
                self.trial.ask(tried);
            }
            // It couldn't get there: what it started from goes back. A camera that was on its own
            // automatic exposure gets it back by way of the configuration.
            match answer.put_back {
                Some(was) if was.auto_exposure == Some(true) || was.exposure_us.is_none() => {
                    self.trial.ask_configuration()
                }
                Some(was) => self.trial.ask(Tried {
                    exposure_us: was.exposure_us,
                    gain: was.gain.map(i64::from),
                }),
                None => {}
            }
        }
        Setting::Dial(running.state())
    }

    /// The last watcher has gone: "Set exposure and gain" stops where it is, and what the page is
    /// told of pictures is forgotten.
    fn unwatched(&self) {
        *self.dial.lock().unwrap() = None;
        *self.live.lock().unwrap() = Live::default();
        *self.focus.lock().unwrap() = Shown::default();
        *self.corner.lock().unwrap() = None;
    }
}

/// The time the view paces its pictures by: the real one ([`RealTime`]), or a test's.
pub trait Pace {
    fn now(&self) -> Instant;
    fn sleep(&self, how_long: Duration);
}

/// The real time.
pub struct RealTime;

impl Pace for RealTime {
    fn now(&self) -> Instant {
        Instant::now()
    }

    fn sleep(&self, how_long: Duration) {
        thread::sleep(how_long);
    }
}

/// Makes pictures from the camera's frames while anyone watches, and a frame at full size when one
/// is asked for, until `stop` is set: one each period of the view's rate by `pace`. Each frame's
/// image goes back to `latest` once its picture is made.
pub fn run(
    camera: &Camera,
    feed: &Feed,
    latest: &Latest,
    view: &View,
    pace: &impl Pace,
    stop: &AtomicBool,
) {
    super::lower_priority(&format!("{}'s live view", camera.name));
    let period = Duration::from_secs_f64(1.0 / f64::from(camera.stream.fps));
    let mut painter = Painter::new(camera.stream);
    let mut looking = Looking::default();
    let mut said = false;
    while !stop.load(Ordering::Relaxed) {
        let watched = view.watchers() > 0;
        if !watched && looking.rate.started() {
            // Its last picture is no one's now; a new watcher waits for a fresh one, and the best
            // focus seen was that watcher's.
            view.show(None);
            looking = Looking::default();
            view.unwatched();
        }
        if !watched && !view.still_asked.load(Ordering::Acquire) {
            thread::sleep(IDLE_CHECK);
            continue;
        }
        let started = pace.now();
        if let Some(left) = feed.ask() {
            latest.recycle(left);
        }
        let Some(frame) = feed.take(FRAME_WAIT) else {
            continue;
        };
        let measured = focus::measure(&frame.image, &frame.tags);
        // Asked for, and still waited for: an ask given up on makes nothing.
        if view.still_asked.swap(false, Ordering::AcqRel) && view.stills.receiver_count() > 0 {
            let still = Still::of(camera, &frame, &measured);
            view.stills_made.fetch_add(1, Ordering::Relaxed);
            view.stills.send_replace(Some(Arc::new(still)));
        }
        if view.forget_focus.swap(false, Ordering::AcqRel) {
            looking.tracker.forget();
            view.focus_forgotten.fetch_add(1, Ordering::Relaxed);
        }
        let painted = watched.then(|| {
            looking.look(view, &frame, &measured);
            painter.paint(&frame, looking.rate.fps())
        });
        latest.recycle(frame.image);
        match painted {
            Some(Ok(jpeg)) => {
                view.made.fetch_add(1, Ordering::Relaxed);
                view.show(Some(Bytes::from(jpeg)));
            }
            Some(Err(why)) if !said => {
                warn!("{}: the live view's picture: {why}", camera.name);
                said = true;
            }
            _ => {}
        }
        // The next a period after this one began; when this one took longer, straight away.
        if let Some(rest) = period.checked_sub(pace.now() - started) {
            pace.sleep(rest);
        }
    }
}

/// What the view keeps from picture to picture while it's watched: the frame rate, the focus
/// reading and its verdict, the magnified corner, and the settings last seen.
#[derive(Default)]
struct Looking {
    rate: Rate,
    tracker: Tracker,
    judge: Judge,
    corner: Corner,
    /// A change of exposure or gain forgets the best focus, which was seen in other light.
    last_settings: Option<InEffect>,
}

impl Looking {
    /// Looks at a watched frame: measures and judges it, steps "Set exposure and gain", makes the
    /// corner, and says it all for the status document.
    fn look(&mut self, view: &View, frame: &Detected, measured: &Measured) {
        let camera = view.camera();
        if self
            .last_settings
            .is_some_and(|last| last != frame.settings)
        {
            self.tracker.forget();
        }
        self.last_settings = Some(frame.settings);
        self.rate.add(frame.sequence, frame.monotonic_nanos);
        let shown = self.tracker.add(measured.reading());
        *view.focus.lock().unwrap() = shown;
        let nearest = measured.nearest.and_then(|i| frame.tags.get(i));
        let centred = nearest
            .is_some_and(|tag| sharpness::in_middle(&tag.corners, camera.width, camera.height));
        let verdict: Verdict = match (sharpness::why_not(measured, centred), shown.now_px) {
            (None, Some(width)) => self.judge.add(Ok(width)),
            (Some(why), _) => self.judge.add(Err(why)),
            // Can't happen: an edge reading is always shown; judged as no tag if it ever does.
            (None, None) => self.judge.add(Err(sharpness::CantJudge::NoTag)),
        };
        let light = measured.light();
        let seen = Seen {
            in_effect: frame.settings,
            white: light.map(|l| l.white),
            at_nanos: frame.monotonic_nanos,
            brightness: frame.brightness,
        };
        let setting = view.step_dial(&seen);
        match self
            .corner
            .make(&frame.image, nearest.map(|tag| &tag.corners))
        {
            Some(Ok(jpeg)) => *view.corner.lock().unwrap() = Some(Bytes::from(jpeg)),
            Some(Err(why)) => warn!("{}: {why}", camera.name),
            None => {}
        }
        *view.live.lock().unwrap() = Live {
            picture: Some(Picture {
                fps: self.rate.fps(),
                in_effect: frame.settings,
                link: frame.link,
                clock: frame.clock,
                light,
                focus: shown,
                verdict,
            }),
            setting,
        };
    }
}

/// Detection's frame rate, from the frames' numbers and capture times over about the last second.
#[derive(Default)]
struct Rate {
    seen: VecDeque<(u32, i64)>,
}

/// The span the rate is measured over.
const RATE_OVER_NANOS: i64 = 1_000_000_000;

impl Rate {
    fn started(&self) -> bool {
        !self.seen.is_empty()
    }

    fn add(&mut self, sequence: u32, monotonic_nanos: i64) {
        if let Some(&(last, _)) = self.seen.back()
            && sequence < last
        {
            self.seen.clear(); // Visor's numbers don't go back: a test's frames, or a wrap
        }
        self.seen.push_back((sequence, monotonic_nanos));
        while self.seen.len() > 2 && monotonic_nanos - self.seen[1].1 >= RATE_OVER_NANOS {
            self.seen.pop_front();
        }
    }

    /// Frames detected per second, once two pictures have been made.
    fn fps(&self) -> Option<f64> {
        let (&(first, from), &(last, to)) = (self.seen.front()?, self.seen.back()?);
        if to <= from {
            return None;
        }
        Some(f64::from(last - first) * 1e9 / (to - from) as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_rate_is_detection_s_over_about_the_last_second() {
        let mut rate = Rate::default();
        assert_eq!(rate.fps(), None);
        // Pictures at 10 a second of a camera detecting 60.
        for picture in 0..30 {
            rate.add(picture * 6, i64::from(picture) * 100_000_000);
            if picture == 0 {
                assert_eq!(rate.fps(), None, "one picture says no rate");
            }
        }
        assert!((rate.fps().unwrap() - 60.0).abs() < 1e-9);
        assert!(rate.seen.len() <= 12, "about a second: {}", rate.seen.len());
        // Detection slows to 30.
        for picture in 30..60 {
            rate.add(180 + (picture - 30) * 3, i64::from(picture) * 100_000_000);
        }
        assert!((rate.fps().unwrap() - 30.0).abs() < 1e-9);
    }

    #[test]
    fn a_still_is_made_only_while_someone_waits_for_it() {
        let view = View::new(&test_camera());
        assert!(!view.still_asked.load(Ordering::Acquire));
        let waiting = view.ask_still();
        assert!(view.still_asked.load(Ordering::Acquire));
        assert_eq!(view.stills.receiver_count(), 1);
        drop(waiting);
        view.forget_still();
        assert!(!view.still_asked.load(Ordering::Acquire));
        assert_eq!(view.stills.receiver_count(), 0);
        assert!(view.stills.borrow().is_none());
    }

    #[test]
    fn a_view_counts_its_watchers() {
        let view = View::new(&test_camera());
        assert_eq!(view.watchers(), 0);
        let one = view.watch();
        let two = view.watch();
        assert_eq!(view.watchers(), 2);
        drop(one);
        assert_eq!(view.watchers(), 1);
        drop(two);
        assert_eq!(view.watchers(), 0);
    }

    #[test]
    fn set_exposure_and_gain_that_can_t_get_there_puts_back_what_it_started_from_and_says_so() {
        use crate::brightness::Brightness;
        use crate::settings::{Range, Ranges, Wanted};
        let mut camera = test_camera();
        camera.gain = Some(40);
        let view = View::new(&camera);
        view.trial.offer(Ranges {
            exposure_us: Some(Range {
                least: 100,
                most: 819_200,
            }),
            gain: Some(Range {
                least: 16,
                most: 255,
            }),
        });
        let started = InEffect {
            exposure_us: Some(1000),
            gain: Some(40),
            auto_exposure: Some(false),
        };
        view.set_exposure_and_gain();
        // A dark room, no tag in view, and a camera that never takes what's asked (so each ask
        // times out): the search walks every rung, finds nothing, and gives up.
        let mut taken = Vec::new();
        let mut setting = Setting::Idle;
        for picture in 0..600 {
            let seen = Seen {
                in_effect: started,
                white: None,
                at_nanos: picture * 100_000_000,
                brightness: Brightness {
                    median: 5,
                    high: 20,
                    clipped: 0,
                },
            };
            setting = view.step_dial(&seen);
            if let Some(wanted) = view.trial.take() {
                taken.push(wanted);
            }
        }
        assert_eq!(
            setting,
            Setting::Dial(crate::stream::dial::State::CouldNot(
                crate::stream::dial::Why::NoTag
            ))
        );
        assert!(taken.len() > 3, "it searched: {taken:?}");
        // The last thing asked is what it started from.
        assert_eq!(
            taken.last(),
            Some(&Wanted::Values(Tried {
                exposure_us: Some(1000),
                gain: Some(40)
            }))
        );
        let said = crate::stream::words::setting(
            crate::stream::dial::State::CouldNot(crate::stream::dial::Why::NoTag),
            started,
        );
        assert!(said.contains("Nothing was changed"), "{said}");
        // And the status document says it, from the view's live state.
        view.live.lock().unwrap().setting = setting;
        view.live.lock().unwrap().picture = Some(Picture {
            fps: Some(60.0),
            in_effect: started,
            link: crate::link_state::LinkState::default(),
            clock: crate::stream::feed::Clock::Synced,
            light: None,
            focus: Shown::default(),
            verdict: Verdict::CantJudge(sharpness::CantJudge::NoTag),
        });
        let document: serde_json::Value = serde_json::from_str(&crate::stream::status::document(
            [(view.camera(), view.live(), view.opened())],
            true,
        ))
        .unwrap();
        let brightness = &document["cameras"][0]["brightness"];
        assert_eq!(brightness["setting"], "could_not");
        assert!(
            brightness["setting_say"]
                .as_str()
                .unwrap()
                .contains("Nothing was changed"),
            "{brightness}"
        );
    }

    fn test_camera() -> Camera {
        Camera {
            name: "front".into(),
            index: 0,
            device: "/dev/null".into(),
            width: 1600,
            height: 1304,
            fps: 60,
            format: None,
            min_tag_px: 16,
            time_offset_nanos: 0,
            exposure_us: Some(1000),
            gain: None,
            controls: Vec::new(),
            lens: None,
            tag_size_m: 0.1651,
            stream: crate::config::Stream::default(),
            config_id: 0,
        }
    }
}
