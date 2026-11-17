//! Dialling in exposure and gain for focusing, against a simulated camera with a real one's
//! traits: brightness goes as exposure times gain, its response is squeezed near the top and clips,
//! a change shows in the frames a few frames after it's asked for (and in the pixels later still),
//! and its frame rate falls once the exposure is longer than a frame. Each room is run from the
//! camera's configured settings, as the focus step starts from them, and fed to the dial at the
//! live view's 10 pictures a second, as the view's thread feeds it.

use std::time::Duration;

use visor::settings::{InEffect, Range, Ranges, Tried};
use visor::stream::dial::{BAND, Dial, Goal, Seen, State, Why};
use visor::stream::light;

/// How the camera's gain amplifies.
#[derive(Debug, Clone, Copy)]
enum GainScale {
    /// In proportion to its value, the least as 1x: 16 to 255, as the measured camera's.
    Proportional,
    /// Doubling each `per_double` of its value from 0, as a camera in decibels does.
    Doubling { per_double: f64 },
}

/// A kind of camera.
#[derive(Debug, Clone, Copy)]
struct Kind {
    name: &'static str,
    ranges: Ranges,
    gain_scale: GainScale,
    fps: f64,
}

/// The measured camera: 1600 x 1304 at 60 fps, exposure 100 µs to 819.2 ms, gain 16 to 255.
const MEASURED: Kind = Kind {
    name: "the measured camera",
    ranges: Ranges {
        exposure_us: Some(Range {
            least: 100,
            most: 819_200,
        }),
        gain: Some(Range {
            least: 16,
            most: 255,
        }),
    },
    gain_scale: GainScale::Proportional,
    fps: 60.0,
};

/// Another: at 30 fps, its exposure no longer than 5 ms, its gain 0 to 100 in decibel-like steps.
const OTHER: Kind = Kind {
    name: "another camera",
    ranges: Ranges {
        exposure_us: Some(Range {
            least: 300,
            most: 5000,
        }),
        gain: Some(Range {
            least: 0,
            most: 100,
        }),
    },
    gain_scale: GainScale::Doubling { per_double: 15.0 },
    fps: 30.0,
};

/// A room: how bright the tag's white is to the sensor, in its linear units per millisecond at 1x
/// gain, and whether there's a tag in view.
#[derive(Debug, Clone, Copy)]
struct Room {
    name: &'static str,
    white_per_ms: f64,
    tag: bool,
}

/// A daylit room: the measured camera saw the tag's white at 199 at 8 ms and the least gain.
const DAYLIT: Room = Room {
    name: "daylit",
    white_per_ms: 38.4,
    tag: true,
};
const DIM: Room = Room {
    name: "dim",
    white_per_ms: 3.0,
    tag: true,
};
const DARK: Room = Room {
    name: "too dark",
    white_per_ms: 0.1,
    tag: true,
};
const BRIGHT: Room = Room {
    name: "very bright",
    white_per_ms: 150.0,
    tag: true,
};
const SUNLIT: Room = Room {
    name: "sunlit",
    white_per_ms: 5000.0,
    tag: true,
};
const NO_TAG: Room = Room {
    name: "no tag",
    white_per_ms: 38.4,
    tag: false,
};

/// The camera's response, as tests/focus.rs has it: a straight line to [`KNEE`], then a slope of
/// [`SQUEEZE`] (a fifth) above it to full white. A ThriftyCam read a sharp tag's edge true (within
/// 3%) to a white of about 170 and falsely sharper past it, 10% by 175 and 30% by 200; a sharp tag
/// rendered through this response reads true to 170, then 9% sharp by 180 and 17% by 188
/// (tests/focus.rs).
const KNEE: f64 = 172.0;
const SQUEEZE: f64 = 0.2;
/// The tag's black, as a share of its white's light: print and flare.
const BLACK_SHARE: f64 = 0.12;
/// A setting change shows in the frames' settings this many frames after it's asked for.
const REPORTED_AFTER_FRAMES: u32 = 3;
/// And in the pixels this long after (measured within 0.6 s).
const PIXELS_AFTER: Duration = Duration::from_millis(450);
/// The live view's rate, at which the dial is given frames.
const VIEW_EVERY: Duration = Duration::from_millis(100);

/// The camera's grey level for a linear level: straight, then squeezed, then clipped.
fn tone(linear: f64) -> u8 {
    let level = if linear <= KNEE {
        linear
    } else {
        KNEE + SQUEEZE * (linear - KNEE)
    };
    level.round().clamp(0.0, 255.0) as u8
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Setting {
    exposure_us: u32,
    gain: i64,
}

/// A simulated camera in a room.
struct Camera {
    kind: Kind,
    room: Room,
    /// What the frames say is in effect, and what the pixels were taken at.
    reported: Setting,
    pixels: Setting,
    /// Asked for, with the frame it was asked at and when.
    pending: Vec<(Setting, u32, i64)>,
    frame: u32,
    now_nanos: i64,
    /// A small, repeatable noise on the white.
    noise: u64,
    /// The longest frame interval seen, in seconds.
    slowest: f64,
}

impl Camera {
    fn new(kind: Kind, room: Room, start: Setting) -> Camera {
        Camera {
            kind,
            room,
            reported: start,
            pixels: start,
            pending: Vec::new(),
            frame: 0,
            now_nanos: 0,
            noise: 0x9e37_79b9_7f4a_7c15,
            slowest: 0.0,
        }
    }

    fn amplification(&self, gain: i64) -> f64 {
        let least = self.kind.ranges.gain.unwrap().least;
        match self.kind.gain_scale {
            GainScale::Proportional => gain as f64 / least as f64,
            GainScale::Doubling { per_double } => 2f64.powf((gain - least) as f64 / per_double),
        }
    }

    /// Asks for settings, held to the camera's ranges as a driver holds them.
    fn ask(&mut self, tried: Tried) {
        let mut wanted = self.pending.last().map_or(self.reported, |p| p.0);
        if let Some(us) = tried.exposure_us {
            let range = self.kind.ranges.exposure_us.unwrap();
            wanted.exposure_us = range.hold(i64::from(us)) as u32;
        }
        if let Some(gain) = tried.gain {
            wanted.gain = self.kind.ranges.gain.unwrap().hold(gain);
        }
        self.pending.push((wanted, self.frame, self.now_nanos));
    }

    /// The next frame: its time moves on by the frame's interval, which an exposure longer than a
    /// frame stretches; gain never does.
    fn next(&mut self) -> (InEffect, Option<u8>) {
        let interval = (1.0 / self.kind.fps).max(f64::from(self.pixels.exposure_us) / 1e6 + 1e-4);
        self.slowest = self.slowest.max(interval);
        self.now_nanos += (interval * 1e9) as i64;
        self.frame += 1;
        for (setting, frame, at) in &self.pending {
            if self.frame >= frame + REPORTED_AFTER_FRAMES {
                self.reported = *setting;
            }
            if self.now_nanos - at >= PIXELS_AFTER.as_nanos() as i64 {
                self.pixels = *setting;
            }
        }
        let (frame, now) = (self.frame, self.now_nanos);
        self.pending.retain(|(_, asked, at)| {
            frame < asked + REPORTED_AFTER_FRAMES || now - at < PIXELS_AFTER.as_nanos() as i64
        });
        let linear = self.room.white_per_ms * f64::from(self.pixels.exposure_us) / 1000.0
            * self.amplification(self.pixels.gain);
        // A deterministic wobble of up to a grey level each way, as a sensor's noise gives.
        self.noise = self
            .noise
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1);
        let wobble = ((self.noise >> 33) % 3) as f64 - 1.0;
        let white = tone(linear + wobble);
        let black = tone(linear * BLACK_SHARE);
        // A tag is found while its white and black differ enough to see.
        let found = self.room.tag && i32::from(white) - i32::from(black) >= 20;
        let in_effect = InEffect {
            exposure_us: Some(self.reported.exposure_us),
            gain: Some(self.reported.gain as i32),
            auto_exposure: Some(false),
        };
        (in_effect, found.then_some(white))
    }
}

/// What a run did.
struct Run {
    state: State,
    asks: Vec<(i64, Tried)>,
    /// The tag's white at the end, as the pixels have it.
    white: Option<u8>,
    camera: Camera,
    seconds: f64,
}

/// Runs the dial on a camera for `seconds` of the camera's time, from `start`, feeding it a frame
/// each view period. It stops early once the dial has settled or given up, unless `on_after` says
/// how much longer to keep going (to see that nothing more is asked).
fn run(
    dial: &mut Dial,
    camera: &mut Camera,
    seconds: f64,
    on_after: Duration,
) -> (Vec<(i64, Tried)>, State, f64) {
    let period = Duration::from_secs_f64(1.0 / camera.kind.fps);
    let mut asks = Vec::new();
    let mut next_view = 0i64;
    let mut done_at: Option<i64> = None;
    while (camera.now_nanos as f64) < seconds * 1e9 {
        let (in_effect, white) = camera.next();
        if camera.now_nanos < next_view {
            continue;
        }
        next_view = camera.now_nanos + VIEW_EVERY.as_nanos() as i64;
        let seen = Seen {
            in_effect,
            white,
            at_nanos: camera.now_nanos,
        };
        let answer = dial.step(&seen, &camera.kind.ranges, period);
        if let Some(tried) = answer.ask {
            asks.push((camera.now_nanos, tried));
            camera.ask(tried);
        }
        if matches!(answer.state, State::Settled | State::CouldNot(_)) {
            let at = *done_at.get_or_insert(camera.now_nanos);
            if camera.now_nanos - at >= on_after.as_nanos() as i64 {
                break;
            }
        }
    }
    (asks, dial.state(), camera.now_nanos as f64 / 1e9)
}

fn dial_in(kind: Kind, room: Room, start: Setting) -> Run {
    let mut dial = Dial::new(Goal::Focusing);
    let mut camera = Camera::new(kind, room, start);
    // Two seconds on past the end: the pixels catch up, and nothing more is asked.
    let (asks, state, seconds) = run(&mut dial, &mut camera, 40.0, Duration::from_secs(2));
    let (_, white) = camera.next();
    Run {
        state,
        asks,
        white,
        camera,
        seconds,
    }
}

/// What a run is expected to end in.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Ends {
    /// In the band, the gain at its least.
    AtLeastGain,
    /// In the band, the exposure as long as a frame allows and the gain raised.
    GainRaised,
    /// Short of the band at the camera's limits, but in light good to focus in.
    Short,
    CouldNot(Why),
}

#[test]
fn the_dial_lights_the_tag_for_focusing_in_every_room_or_says_why_it_can_t() {
    let configured = Setting {
        exposure_us: 1000,
        gain: 40,
    };
    let other_start = Setting {
        exposure_us: 2000,
        gain: 20,
    };
    // Each row: the camera, the room, its settings when the step starts, and how it ends.
    let rows = [
        (MEASURED, DAYLIT, configured, Ends::AtLeastGain),
        (MEASURED, DIM, configured, Ends::GainRaised),
        (MEASURED, DARK, configured, Ends::CouldNot(Why::TooDark)),
        (MEASURED, BRIGHT, configured, Ends::AtLeastGain),
        (
            MEASURED,
            SUNLIT,
            Setting {
                exposure_us: 100,
                gain: 16,
            },
            Ends::CouldNot(Why::TooBright),
        ),
        (MEASURED, NO_TAG, configured, Ends::CouldNot(Why::NoTag)),
        // Started where no tag can be seen: washed out, then found from the ladder's short start.
        (
            MEASURED,
            DAYLIT,
            Setting {
                exposure_us: 200_000,
                gain: 255,
            },
            Ends::AtLeastGain,
        ),
        (OTHER, DAYLIT, other_start, Ends::AtLeastGain),
        (OTHER, BRIGHT, other_start, Ends::AtLeastGain),
        (OTHER, DIM, other_start, Ends::GainRaised),
        (OTHER, DARK, other_start, Ends::CouldNot(Why::TooDark)),
        // Dark enough that the most gain falls short of the band, but light enough to focus in.
        (
            OTHER,
            Room {
                name: "dark but workable",
                white_per_ms: 0.19,
                tag: true,
            },
            other_start,
            Ends::Short,
        ),
        (OTHER, NO_TAG, other_start, Ends::CouldNot(Why::NoTag)),
    ];
    for (kind, room, start, ends) in rows {
        let what = format!("{} in a {} room", kind.name, room.name);
        let run = dial_in(kind, room, start);
        println!(
            "{what}: {:?} after {:.1} s, {} asks, white {:?}, last {:?}",
            run.state,
            run.seconds,
            run.asks.len(),
            run.white,
            run.asks.last()
        );
        let exposure = kind.ranges.exposure_us.unwrap();
        let gain = kind.ranges.gain.unwrap();
        let frame_us = 1e6 / kind.fps;
        // Never past the camera's ranges, never an exposure longer than a frame (so the frame
        // rate holds), and never an ask before the last could show in the pixels.
        for (at, tried) in &run.asks {
            let us = i64::from(tried.exposure_us.unwrap());
            assert!(
                (exposure.least..=exposure.most).contains(&us) && (us as f64) < frame_us,
                "{what}: {tried:?} at {at}"
            );
            assert!(gain.least <= tried.gain.unwrap() && tried.gain.unwrap() <= gain.most);
        }
        for pair in run.asks.windows(2) {
            let apart = Duration::from_nanos((pair[1].0 - pair[0].0) as u64);
            assert!(
                apart >= Duration::from_millis(700),
                "{what}: asks {apart:?} apart"
            );
        }
        if start.exposure_us as f64 <= frame_us {
            assert!(
                run.camera.slowest <= 1.0 / kind.fps + 1e-9,
                "{what}: slowed to {:.1} fps",
                1.0 / run.camera.slowest
            );
        }
        assert!(run.seconds < 30.0, "{what}: {:.1} s", run.seconds);
        let pixels = run.camera.pixels;
        match ends {
            Ends::CouldNot(why) => assert_eq!(run.state, State::CouldNot(why), "{what}"),
            Ends::Short => {
                assert_eq!(run.state, State::Settled, "{what}");
                let white = run.white.expect("a tag in view");
                assert!(
                    white < BAND.0 && white >= light::TOO_DIM_BELOW,
                    "{what}: {white}"
                );
                assert_eq!(pixels.gain, gain.most, "{what}");
            }
            Ends::AtLeastGain | Ends::GainRaised => {
                assert_eq!(run.state, State::Settled, "{what}");
                let white = run.white.expect("a tag in view");
                assert!((BAND.0..=BAND.1).contains(&white), "{what}: white {white}");
                assert!((light::TOO_DIM_BELOW..=light::TOO_BRIGHT_ABOVE).contains(&white));
                if ends == Ends::AtLeastGain {
                    assert_eq!(pixels.gain, gain.least, "{what}");
                } else {
                    assert!(pixels.gain > gain.least, "{what}");
                    let longest = (frame_us * 0.96) as u32 / 100 * 100;
                    let longest = longest.min(exposure.most as u32);
                    assert_eq!(pixels.exposure_us, longest, "{what}");
                }
            }
        }
    }
}

#[test]
fn once_settled_it_holds_still_and_set_it_for_me_again_dials_in_afresh() {
    let start = Setting {
        exposure_us: 1000,
        gain: 40,
    };
    let mut dial = Dial::new(Goal::Focusing);
    let mut camera = Camera::new(MEASURED, DAYLIT, start);
    let (_, state, _) = run(&mut dial, &mut camera, 30.0, Duration::ZERO);
    assert_eq!(state, State::Settled);
    let settled = camera.pixels;

    // The light changes: a cloud, a lamp. Settled, it doesn't chase it.
    camera.room.white_per_ms = DAYLIT.white_per_ms * 2.0;
    let more = camera.now_nanos as f64 / 1e9 + 5.0;
    let (asks, state, _) = run(&mut dial, &mut camera, more, Duration::from_secs(5));
    assert!(asks.is_empty(), "{asks:?}");
    assert_eq!(state, State::Settled);
    assert_eq!(camera.pixels, settled);

    // Set it for me again: a fresh dial, from what's in effect, into the band in the new light.
    let mut again = Dial::new(Goal::Focusing);
    let more = camera.now_nanos as f64 / 1e9 + 30.0;
    let (asks, state, _) = run(&mut again, &mut camera, more, Duration::from_secs(2));
    assert_eq!(state, State::Settled);
    assert!(!asks.is_empty());
    assert!(camera.pixels.exposure_us < settled.exposure_us);
    let (_, white) = camera.next();
    assert!((BAND.0..=BAND.1).contains(&white.unwrap()), "{white:?}");
}

#[test]
fn a_camera_without_an_exposure_control_is_said_at_once() {
    let mut dial = Dial::new(Goal::Focusing);
    let seen = Seen {
        in_effect: InEffect::default(),
        white: Some(100),
        at_nanos: 0,
    };
    let ranges = Ranges {
        exposure_us: None,
        gain: MEASURED.ranges.gain,
    };
    let answer = dial.step(&seen, &ranges, Duration::from_millis(16));
    assert_eq!(answer.ask, None);
    assert_eq!(answer.state, State::CouldNot(Why::NoExposure));
}
