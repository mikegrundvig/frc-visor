//! Setting exposure and gain: finding settings that light a tag well, frame by frame, for the
//! page's "Set exposure and gain". It's a state machine and nothing else: each frame it is told the
//! settings in effect, the camera's ranges, the frame period and the measured tag's white, and
//! answers with the settings to ask for next, if any, and a state to show. It never touches a
//! camera; the live view asks for what it answers.
//!
//! The exposure is the one the camera's configuration asks for, which is chosen for motion blur,
//! and the gain is steered until the tag's white is in [`BAND`]: gain hardly moves the focus number
//! in good light (measured), so it's the one to spend. Only when the gain runs out, at either end,
//! does the exposure move, and never past a frame's length (less a margin), so the frame rate holds;
//! the state then says it moved. A camera configured with no exposure (its own automatic exposure)
//! starts from a ladder: exposures doubling from a short one at the least gain, then gains, until
//! a tag is seen; the exposure it's seen at is then kept as a configured one is. After each ask it
//! waits for the frames to say the change is in effect, and a little longer for the pixels to
//! follow, before it looks again. Once the white is in the band it is settled and does nothing more,
//! whatever the light does after.

use std::time::Duration;

use crate::brightness::{self, Brightness};
use crate::settings::{EXPOSURE_UNIT_US, InEffect, Range, Ranges, Tried};
use crate::stream::light;

/// What the dial starts from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Start {
    /// The exposure the configuration asks for, in microseconds: kept, the gain steered.
    Exposure(u32),
    /// No exposure configured: the ladder, then the exposure a tag was first seen at, kept.
    Ladder,
}

/// What became of the exposure once settled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Exposure {
    /// The configured exposure, kept: only the gain changed.
    Kept,
    /// The gain ran out, so the exposure moved from the configured one.
    Moved,
    /// None was configured: the dial chose one.
    Chosen,
}

/// The tag's white the dial steers into: centred on 130, inside the 90 to 170 a ThriftyCam read a
/// sharp tag's edge flat (within 3%) in, and 20 under the white past which a reading isn't trusted
/// ([`light::TOO_BRIGHT_ABOVE`]), so noise and a little light changing after it settles don't
/// carry it over. With the band at 160 to 200 the dial settled at a white of 200, where a 1.5 px
/// edge read 1.1 px.
pub const BAND: (u8, u8) = (110, 150);

/// The middle of [`BAND`], which each step aims at.
const TARGET: f64 = 130.0;

/// A white at or above this is clipped: how much too bright it is can't be told, so the light is
/// halved.
const CLIPPED_FROM: u8 = 250;

/// The most one step changes the light by, up or down: the camera's response is squeezed near the
/// top, so a reading there understates how far over it is, and a big step up could overshoot into
/// clipping.
const MOST_STEP: f64 = 2.0;

/// The longest exposure, as a share of the frame period. Measured on a camera at 60 fps: 60.3 fps
/// up to 16.0 ms, 59.6 at 16.6 ms, 49.7 at 20 ms, and 16.0 ms is 96% of its 16.7 ms frame.
const FRAME_SHARE: f64 = 0.96;

/// The first rung of the ladder walked with no tag in view: short enough that a brightly lit tag
/// isn't washed out.
const LADDER_START_US: u32 = 1000;

/// A change shows in the frames within 0.6 s of being asked for (measured, not timed finer), so
/// the dial looks again no sooner than this after an ask.
const SETTLE: Duration = Duration::from_millis(700);

/// And no sooner than this many frames after the frames first say the change is in effect.
const SETTLE_FRAMES: u32 = 2;

/// An ask the frames haven't said is in effect after this long is taken as done: the camera may
/// have held a value tighter than its range says, and the next step goes on from what's in effect.
const TIMEOUT: Duration = Duration::from_secs(2);

/// The most asks before the dial gives up: enough for the whole ladder and a dozen steps after.
const MOST_ASKS: u32 = 30;

/// What the dial is doing, to show.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// No tag seen yet: trying rung `rung` (from 1) of `rungs`, at this exposure.
    Searching {
        rung: usize,
        rungs: usize,
        exposure_us: u32,
    },
    /// A tag seen: steering its white into the band.
    Steering,
    /// The tag's white is in the band, or as near as the camera's steps and limits allow with the
    /// light still good enough to measure focus in: done, and what became of the exposure.
    Settled(Exposure),
    /// It can't get there, and why.
    CouldNot(Why),
}

/// Why the dial stopped short.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Why {
    /// The tag is still too dim at the longest exposure a frame allows and the highest gain.
    TooDark,
    /// The tag is still too bright at the shortest exposure and the least gain.
    TooBright,
    /// No tag was seen at any rung of the ladder.
    NoTag,
    /// The light, or the tag, kept changing: no setting held the white in the band.
    Unsteady,
    /// The camera has no exposure control Visor can set.
    NoExposure,
}

/// What one frame tells the dial.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seen {
    /// The exposure and gain the frame was taken at.
    pub in_effect: InEffect,
    /// The measured tag's white (90th percentile inside its square): None with no tag measured.
    pub white: Option<u8>,
    /// When it was captured, in nanoseconds on any steady clock.
    pub at_nanos: i64,
    /// The whole frame's brightness: with no tag seen, which way to look for one.
    pub brightness: Brightness,
}

/// What the dial answers a frame with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Answer {
    /// The settings to ask the camera for now, if any.
    pub ask: Option<Tried>,
    pub state: State,
    /// Once, as it gives up after asking for anything: the settings it started from, to be set
    /// again. Only a settled dial leaves new settings.
    pub put_back: Option<InEffect>,
}

/// An exposure and gain, as the dial reasons about them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Setting {
    exposure_us: u32,
    /// None for a camera without gain.
    gain: Option<i64>,
}

/// An ask the dial is waiting on.
#[derive(Debug, Clone, Copy)]
struct Waiting {
    asked: Setting,
    asked_at: i64,
    /// How many frames have said it's in effect.
    frames_in_effect: u32,
}

/// The dial for one go: made afresh for each.
#[derive(Debug, Clone)]
pub struct Dial {
    start: Start,
    /// The exposure kept: the configured one, or the one a tag was first seen at.
    kept: Option<u32>,
    state: State,
    waiting: Option<Waiting>,
    /// The ladder's rung last asked for, while searching.
    rung: Option<usize>,
    /// The brightest setting seen too dim, and the dimmest seen too bright, with their whites: the
    /// answer is between them, so a step never goes back past either.
    too_dim: Option<(Setting, u8)>,
    too_bright: Option<(Setting, u8)>,
    asks: u32,
    /// The settings in effect at its first frame, put back if it can't get there.
    started: Option<InEffect>,
    /// The search's rungs, chosen when a search begins by which way the frame says to look.
    rungs: Vec<Setting>,
    /// Whether it has said what to put back.
    put_back: bool,
}

/// Which way a search for a tag walks the light.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Way {
    /// A dark frame: up.
    Up,
    /// A clipping frame: down.
    Down,
    /// Neither: down first, then up from where it started. A frame that is neither dark nor
    /// clipping with no tag found is likelier to have the tag washed out than too dim to see (a
    /// tag is found down to 20 grey levels of contrast), and the rungs down are few and shorten
    /// the exposure, which costs nothing in blur.
    DownThenUp,
}

/// The camera's limits as the dial uses them.
#[derive(Debug, Clone, Copy)]
struct Limits {
    exposure: Range,
    /// The longest exposure that keeps the frame rate: at most the camera's longest.
    longest_us: i64,
    gain: Option<Range>,
}

impl Dial {
    pub fn new(start: Start) -> Dial {
        let kept = match start {
            Start::Exposure(us) => Some(us),
            Start::Ladder => None,
        };
        Dial {
            start,
            kept,
            state: State::Steering,
            waiting: None,
            rung: None,
            too_dim: None,
            too_bright: None,
            asks: 0,
            started: None,
            rungs: Vec::new(),
            put_back: false,
        }
    }

    /// What it is doing now.
    pub fn state(&self) -> State {
        self.state
    }

    /// Takes a frame: what to ask for next, if anything, and what to show.
    pub fn step(&mut self, seen: &Seen, ranges: &Ranges, period: Duration) -> Answer {
        if self.started.is_none() {
            self.started = Some(seen.in_effect);
        }
        let mut answer = self.decide(seen, ranges, period);
        // Gave up having changed something: what it started from goes back, once.
        if let State::CouldNot(_) = self.state
            && !self.put_back
            && self.asks > 0
        {
            self.put_back = true;
            answer.put_back = self.started;
        }
        answer
    }

    /// What to ask for next, if anything, and what to show.
    fn decide(&mut self, seen: &Seen, ranges: &Ranges, period: Duration) -> Answer {
        let answer = |dial: &Dial, ask: Option<Setting>| Answer {
            ask: ask.map(|s| Tried {
                exposure_us: Some(s.exposure_us),
                gain: s.gain,
            }),
            state: dial.state,
            put_back: None,
        };
        if matches!(self.state, State::Settled(_) | State::CouldNot(_)) {
            return answer(self, None);
        }
        let Some(limits) = limits(ranges, period) else {
            self.state = State::CouldNot(Why::NoExposure);
            return answer(self, None);
        };
        if !self.ready(seen) {
            return answer(self, None);
        }
        // The exposure kept, never past a frame's length.
        self.kept = self
            .kept
            .map(|us| (i64::from(us).clamp(limits.exposure.least, limits.longest_us)) as u32);
        let ask = match (seen.white, current(seen.in_effect, &limits)) {
            (Some(white), Some(now)) => {
                if self.kept.is_none() {
                    self.kept = Some(now.exposure_us);
                }
                self.steer(now, white, &limits)
            }
            // A tag, but an exposure not known (the camera's automatic exposure, say): from the
            // ladder's first rung, then steered from there.
            (Some(_), None) => self.search(&limits, 0, seen),
            (None, _) => {
                // A tag lost while steering, or none seen yet: a search afresh, the way the frame
                // says; otherwise the search's next rung.
                let next = match (self.state, self.rung) {
                    (State::Searching { .. }, Some(rung)) => rung + 1,
                    _ => 0,
                };
                self.search(&limits, next, seen)
            }
        };
        if let Some(asked) = ask {
            self.asks += 1;
            if self.asks > MOST_ASKS {
                self.state = State::CouldNot(Why::Unsteady);
                return answer(self, None);
            }
            self.waiting = Some(Waiting {
                asked,
                asked_at: seen.at_nanos,
                frames_in_effect: 0,
            });
        }
        answer(self, ask)
    }

    /// Whether the last ask has had time to show in the frames: it's in effect, a little after,
    /// or it has taken too long.
    fn ready(&mut self, seen: &Seen) -> bool {
        let Some(waiting) = &mut self.waiting else {
            return true;
        };
        let since =
            Duration::from_nanos(seen.at_nanos.saturating_sub(waiting.asked_at).max(0) as u64);
        let exposure_in = seen.in_effect.exposure_us == Some(waiting.asked.exposure_us);
        let gain_in = waiting.asked.gain.is_none()
            || seen.in_effect.gain.map(i64::from) == waiting.asked.gain;
        if exposure_in && gain_in {
            waiting.frames_in_effect += 1;
        }
        let shown = waiting.frames_in_effect > SETTLE_FRAMES && since >= SETTLE;
        if shown || since >= TIMEOUT {
            self.waiting = None;
            return true;
        }
        false
    }

    /// Rung `next` of the search; at its first, the rungs are chosen. None left is no tag found.
    fn search(&mut self, limits: &Limits, next: usize, seen: &Seen) -> Option<Setting> {
        if next == 0 {
            self.rungs = match self.start {
                Start::Ladder => ladder(limits),
                Start::Exposure(_) => {
                    let from = current(seen.in_effect, limits).unwrap_or(Setting {
                        exposure_us: self.kept.unwrap_or(limits.exposure.least as u32),
                        gain: limits.gain.map(|g| g.least),
                    });
                    let from = Setting {
                        exposure_us: self.kept.unwrap_or(from.exposure_us),
                        ..from
                    };
                    match way(seen.brightness) {
                        Way::Up => toward(from, Way::Up, limits),
                        Way::Down => toward(from, Way::Down, limits),
                        Way::DownThenUp => {
                            let mut rungs = toward(from, Way::Down, limits);
                            rungs.extend(toward(from, Way::Up, limits));
                            rungs
                        }
                    }
                }
            };
        }
        let rungs = &self.rungs;
        let Some(setting) = rungs.get(next).copied() else {
            self.state = State::CouldNot(Why::NoTag);
            return None;
        };
        self.rung = Some(next);
        self.state = State::Searching {
            rung: next + 1,
            rungs: rungs.len(),
            exposure_us: setting.exposure_us,
        };
        Some(setting)
    }

    /// A step toward the band from `now`, where the tag's white is `white`.
    fn steer(&mut self, now: Setting, white: u8, limits: &Limits) -> Option<Setting> {
        self.state = State::Steering;
        if (BAND.0..=BAND.1).contains(&white) {
            // In the band, but not at the exposure to keep (a first look, from settings tried by
            // hand): the same light at that exposure, when the gain can make it up, then a look
            // again.
            let same = self.spread(light(now, limits), now, false, limits);
            if Some(same.exposure_us) == self.kept && same.exposure_us != now.exposure_us {
                return Some(same);
            }
            self.state = State::Settled(self.became(now));
            return None;
        }
        let brighter = white < BAND.0;
        if brighter {
            if self
                .too_dim
                .is_none_or(|(s, _)| light(s, limits) < light(now, limits))
            {
                self.too_dim = Some((now, white));
            }
        } else if self
            .too_bright
            .is_none_or(|(s, _)| light(s, limits) > light(now, limits))
        {
            self.too_bright = Some((now, white));
        }
        let at_most = now.exposure_us as i64 >= limits.longest_us
            && limits
                .gain
                .is_none_or(|g| now.gain.is_none_or(|n| n >= g.most));
        let at_least = now.exposure_us as i64 <= limits.exposure.least
            && limits
                .gain
                .is_none_or(|g| now.gain.is_none_or(|n| n <= g.least));
        // At the camera's limits: done where the light is still good enough to focus in, short of
        // the band; otherwise it can't be done.
        let good = (light::TOO_DIM_BELOW..=light::TOO_BRIGHT_ABOVE).contains(&white);
        if (brighter && at_most) || (!brighter && at_least) {
            self.state = match (good, brighter) {
                (true, _) => State::Settled(self.became(now)),
                (false, true) => State::CouldNot(Why::TooDark),
                (false, false) => State::CouldNot(Why::TooBright),
            };
            return None;
        }
        let factor = if white >= CLIPPED_FROM {
            1.0 / MOST_STEP
        } else {
            (TARGET / f64::from(white.max(1))).clamp(1.0 / MOST_STEP, MOST_STEP)
        };
        let mut wanted = light(now, limits) * factor;
        // Never back past a setting already seen on the other side: halfway between, instead.
        if let (Some((dim, _)), Some((bright, _))) = (self.too_dim, self.too_bright) {
            let (low, high) = (light(dim, limits), light(bright, limits));
            if wanted <= low || wanted >= high {
                wanted = (low * high).sqrt();
            }
        }
        let next = self.spread(wanted, now, brighter, limits);
        let seen_before = [self.too_dim, self.too_bright]
            .iter()
            .flatten()
            .any(|(s, _)| *s == next);
        if next == now || seen_before {
            return self.nearest_side();
        }
        Some(next)
    }

    /// The camera's steps are too coarse to land in the band: the setting of the two either side
    /// whose white is nearer the middle, when its light is good to focus in.
    fn nearest_side(&mut self) -> Option<Setting> {
        let off = |white: u8| (f64::from(white) - TARGET).abs();
        let good = |white: u8| (light::TOO_DIM_BELOW..=light::TOO_BRIGHT_ABOVE).contains(&white);
        let best = [self.too_dim, self.too_bright]
            .into_iter()
            .flatten()
            .filter(|(_, white)| good(*white))
            .min_by(|a, b| off(a.1).total_cmp(&off(b.1)));
        match best {
            Some((setting, _)) => {
                self.state = State::Settled(self.became(setting));
                Some(setting)
            }
            None => {
                self.state = State::CouldNot(Why::Unsteady);
                None
            }
        }
    }

    /// What became of the exposure, settled at `at`.
    fn became(&self, at: Setting) -> Exposure {
        match self.start {
            Start::Ladder => Exposure::Chosen,
            Start::Exposure(us) if at.exposure_us == us => Exposure::Kept,
            Start::Exposure(_) => Exposure::Moved,
        }
    }

    /// An exposure and gain giving `wanted` light, in the camera's steps: the kept exposure and the
    /// gain to match, or, where the gain runs out, the gain at its end and the exposure moved. One
    /// step at least away from `now` in the direction wanted, when there's room.
    fn spread(&self, wanted: f64, now: Setting, brighter: bool, limits: &Limits) -> Setting {
        let kept = self
            .kept
            .unwrap_or(now.exposure_us)
            .clamp(limits.exposure.least as u32, limits.longest_us as u32);
        let Some(range) = limits.gain else {
            // No gain: the exposure is all there is.
            let mut us = units(wanted, limits);
            if us == now.exposure_us {
                us = nudge(us, brighter, limits);
            }
            return Setting {
                exposure_us: us,
                gain: None,
            };
        };
        let factor = wanted / f64::from(kept);
        let most = gain_factor(Some(range.most), limits);
        if factor < 1.0 {
            // Too bright even at the least gain: a shorter exposure.
            let mut us = units(wanted, limits);
            if us == now.exposure_us && now.gain == Some(range.least) {
                us = nudge(us, brighter, limits);
            }
            return Setting {
                exposure_us: us,
                gain: Some(range.least),
            };
        }
        if factor > most {
            // Too dim even at the most gain: a longer exposure, never past a frame.
            let mut us = units(wanted / most, limits);
            if us == now.exposure_us && now.gain == Some(range.most) {
                us = nudge(us, brighter, limits);
            }
            return Setting {
                exposure_us: us,
                gain: Some(range.most),
            };
        }
        let mut gain = range.hold(gain_for(factor, range).round() as i64);
        if kept == now.exposure_us && Some(gain) == now.gain {
            gain = range.hold(gain + if brighter { 1 } else { -1 });
        }
        Setting {
            exposure_us: kept,
            gain: Some(gain),
        }
    }
}

/// The camera's limits, with the longest exposure kept within a frame; none without an exposure
/// control.
fn limits(ranges: &Ranges, period: Duration) -> Option<Limits> {
    let exposure = ranges.exposure_us?;
    let unit = i64::from(EXPOSURE_UNIT_US);
    // To the microsecond first: a 60 fps period is 16,666.666 µs, and 96% of it is 16,000.
    let in_a_frame = (period.as_secs_f64() * 1e6 * FRAME_SHARE).round() as i64 / unit * unit;
    Some(Limits {
        exposure,
        longest_us: in_a_frame.clamp(exposure.least, exposure.most.max(exposure.least)),
        gain: ranges.gain,
    })
}

/// The setting a frame was taken at, as the dial reasons about it: none while the exposure isn't
/// known.
fn current(in_effect: InEffect, limits: &Limits) -> Option<Setting> {
    let exposure_us = in_effect.exposure_us?;
    if in_effect.auto_exposure == Some(true) {
        return None;
    }
    let gain = match (limits.gain, in_effect.gain) {
        (Some(_), Some(gain)) => Some(i64::from(gain)),
        (Some(range), None) => Some(range.least),
        (None, _) => None,
    };
    Some(Setting { exposure_us, gain })
}

/// How much light a setting lets in, in microseconds at the least gain.
fn light(setting: Setting, limits: &Limits) -> f64 {
    f64::from(setting.exposure_us) * gain_factor(setting.gain, limits)
}

/// How many times the least gain a gain is. A camera whose gain starts above zero is taken to
/// amplify in proportion to it (one whose least is 16, as 1x, gives 2x at 32: measured so); one
/// that starts at zero says nothing of its scale, so each eighth of its range is taken to double.
/// Either way the dial measures after each step, so a wrong guess costs a step, not the answer.
fn gain_factor(gain: Option<i64>, limits: &Limits) -> f64 {
    let (Some(gain), Some(range)) = (gain, limits.gain) else {
        return 1.0;
    };
    if range.least > 0 {
        gain as f64 / range.least as f64
    } else {
        let eighth = ((range.most - range.least) as f64 / 8.0).max(1.0);
        2f64.powf((gain - range.least) as f64 / eighth)
    }
}

/// The gain that is `factor` times the least, by [`gain_factor`]'s rule.
fn gain_for(factor: f64, range: Range) -> f64 {
    if range.least > 0 {
        factor * range.least as f64
    } else {
        let eighth = ((range.most - range.least) as f64 / 8.0).max(1.0);
        range.least as f64 + factor.max(1.0).log2() * eighth
    }
}

/// An exposure in the camera's steps of [`EXPOSURE_UNIT_US`], within its range and a frame.
fn units(exposure_us: f64, limits: &Limits) -> u32 {
    let unit = f64::from(EXPOSURE_UNIT_US);
    let rounded = ((exposure_us / unit).round() * unit) as i64;
    rounded.clamp(limits.exposure.least, limits.longest_us) as u32
}

/// One step of exposure further in the direction wanted, within the limits.
fn nudge(exposure_us: u32, brighter: bool, limits: &Limits) -> u32 {
    let unit = i64::from(EXPOSURE_UNIT_US);
    let moved = if brighter {
        i64::from(exposure_us) + unit
    } else {
        i64::from(exposure_us) - unit
    };
    moved.clamp(limits.exposure.least, limits.longest_us) as u32
}

/// Which way to look for a tag by the frame's brightness: up when its brightest 1% is dark,
/// down when more than 1% of it clips (the thresholds [`brightness`] warns of), down then up when
/// it's neither.
fn way(frame: Brightness) -> Way {
    if frame.clipped > brightness::CLIPPED_ABOVE {
        Way::Down
    } else if frame.high < brightness::DARK_BELOW {
        Way::Up
    } else {
        Way::DownThenUp
    }
}

/// The rungs from `from` one way, the kept exposure first: up, gains doubling to the most, then
/// exposures doubling at the most gain to the longest a frame allows; down, gains halving to the
/// least, then exposures halving at the least gain to the shortest. `from` itself isn't a rung.
fn toward(from: Setting, way: Way, limits: &Limits) -> Vec<Setting> {
    let mut rungs = Vec::new();
    let mut gain = from.gain;
    if let (Some(range), Some(start)) = (limits.gain, from.gain) {
        let mut factor = gain_factor(Some(start), limits);
        let mut at = start;
        loop {
            factor = match way {
                Way::Down => factor / 2.0,
                _ => factor * 2.0,
            };
            let next = range.hold(gain_for(factor.max(1.0), range).round() as i64);
            let further = match way {
                Way::Down => next < at,
                _ => next > at,
            };
            if !further {
                break;
            }
            rungs.push(Setting {
                exposure_us: from.exposure_us,
                gain: Some(next),
            });
            at = next;
        }
        gain = Some(at);
    }
    let mut us = i64::from(from.exposure_us);
    loop {
        let next = match way {
            Way::Down => i64::from(units(us as f64 / 2.0, limits)),
            _ => (us * 2).min(limits.longest_us),
        };
        if next == us {
            break;
        }
        us = next;
        rungs.push(Setting {
            exposure_us: us as u32,
            gain,
        });
    }
    rungs
}

/// The ladder walked with no tag in view: exposures doubling from [`LADDER_START_US`] to the
/// longest a frame allows, at the least gain; then gains doubling at that exposure, to the most.
fn ladder(limits: &Limits) -> Vec<Setting> {
    let least_gain = limits.gain.map(|g| g.least);
    let mut rungs = Vec::new();
    let mut exposure = i64::from(LADDER_START_US).clamp(limits.exposure.least, limits.longest_us);
    loop {
        rungs.push(Setting {
            exposure_us: exposure as u32,
            gain: least_gain,
        });
        if exposure >= limits.longest_us {
            break;
        }
        exposure = (exposure * 2).min(limits.longest_us);
    }
    if let Some(range) = limits.gain {
        let mut factor = 1.0;
        loop {
            factor *= 2.0;
            let gain = range.hold(gain_for(factor, range).round() as i64);
            let last = rungs.last().and_then(|r| r.gain);
            if last.is_some_and(|last| gain <= last) {
                break;
            }
            rungs.push(Setting {
                exposure_us: exposure as u32,
                gain: Some(gain),
            });
            if gain >= range.most {
                break;
            }
        }
    }
    rungs
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limits_of(exposure: (i64, i64), gain: Option<(i64, i64)>, fps: u32) -> Limits {
        let ranges = Ranges {
            exposure_us: Some(Range {
                least: exposure.0,
                most: exposure.1,
            }),
            gain: gain.map(|(least, most)| Range { least, most }),
        };
        limits(&ranges, Duration::from_secs(1) / fps).unwrap()
    }

    /// A ladder's exposures and gains.
    type Rungs = Vec<(u32, Option<i64>)>;

    #[test]
    fn the_longest_exposure_keeps_the_frame_rate_and_the_ladder_doubles_to_it_then_the_gain() {
        // Each row: the exposure's range, the gain's, the frame rate, the longest exposure, and
        // the ladder's exposures and gains.
        let rows: [(_, _, u32, i64, Rungs); 4] = [
            (
                (100, 819_200),
                Some((16, 255)),
                60,
                16_000,
                vec![
                    (1000, Some(16)),
                    (2000, Some(16)),
                    (4000, Some(16)),
                    (8000, Some(16)),
                    (16_000, Some(16)),
                    (16_000, Some(32)),
                    (16_000, Some(64)),
                    (16_000, Some(128)),
                    (16_000, Some(255)),
                ],
            ),
            (
                (100, 819_200),
                None,
                30,
                32_000,
                vec![
                    (1000, None),
                    (2000, None),
                    (4000, None),
                    (8000, None),
                    (16_000, None),
                    (32_000, None),
                ],
            ),
            // A camera whose longest exposure is shorter than a frame, and whose gain starts at 0.
            (
                (300, 5000),
                Some((0, 100)),
                60,
                5000,
                vec![
                    (1000, Some(0)),
                    (2000, Some(0)),
                    (4000, Some(0)),
                    (5000, Some(0)),
                    (5000, Some(13)),
                    (5000, Some(25)),
                    (5000, Some(38)),
                    (5000, Some(50)),
                    (5000, Some(63)),
                    (5000, Some(75)),
                    (5000, Some(88)),
                    (5000, Some(100)),
                ],
            ),
            // One whose shortest is longer than the ladder's start.
            (
                (3000, 8000),
                Some((1, 4)),
                120,
                8000,
                vec![
                    (3000, Some(1)),
                    (6000, Some(1)),
                    (8000, Some(1)),
                    (8000, Some(2)),
                    (8000, Some(4)),
                ],
            ),
        ];
        for (exposure, gain, fps, longest, rungs) in rows {
            let limits = limits_of(exposure, gain, fps);
            assert_eq!(limits.longest_us, longest, "{exposure:?} at {fps}");
            let ladder: Vec<(u32, Option<i64>)> = ladder(&limits)
                .into_iter()
                .map(|s| (s.exposure_us, s.gain))
                .collect();
            assert_eq!(ladder, rungs, "{exposure:?} {gain:?}");
        }
    }

    #[test]
    fn with_no_tag_the_search_walks_toward_the_light_the_frame_says() {
        let limits = limits_of((100, 819_200), Some((16, 255)), 60);
        let from = Setting {
            exposure_us: 4000,
            gain: Some(16),
        };
        let walk = |from: Setting, way: Way, limits: &Limits| -> Rungs {
            toward(from, way, limits)
                .into_iter()
                .map(|s| (s.exposure_us, s.gain))
                .collect()
        };
        assert_eq!(
            walk(from, Way::Up, &limits),
            [
                (4000, Some(32)),
                (4000, Some(64)),
                (4000, Some(128)),
                (4000, Some(255)),
                (8000, Some(255)),
                (16_000, Some(255)),
            ]
        );
        let from_gain = Setting {
            exposure_us: 4000,
            gain: Some(40),
        };
        assert_eq!(
            walk(from_gain, Way::Down, &limits),
            [
                (4000, Some(20)),
                (4000, Some(16)),
                (2000, Some(16)),
                (1000, Some(16)),
                (500, Some(16)),
                (300, Some(16)),
                (200, Some(16)),
                (100, Some(16)),
            ]
        );
        // A camera without gain: the exposure alone.
        let no_gain = limits_of((100, 819_200), None, 60);
        let bare = Setting {
            exposure_us: 4000,
            gain: None,
        };
        assert_eq!(
            walk(bare, Way::Up, &no_gain),
            [(8000, None), (16_000, None)]
        );
        // Which way, by the frame: dark walks up, clipping down, neither down then up.
        let frame = |high: u8, clipped: u16| Brightness {
            median: high / 2,
            high,
            clipped,
        };
        assert_eq!(way(frame(40, 0)), Way::Up);
        assert_eq!(way(frame(255, 3000)), Way::Down);
        assert_eq!(way(frame(200, 50)), Way::DownThenUp);
    }

    #[test]
    fn light_comes_from_the_gain_at_the_kept_exposure_and_the_exposure_moves_only_past_its_ends() {
        let limits = limits_of((100, 819_200), Some((16, 255)), 60);
        let dial = Dial::new(Start::Exposure(4000));
        let now = Setting {
            exposure_us: 4000,
            gain: Some(40),
        };
        // Each row: the light wanted (microseconds at the least gain), then the exposure and gain
        // it's spread over.
        let rows = [
            (4000.0, 4000, 16),
            (8000.0, 4000, 32),
            (12_340.0, 4000, 49),
            (63_000.0, 4000, 252),
            // Past the most gain: a longer exposure, never past a frame (16.0 ms at 60 fps).
            (127_500.0, 8000, 255),
            (1e9, 16_000, 255),
            // Under the least gain: a shorter exposure.
            (2000.0, 2000, 16),
            (10.0, 100, 16),
        ];
        for (wanted, exposure_us, gain) in rows {
            let brighter = wanted > light(now, &limits);
            let spread = dial.spread(wanted, now, brighter, &limits);
            assert_eq!(
                spread,
                Setting {
                    exposure_us,
                    gain: Some(gain)
                },
                "{wanted}"
            );
        }
        // Where rounding would leave it where it is, it still moves a step the way it's wanted.
        let near = dial.spread(10_020.0, now, true, &limits);
        assert_eq!(near.gain, Some(41));
    }
}
