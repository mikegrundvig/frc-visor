//! What the live view says in plain words: a sentence for each state of focus and of the focus
//! step, short enough for the picture's line, that a student can act on; and a sentence when the
//! camera has slowed because the exposure is longer than a frame. The edge's width in pixels stays,
//! as the small print ([`small_print`]).

use crate::stream::dial::{State, Why};
use crate::stream::focus::{Reading, Shown};
use crate::stream::step::Phase;

/// A reading this near the best (as a share of it) is as good as the best. The best is the
/// smallest reading shown, so with the lens still the reading sits a little over it: a ThriftyCam
/// left alone read 1.44 to 1.49 px against a best of 1.4, and at 3% the sentence flipped between
/// "Sharp" and "Keep turning". 6% of a sharp edge is under a tenth of a pixel.
const AS_GOOD_AS_BEST: f64 = 1.06;

/// A measured rate under this share of the rate asked for counts as slowed.
const SLOWED_BELOW: f64 = 0.95;

/// The sentence for the camera's focus and its focus step.
pub fn sentence(phase: Phase, focus: Shown) -> String {
    match phase {
        Phase::Dialling(State::Searching {
            rung,
            rungs,
            exposure_us,
        }) => format!(
            "Looking for a tag: trying exposure {} (step {rung} of {rungs}). Hold a tag in the middle of the view.",
            milliseconds(exposure_us)
        ),
        Phase::Dialling(State::Steering) => {
            "Setting the exposure for focusing: hold the tag still.".to_string()
        }
        Phase::Dialling(State::CouldNot(why)) => could_not(why).to_string(),
        Phase::Off | Phase::ByHand | Phase::Dialling(State::Settled) => focus_sentence(focus),
    }
}

/// Why the dial stopped, and what to do.
fn could_not(why: Why) -> &'static str {
    match why {
        Why::TooDark => {
            "Could not set it: too dark even at the longest exposure and most gain. Add light on the tag, then try again."
        }
        Why::TooBright => {
            "Could not set it: too bright even at the shortest exposure. Shade the tag, then try again."
        }
        Why::NoTag => {
            "Could not find a tag at any exposure. Put a tag in the middle of the view, 1 to 3 m away, then try again."
        }
        Why::Unsteady => {
            "Could not settle: the light or the tag kept changing. Hold the tag still, then try again."
        }
        Why::NoExposure => "This camera's exposure can't be set from here: set the light by hand.",
    }
}

/// The sentence for focus alone.
pub fn focus_sentence(focus: Shown) -> String {
    let text = match focus.reading {
        Reading::NoTag => "No tag in view: point the camera at a tag, near the middle.",
        Reading::TooDim => {
            "Too dim to measure focus: lengthen the exposure, or add light on the tag."
        }
        Reading::TooBright => {
            "Too bright to measure focus: the tag's white is washed out. Shorten the exposure or lower the gain."
        }
        Reading::Unreadable => {
            "Can't read the tag's edge: check for glare on it, and that all of it is in view."
        }
        Reading::Edge(_) => match (focus.now_px, focus.best_px) {
            (Some(now), Some(best)) if now <= best * AS_GOOD_AS_BEST => {
                "Sharp: as good as the best seen. Lock the lens here."
            }
            (Some(_), Some(_)) => {
                "Keep turning: it was sharper before. Turn back slowly until the bar is full."
            }
            _ => "Turn the lens slowly and watch the bar.",
        },
    };
    text.to_string()
}

/// The small print: `edge 2.4 px wide, 1.9 at best` (smaller is sharper), or nothing measured.
pub fn small_print(focus: Shown) -> String {
    let best =
        |best: Option<f64>| best.map_or(String::new(), |best| format!(", {best:.1} at best"));
    match focus.now_px {
        Some(now) => format!("edge {now:.1} px wide{}", best(focus.best_px)),
        None => format!("edge not measured{}", best(focus.best_px)),
    }
}

/// When the camera has slowed because its exposure is longer than a frame: the frame rate
/// measured is under the one asked for, and the exposure in effect is longer than a frame at it.
/// Gain never slows a camera, so it's never named.
pub fn slowed(fps: Option<f64>, asked_fps: u32, exposure_us: Option<u32>) -> Option<String> {
    let (fps, exposure_us) = (fps?, exposure_us?);
    let frame_us = 1e6 / f64::from(asked_fps.max(1));
    if fps >= f64::from(asked_fps) * SLOWED_BELOW || f64::from(exposure_us) <= frame_us {
        return None;
    }
    Some(format!(
        "The exposure ({}) is longer than a frame, so the camera has slowed to {:.0} frames a second: shorten it.",
        milliseconds(exposure_us),
        fps
    ))
}

fn milliseconds(us: u32) -> String {
    format!("{:.1} ms", f64::from(us) / 1000.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shown(now_px: Option<f64>, best_px: Option<f64>, reading: Reading) -> Shown {
        Shown {
            now_px,
            best_px,
            reading,
        }
    }

    #[test]
    fn every_state_has_a_sentence_that_says_what_to_do() {
        let edge = Reading::Edge(1.6);
        // Each row: the step, the focus, and a part of what's said.
        let rows = [
            (
                Phase::Off,
                shown(None, None, Reading::NoTag),
                "No tag in view",
            ),
            (
                Phase::Off,
                shown(None, Some(1.6), Reading::TooDim),
                "Too dim",
            ),
            (
                Phase::ByHand,
                shown(None, None, Reading::TooBright),
                "Too bright",
            ),
            (Phase::Off, shown(None, None, Reading::Unreadable), "glare"),
            (
                Phase::Off,
                shown(Some(1.6), None, edge),
                "Turn the lens slowly",
            ),
            (Phase::Off, shown(Some(1.62), Some(1.6), edge), "Sharp"),
            (
                Phase::Off,
                shown(Some(2.4), Some(1.6), edge),
                "Keep turning",
            ),
            (
                Phase::Dialling(State::Settled),
                shown(Some(1.6), Some(1.6), edge),
                "Sharp",
            ),
            (
                Phase::Dialling(State::Searching {
                    rung: 3,
                    rungs: 9,
                    exposure_us: 4000,
                }),
                shown(None, None, Reading::NoTag),
                "trying exposure 4.0 ms (step 3 of 9)",
            ),
            (
                Phase::Dialling(State::Steering),
                shown(None, None, Reading::TooBright),
                "Setting the exposure",
            ),
            (
                Phase::Dialling(State::CouldNot(Why::TooDark)),
                shown(None, None, Reading::TooDim),
                "Add light",
            ),
            (
                Phase::Dialling(State::CouldNot(Why::TooBright)),
                shown(None, None, Reading::TooBright),
                "Shade the tag",
            ),
            (
                Phase::Dialling(State::CouldNot(Why::NoTag)),
                shown(None, None, Reading::NoTag),
                "Could not find a tag",
            ),
            (
                Phase::Dialling(State::CouldNot(Why::Unsteady)),
                shown(None, None, Reading::NoTag),
                "kept changing",
            ),
            (
                Phase::Dialling(State::CouldNot(Why::NoExposure)),
                shown(None, None, Reading::NoTag),
                "can't be set",
            ),
        ];
        for (phase, focus, says) in rows {
            let said = sentence(phase, focus);
            assert!(said.contains(says), "{phase:?} {focus:?}: {said}");
            // Fits the picture's line at its default size: 800 px of 7 px letters.
            assert!(said.len() <= 114, "{said}");
        }
        assert_eq!(
            small_print(shown(Some(2.44), Some(1.94), edge)),
            "edge 2.4 px wide, 1.9 at best"
        );
        assert_eq!(
            small_print(shown(None, None, Reading::NoTag)),
            "edge not measured"
        );
    }

    #[test]
    fn the_camera_slowed_by_a_long_exposure_is_said_and_gain_never_is() {
        // Each row: the rate measured, the exposure in effect, and what's said, at 60 asked for.
        let rows = [
            (
                Some(30.1),
                Some(33_000),
                Some(
                    "(33.0 ms) is longer than a frame, so the camera has slowed to 30 frames a second",
                ),
            ),
            (Some(49.7), Some(20_000), Some("slowed to 50 frames")),
            // Slow, but the exposure is shorter than a frame: something else (USB), not said here.
            (Some(30.0), Some(8000), None),
            // A long exposure that hasn't slowed it yet.
            (Some(59.6), Some(16_600), None),
            (None, Some(33_000), None),
            (Some(30.0), None, None),
        ];
        for (fps, exposure_us, says) in rows {
            let said = slowed(fps, 60, exposure_us);
            match (said, says) {
                (Some(said), Some(says)) => assert!(said.contains(says), "{said}"),
                (None, None) => {}
                (said, _) => panic!("{fps:?} {exposure_us:?}: {said:?}"),
            }
        }
    }
}
