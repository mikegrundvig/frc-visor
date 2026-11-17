//! What the live view's page says in plain words: a sentence for the brightness, for focus and for
//! "Set exposure and gain", each short and something a student can act on; a sentence when the
//! camera has slowed because the exposure is longer than a frame; and what to write into the
//! configuration to keep settings tried here. The edge's width in pixels stays, as the small print
//! ([`small_print`]).

use crate::settings::InEffect;
use crate::stream::dial::{Exposure, State, Why};
use crate::stream::focus::Shown;
use crate::stream::light::Light;
use crate::stream::sharpness::{CantJudge, Sharpness, Verdict};

/// A reading this near the best (as a share of it) is as good as the best. The best is the
/// smallest reading shown, so with the lens still the reading sits a little over it: a ThriftyCam
/// left alone read 1.44 to 1.49 px against a best of 1.4, and at 3% the sentence flipped between
/// "as good as the best" and "it was sharper before". 6% of a sharp edge is under a tenth of a
/// pixel.
const AS_GOOD_AS_BEST: f64 = 1.06;

/// A measured rate under this share of the rate asked for counts as slowed.
pub const SLOWED_BELOW: f64 = 0.95;

/// The sentence for the brightness, by the light of the tag the view goes by.
pub fn brightness(light: Option<Light>) -> &'static str {
    match light {
        None => "No tag in view to judge the brightness by: hold a tag in front of the camera.",
        Some(Light::TooDim) => {
            "Too dark: the tag's white is dim. Press Set exposure and gain, or add light."
        }
        Some(Light::Good) => "Good: the tag is lit well enough to measure.",
        Some(Light::TooBright) => {
            "Too bright: the tag's white is washed out. Press Set exposure and gain."
        }
    }
}

/// The sentence for focus: the verdict, and how the lens is doing against the best seen.
pub fn focus(verdict: Verdict, shown: Shown) -> String {
    let guide = match (shown.now_px, shown.best_px) {
        (Some(now), Some(best)) if now <= best * AS_GOOD_AS_BEST => {
            " As good as the best seen while turning."
        }
        (Some(_), Some(_)) => " It was sharper before: turn back slowly until the bar is full.",
        _ => "",
    };
    let text = match verdict {
        Verdict::Judged(Sharpness::Sharp) => "Sharp: lock the lens.",
        Verdict::Judged(Sharpness::Soft) => {
            "Soft: turn the lens slowly and stop where the bar is full."
        }
        Verdict::Judged(Sharpness::NeedsFocusing) => {
            "Needs focusing: turn the lens slowly and watch the edge's width fall."
        }
        Verdict::CantJudge(why) => return cant_judge(why).to_string(),
    };
    format!("{text}{guide}")
}

/// Why focus can't be judged, and what to do.
fn cant_judge(why: CantJudge) -> &'static str {
    match why {
        CantJudge::NoTag => "No tag in view: point the camera at a tag, near the middle.",
        CantJudge::NotInMiddle => {
            "Put a tag near the middle of the view: the lens is softer toward its edges, so tags there aren't judged."
        }
        CantJudge::TooDim | CantJudge::TooBright => {
            "Can't judge focus until the brightness is set: press Set exposure and gain."
        }
        CantJudge::Unreadable => {
            "Can't read the tag's edge: check for glare on it, and that all of it is in view."
        }
    }
}

/// The sentence for "Set exposure and gain" as it runs and once it's done, with the settings in
/// effect.
pub fn setting(state: State, in_effect: InEffect) -> String {
    let exposure = in_effect
        .exposure_us
        .map_or("not known".to_string(), milliseconds);
    let gain = in_effect.gain.map_or("none".to_string(), |g| g.to_string());
    match state {
        State::Searching {
            rung,
            rungs,
            exposure_us,
        } => format!(
            "Looking for a tag: trying exposure {} (step {rung} of {rungs}). Hold a tag in front of the camera.",
            milliseconds(exposure_us)
        ),
        State::Steering => "Setting the gain: hold the tag still.".to_string(),
        State::Settled(Exposure::Kept) => {
            format!("Set: gain {gain}, the configured exposure ({exposure}) kept.")
        }
        State::Settled(Exposure::Moved) => format!(
            "Set, but the gain ran out, so the exposure was changed to {exposure} (gain {gain})."
        ),
        State::Settled(Exposure::Chosen) => {
            format!("Set: exposure {exposure} and gain {gain}, as none is configured.")
        }
        State::CouldNot(why) => format!(
            "{} Nothing was changed: the exposure and gain it started with are back.",
            could_not(why)
        ),
    }
}

/// Why setting exposure and gain stopped, and what to do.
fn could_not(why: Why) -> &'static str {
    match why {
        Why::TooDark => {
            "Could not set it: too dark even at the most gain and the longest exposure a frame allows. Add light on the tag."
        }
        Why::TooBright => {
            "Could not set it: too bright even at the least gain and the shortest exposure. Shade the tag."
        }
        Why::NoTag => {
            "Could not find a tag at any setting. Hold a tag in the middle of the view, 1 to 3 m away, and try again."
        }
        Why::Unsteady => {
            "Could not settle: the light or the tag kept changing. Hold the tag still and try again."
        }
        Why::NoExposure => "This camera's exposure can't be set from here.",
    }
}

/// Whether the exposure and gain in effect are the ones the configuration gave the camera.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Settings {
    Configured,
    ChangedHere,
}

/// The exposure and gain in effect against the ones the configuration gave the camera when it
/// was opened: whether they differ, and a plain sentence saying so, with what to give the
/// configuration to keep them.
pub fn against_configured(configured: InEffect, now: InEffect) -> (Settings, String) {
    let mut changed = Vec::new();
    let mut keep = Vec::new();
    let manual = now.auto_exposure != Some(true);
    if manual && now.exposure_us.is_some() && now.exposure_us != configured.exposure_us {
        let us = now.exposure_us.unwrap_or_default();
        let was = match (configured.auto_exposure, configured.exposure_us) {
            (Some(true), _) | (_, None) => "the camera's automatic exposure".to_string(),
            (_, Some(was)) => format!("the configured {}", milliseconds(was)),
        };
        changed.push(format!(
            "Exposure {}, changed here from {was}.",
            milliseconds(us)
        ));
        keep.push(format!("exposure_us = {us}"));
    }
    if let Some(gain) = now.gain.filter(|_| now.gain != configured.gain) {
        let was = configured.gain.map_or("the camera's own".to_string(), |g| {
            format!("the configured {g}")
        });
        changed.push(format!("Gain {gain}, changed here from {was}."));
        keep.push(format!("gain = {gain}"));
    }
    if changed.is_empty() {
        return (
            Settings::Configured,
            "These are the configuration's settings.".to_string(),
        );
    }
    let they = if changed.len() == 1 {
        "it lasts"
    } else {
        "they last"
    };
    (
        Settings::ChangedHere,
        format!(
            "{} Not kept: {they} until Visor restarts. To keep {}, give this camera {} in its configuration.",
            changed.join(" "),
            if changed.len() == 1 { "it" } else { "them" },
            keep.join(" and ")
        ),
    )
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

/// Microseconds as milliseconds to a tenth: `4.0 ms`.
pub fn milliseconds(us: u32) -> String {
    format!("{:.1} ms", f64::from(us) / 1000.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stream::focus::Reading;

    fn shown(now_px: Option<f64>, best_px: Option<f64>) -> Shown {
        Shown {
            now_px,
            best_px,
            reading: Reading::Edge(now_px.unwrap_or(1.6)),
        }
    }

    #[test]
    fn every_state_has_a_sentence_that_says_what_to_do() {
        use Sharpness::*;
        let cant = Verdict::CantJudge;
        // Each row: the focus verdict, the reading, and a part of what's said.
        let focus_rows = [
            (
                Verdict::Judged(Sharp),
                shown(Some(1.6), Some(1.55)),
                "Sharp: lock the lens. As good",
            ),
            (
                Verdict::Judged(Soft),
                shown(Some(2.4), None),
                "Soft: turn the lens slowly",
            ),
            (
                Verdict::Judged(Soft),
                shown(Some(2.4), Some(1.6)),
                "It was sharper before",
            ),
            (
                Verdict::Judged(NeedsFocusing),
                shown(Some(4.0), None),
                "Needs focusing",
            ),
            (cant(CantJudge::NoTag), shown(None, None), "No tag in view"),
            (
                cant(CantJudge::NotInMiddle),
                shown(None, None),
                "near the middle",
            ),
            (
                cant(CantJudge::TooDim),
                shown(None, None),
                "until the brightness is set",
            ),
            (
                cant(CantJudge::TooBright),
                shown(None, None),
                "until the brightness is set",
            ),
            (cant(CantJudge::Unreadable), shown(None, None), "glare"),
        ];
        for (verdict, focus_now, says) in focus_rows {
            let said = focus(verdict, focus_now);
            assert!(said.contains(says), "{verdict:?}: {said}");
        }
        let lit = [
            (None, "No tag in view"),
            (Some(Light::TooDim), "Too dark"),
            (Some(Light::Good), "Good"),
            (Some(Light::TooBright), "Too bright"),
        ];
        for (light, says) in lit {
            assert!(brightness(light).contains(says), "{light:?}");
        }
        let in_effect = InEffect {
            exposure_us: Some(4000),
            gain: Some(44),
            auto_exposure: Some(false),
        };
        let searching = State::Searching {
            rung: 3,
            rungs: 9,
            exposure_us: 4000,
        };
        let setting_rows = [
            (searching, "trying exposure 4.0 ms (step 3 of 9)"),
            (State::Steering, "Setting the gain"),
            (
                State::Settled(Exposure::Kept),
                "gain 44, the configured exposure (4.0 ms) kept",
            ),
            (
                State::Settled(Exposure::Moved),
                "the exposure was changed to 4.0 ms",
            ),
            (
                State::Settled(Exposure::Chosen),
                "exposure 4.0 ms and gain 44",
            ),
            (
                State::CouldNot(Why::TooDark),
                "Add light on the tag. Nothing was changed",
            ),
            (State::CouldNot(Why::TooBright), "Shade the tag"),
            (State::CouldNot(Why::NoTag), "Could not find a tag"),
            (State::CouldNot(Why::Unsteady), "kept changing"),
            (State::CouldNot(Why::NoExposure), "can't be set"),
        ];
        for (state, says) in setting_rows {
            let said = setting(state, in_effect);
            assert!(said.contains(says), "{state:?}: {said}");
        }
        assert_eq!(
            small_print(shown(Some(2.44), Some(1.94))),
            "edge 2.4 px wide, 1.9 at best"
        );
        assert_eq!(small_print(shown(None, None)), "edge not measured");
    }

    #[test]
    fn settings_changed_here_are_said_against_the_configuration_s_with_how_to_keep_them() {
        let at = |exposure_us: Option<u32>, gain: Option<i32>, auto: bool| InEffect {
            exposure_us,
            gain,
            auto_exposure: Some(auto),
        };
        let configured = at(Some(4000), Some(160), false);
        // Each row: what the configuration gave, what's in effect, whether they differ, and parts
        // of what's said.
        let rows = [
            (
                configured,
                configured,
                Settings::Configured,
                vec!["the configuration's settings"],
            ),
            (
                configured,
                at(Some(4000), Some(52), false),
                Settings::ChangedHere,
                vec![
                    "Gain 52, changed here from the configured 160. Not kept: it lasts until Visor restarts.",
                    "give this camera gain = 52",
                ],
            ),
            (
                configured,
                at(Some(8000), Some(255), false),
                Settings::ChangedHere,
                vec![
                    "Exposure 8.0 ms, changed here from the configured 4.0 ms.",
                    "they last until Visor restarts",
                    "exposure_us = 8000 and gain = 255",
                ],
            ),
            (
                at(None, Some(16), true),
                at(Some(3000), Some(16), false),
                Settings::ChangedHere,
                vec!["from the camera's automatic exposure"],
            ),
        ];
        for (was, now, differs, says) in rows {
            let (got, said) = against_configured(was, now);
            assert_eq!(got, differs, "{said}");
            for part in says {
                assert!(said.contains(part), "{said}");
            }
        }
    }

    #[test]
    fn the_camera_slowed_by_a_long_exposure_is_said_and_gain_never_is() {
        // Each row: the rate measured, the exposure in effect, and what's said, at 60 asked for.
        let rows = [
            (
                Some(30.1),
                Some(33_000),
                Some("(33.0 ms) is longer than a frame, so the camera has slowed to 30 frames"),
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
