//! The page's one status document, `GET /status`: every camera's facts, checks, brightness, focus
//! and lens, as JSON, made from what each view's thread last saw. Everything the page shows comes
//! from it, so nothing it shows is older than its last read.

use serde::Serialize;

use crate::config::Camera;
use crate::link_state::{LinkState, Speed};
use crate::settings::{InEffect, Range, Ranges};
use crate::stream::checks::{self, Check};
use crate::stream::dial::State;
use crate::stream::feed::Clock;
use crate::stream::focus::Shown;
use crate::stream::light::{Light, TagLight};
use crate::stream::picture::share_of_best;
use crate::stream::server;
use crate::stream::sharpness::Verdict;
use crate::stream::still::thousandths;
use crate::stream::words;

/// What a view's newest picture said.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Picture {
    /// Detection's frame rate, once measured.
    pub fps: Option<f64>,
    pub in_effect: InEffect,
    pub link: LinkState,
    pub clock: Clock,
    /// The light of the tag the view goes by.
    pub light: Option<TagLight>,
    pub focus: Shown,
    pub verdict: Verdict,
}

/// "Set exposure and gain", as it stands.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Setting {
    /// Not asked for since the view was last watched, or stopped by a change by hand.
    #[default]
    Idle,
    /// Asked for: what the dial is doing, or how it ended.
    Dial(State),
}

/// A view's status: its newest picture, and the setting of exposure and gain.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Live {
    pub picture: Option<Picture>,
    pub setting: Setting,
}

/// What the capture thread said of the camera when it last opened it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Opened {
    pub ranges: Option<Ranges>,
    pub serial: Option<String>,
    /// The exposure and gain its configuration gave it, as read back.
    pub configured: Option<InEffect>,
}

/// The document: whether settings can be changed from the page, and each camera.
#[derive(Debug, Serialize)]
pub struct Document {
    pub can_set: bool,
    pub cameras: Vec<CameraStatus>,
}

/// One camera's part of the document (docs/live-view.md, "The status document").
#[derive(Debug, Serialize)]
pub struct CameraStatus {
    name: String,
    index: u8,
    serial: Option<String>,
    device: String,
    width: u32,
    height: u32,
    fps_asked: u32,
    fps: Option<f64>,
    link: Option<String>,
    stream: String,
    corner: String,
    frame: String,
    forget: String,
    set: String,
    by_hand: String,
    checks: Vec<CheckStatus>,
    brightness: BrightnessStatus,
    focus: FocusStatus,
    lens: LensStatus,
}

#[derive(Debug, Serialize)]
struct CheckStatus {
    topic: &'static str,
    mark: &'static str,
    say: String,
}

#[derive(Debug, Serialize)]
struct BrightnessStatus {
    verdict: &'static str,
    say: &'static str,
    black: Option<u8>,
    white: Option<u8>,
    exposure_us: Option<u32>,
    gain: Option<i32>,
    auto_exposure: Option<bool>,
    configured_exposure_us: Option<u32>,
    configured_gain: Option<i32>,
    configured_auto_exposure: Option<bool>,
    changed_here: Option<bool>,
    settings_say: Option<String>,
    exposure_range_us: Option<[i64; 2]>,
    gain_range: Option<[i64; 2]>,
    can_set: bool,
    setting: &'static str,
    setting_say: Option<String>,
}

#[derive(Debug, Serialize)]
struct FocusStatus {
    verdict: &'static str,
    say: String,
    now_px: Option<f64>,
    best_px: Option<f64>,
    bar: Option<f64>,
    small_print: String,
}

#[derive(Debug, Serialize)]
struct LensStatus {
    calibrated: bool,
    say: String,
}

/// The document as JSON.
pub fn document<'a>(
    cameras: impl IntoIterator<Item = (&'a Camera, Live, Opened)>,
    can_set: bool,
) -> String {
    let document = Document {
        can_set,
        cameras: cameras
            .into_iter()
            .map(|(camera, live, opened)| one(camera, &live, &opened, can_set))
            .collect(),
    };
    serde_json::to_string(&document).expect("plain structs serialize")
}

fn one(camera: &Camera, live: &Live, opened: &Opened, can_set: bool) -> CameraStatus {
    let picture = live.picture.as_ref();
    let checks: Vec<Check> = checks::checks(camera, picture, opened.configured);
    CameraStatus {
        name: camera.name.clone(),
        index: camera.index,
        serial: opened.serial.clone(),
        device: camera.device.display().to_string(),
        width: camera.width,
        height: camera.height,
        fps_asked: camera.fps,
        fps: picture.and_then(|p| p.fps).and_then(thousandths),
        link: picture.map(|p| link_words(p.link)),
        stream: server::stream_path(&camera.name),
        corner: server::corner_path(&camera.name),
        frame: server::frame_path(&camera.name),
        forget: server::focus_path(&camera.name),
        set: server::set_path(&camera.name),
        by_hand: server::settings_path(&camera.name),
        checks: checks
            .into_iter()
            .map(|c| CheckStatus {
                topic: c.topic.word(),
                mark: c.mark.word(),
                say: c.say,
            })
            .collect(),
        brightness: brightness(live, opened, can_set),
        focus: focus(picture),
        lens: lens(camera),
    }
}

/// A link as the page's facts say it: `USB 3 (5 Gb/s), GREY`.
fn link_words(link: LinkState) -> String {
    let speed = match link.speed {
        Speed::Unknown => "USB speed not known",
        Speed::LowOrFull => "USB 1 (12 Mb/s or less)",
        Speed::High => "USB 2 (480 Mb/s)",
        Speed::Super => "USB 3 (5 Gb/s)",
        Speed::Faster => "USB 3 (faster than 5 Gb/s)",
    };
    match link.format {
        Some(format) => format!("{speed}, {}", format.name()),
        None => speed.to_string(),
    }
}

fn brightness(live: &Live, opened: &Opened, can_set: bool) -> BrightnessStatus {
    let picture = live.picture.as_ref();
    let light = picture.and_then(|p| p.light);
    let verdict = match light.map(|l| l.verdict()) {
        None => "no_tag",
        Some(Light::TooDim) => "too_dark",
        Some(Light::Good) => "good",
        Some(Light::TooBright) => "too_bright",
    };
    let in_effect = picture.map(|p| p.in_effect).unwrap_or_default();
    let (setting, setting_say) = match live.setting {
        Setting::Idle => ("idle", None),
        Setting::Dial(state) => {
            let word = match state {
                State::Searching { .. } => "looking",
                State::Steering => "setting",
                State::Settled(_) => "settled",
                State::CouldNot(_) => "could_not",
            };
            (word, Some(words::setting(state, in_effect)))
        }
    };
    let range = |r: Option<Range>| r.map(|r| [r.least, r.most]);
    let ranges = opened.ranges.unwrap_or_default();
    // Against what the configuration gave the camera when it was opened.
    let against = match (opened.configured, picture) {
        (Some(configured), Some(p)) => Some(words::against_configured(configured, p.in_effect)),
        _ => None,
    };
    let configured = opened.configured;
    BrightnessStatus {
        verdict,
        say: words::brightness(light.map(|l| l.verdict())),
        black: light.map(|l| l.black),
        white: light.map(|l| l.white),
        exposure_us: in_effect.exposure_us,
        gain: in_effect.gain,
        auto_exposure: in_effect.auto_exposure,
        configured_exposure_us: configured.and_then(|c| c.exposure_us),
        configured_gain: configured.and_then(|c| c.gain),
        configured_auto_exposure: configured.and_then(|c| c.auto_exposure),
        changed_here: against
            .as_ref()
            .map(|(differs, _)| *differs == words::Settings::ChangedHere),
        settings_say: against.map(|(_, say)| say),
        exposure_range_us: range(ranges.exposure_us),
        gain_range: range(ranges.gain),
        can_set,
        setting,
        setting_say,
    }
}

fn focus(picture: Option<&Picture>) -> FocusStatus {
    let shown = picture.map(|p| p.focus).unwrap_or_default();
    let (verdict, say) = match picture {
        Some(p) => {
            let word = match p.verdict {
                Verdict::Judged(sharpness) => sharpness.word(),
                Verdict::CantJudge(_) => "cant_judge",
            };
            (word, words::focus(p.verdict, p.focus))
        }
        None => ("cant_judge", "No frames from this camera yet.".to_string()),
    };
    FocusStatus {
        verdict,
        say,
        now_px: shown.now_px.and_then(thousandths),
        best_px: shown.best_px.and_then(thousandths),
        bar: share_of_best(shown).and_then(thousandths),
        small_print: words::small_print(shown),
    }
}

fn lens(camera: &Camera) -> LensStatus {
    let say = match camera.lens {
        Some(_) => format!(
            "This camera's configuration has a lens calibration for {} x {}.",
            camera.width, camera.height
        ),
        None => "This camera's configuration has no lens calibration: the robot gets tags' corners only, no distance or pose."
            .to_string(),
    };
    LensStatus {
        calibrated: camera.lens.is_some(),
        say,
    }
}
