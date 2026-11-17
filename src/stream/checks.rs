//! A camera's checks for the page's cameras screen: its connection, frame rate, brightness, focus,
//! lens calibration and clock, each a plain sentence marked as needing attention, not yet known,
//! or fine, in that order.

use crate::config::Camera;
use crate::link_state::{Headroom, Speed};
use crate::recovery::Recovered;
use crate::settings::InEffect;
use crate::stream::feed::Clock;
use crate::stream::light::Light;
use crate::stream::sharpness::{Sharpness, Verdict};
use crate::stream::status::Picture;
use crate::stream::words::{self, Settings};

/// What a check is of.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Topic {
    Connection,
    FrameRate,
    Brightness,
    Focus,
    Lens,
    Clock,
    /// The exposure and gain against the configuration's: information, not a fault.
    Settings,
}

/// How a check stands, in the order the page lists them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Mark {
    Attention,
    /// Can't be told yet: no frames, or no tag to judge by.
    Unknown,
    /// Information worth knowing, not a fault.
    Note,
    Fine,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Check {
    pub topic: Topic,
    pub mark: Mark,
    pub say: String,
}

impl Topic {
    pub fn word(&self) -> &'static str {
        match self {
            Topic::Connection => "connection",
            Topic::FrameRate => "frame_rate",
            Topic::Brightness => "brightness",
            Topic::Focus => "focus",
            Topic::Lens => "lens",
            Topic::Clock => "clock",
            Topic::Settings => "settings",
        }
    }
}

impl Mark {
    pub fn word(&self) -> &'static str {
        match self {
            Mark::Attention => "attention",
            Mark::Unknown => "unknown",
            Mark::Note => "note",
            Mark::Fine => "fine",
        }
    }
}

/// A camera's checks, attention first, from its newest picture's facts (none: no frames yet), and
/// the exposure and gain its configuration gave it when it was opened.
pub fn checks(
    camera: &Camera,
    picture: Option<&Picture>,
    configured: Option<InEffect>,
) -> Vec<Check> {
    let check = |topic, mark, say: &str| Check {
        topic,
        mark,
        say: say.to_string(),
    };
    let mut list = match picture {
        Some(picture) => vec![
            connection(picture),
            frame_rate(camera, picture),
            brightness(picture),
            focus(picture),
            clock(picture),
        ],
        None => vec![
            check(
                Topic::Connection,
                Mark::Unknown,
                "No frames from this camera yet: is it plugged in?",
            ),
            check(Topic::FrameRate, Mark::Unknown, "No frames yet."),
            check(Topic::Brightness, Mark::Unknown, "No frames yet."),
            check(Topic::Focus, Mark::Unknown, "No frames yet."),
            check(Topic::Clock, Mark::Unknown, "No frames yet."),
        ],
    };
    list.push(lens(camera));
    if let (Some(configured), Some(picture)) = (configured, picture)
        && let (Settings::ChangedHere, say) =
            words::against_configured(configured, picture.in_effect)
    {
        list.push(Check {
            topic: Topic::Settings,
            mark: Mark::Note,
            say,
        });
    }
    // Stable: within a mark, the order above.
    list.sort_by_key(|c| c.mark);
    list
}

fn connection(picture: &Picture) -> Check {
    let link = picture.link;
    let (mark, say) = match (link.speed, link.headroom) {
        (Speed::Super | Speed::Faster, _) => (Mark::Fine, "Connected at USB 3 speed.".to_string()),
        (Speed::High, Headroom::MayDoBetter) => {
            let tried = match link.recovered {
                Recovered::TriedStillSlow => " Visor reset it and it stayed slow.",
                Recovered::NotTried => "",
            };
            (
                Mark::Attention,
                format!(
                    "On USB 2 though it can run at USB 3: reseat its plug or change its cable.{tried}"
                ),
            )
        }
        (Speed::High, Headroom::NoneKnown) => (Mark::Fine, "Connected at USB 2 speed.".to_string()),
        (Speed::LowOrFull, _) => (
            Mark::Attention,
            "On a slow USB 1 link: plug it into another port.".to_string(),
        ),
        (Speed::Unknown, _) => (
            Mark::Fine,
            "Sending frames (its USB speed isn't known).".to_string(),
        ),
    };
    Check {
        topic: Topic::Connection,
        mark,
        say,
    }
}

fn frame_rate(camera: &Camera, picture: &Picture) -> Check {
    let asked = camera.fps;
    let (mark, say) = match picture.fps {
        None => (Mark::Unknown, "Measuring the frame rate.".to_string()),
        Some(fps) if fps >= f64::from(asked) * words::SLOWED_BELOW => (
            Mark::Fine,
            format!("{fps:.0} frames a second, as asked for."),
        ),
        Some(fps) => {
            let say = words::slowed(Some(fps), asked, picture.in_effect.exposure_us)
                .unwrap_or_else(|| {
                    format!(
                        "{fps:.0} frames a second, slower than the {asked} asked for: check its USB link."
                    )
                });
            (Mark::Attention, say)
        }
    };
    Check {
        topic: Topic::FrameRate,
        mark,
        say,
    }
}

fn brightness(picture: &Picture) -> Check {
    let light = picture.light.map(|l| l.verdict());
    let mark = match light {
        None => Mark::Unknown,
        Some(Light::Good) => Mark::Fine,
        Some(Light::TooDim | Light::TooBright) => Mark::Attention,
    };
    Check {
        topic: Topic::Brightness,
        mark,
        say: words::brightness(light).to_string(),
    }
}

fn focus(picture: &Picture) -> Check {
    let mark = match picture.verdict {
        Verdict::Judged(Sharpness::Sharp) => Mark::Fine,
        Verdict::Judged(_) => Mark::Attention,
        Verdict::CantJudge(_) => Mark::Unknown,
    };
    Check {
        topic: Topic::Focus,
        mark,
        say: words::focus(picture.verdict, picture.focus),
    }
}

fn lens(camera: &Camera) -> Check {
    let (mark, say) = match camera.lens {
        Some(_) => (
            Mark::Fine,
            format!(
                "Lens calibrated at {} x {}.",
                camera.width, camera.height
            ),
        ),
        None => (
            Mark::Attention,
            "No lens calibration: the robot gets tags' corners only, no distance or pose. Calibrate after focusing."
                .to_string(),
        ),
    };
    Check {
        topic: Topic::Lens,
        mark,
        say,
    }
}

fn clock(picture: &Picture) -> Check {
    let (mark, say) = match picture.clock {
        Clock::Synced => (Mark::Fine, "Clock synced to the robot."),
        Clock::NotSynced => (
            Mark::Attention,
            "Clock not synced to the robot: frame times won't match the robot's.",
        ),
    };
    Check {
        topic: Topic::Clock,
        mark,
        say: say.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::link_state::LinkState;
    use crate::stream::focus::Shown;
    use crate::stream::light::TagLight;
    use crate::stream::sharpness::CantJudge;

    fn camera() -> Camera {
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
            exposure_us: Some(4000),
            gain: Some(16),
            controls: Vec::new(),
            lens: None,
            tag_size_m: 0.1651,
            stream: crate::config::Stream::default(),
            config_id: 0,
        }
    }

    fn picture() -> Picture {
        Picture {
            fps: Some(60.1),
            in_effect: InEffect {
                exposure_us: Some(4000),
                gain: Some(16),
                auto_exposure: Some(false),
            },
            link: LinkState {
                speed: Speed::Super,
                ..LinkState::default()
            },
            clock: Clock::Synced,
            light: Some(TagLight {
                black: 20,
                white: 130,
            }),
            focus: Shown::default(),
            verdict: Verdict::Judged(Sharpness::Sharp),
        }
    }

    fn said(list: &[Check], topic: Topic) -> (Mark, String) {
        let check = list.iter().find(|c| c.topic == topic).unwrap();
        (check.mark, check.say.clone())
    }

    #[test]
    fn each_check_says_what_is_fine_and_what_needs_attention_attention_first() {
        // Each row: what's changed from a camera that's all fine but its lens, the check it
        // changes, its mark and part of what it says.
        type Change = fn(&mut Picture);
        let rows: [(&str, Change, Topic, Mark, &str); 10] = [
            ("all fine", |_| {}, Topic::Connection, Mark::Fine, "USB 3"),
            (
                "a USB 3 camera on USB 2",
                |p| {
                    p.link.speed = Speed::High;
                    p.link.headroom = Headroom::MayDoBetter;
                },
                Topic::Connection,
                Mark::Attention,
                "reseat its plug",
            ),
            (
                "slowed by a long exposure",
                |p| {
                    p.fps = Some(30.1);
                    p.in_effect.exposure_us = Some(33_000);
                },
                Topic::FrameRate,
                Mark::Attention,
                "slowed to 30 frames",
            ),
            (
                "slow for another reason",
                |p| p.fps = Some(40.0),
                Topic::FrameRate,
                Mark::Attention,
                "check its USB link",
            ),
            (
                "too bright",
                |p| {
                    p.light = Some(TagLight {
                        black: 60,
                        white: 240,
                    })
                },
                Topic::Brightness,
                Mark::Attention,
                "Too bright",
            ),
            (
                "no tag",
                |p| p.light = None,
                Topic::Brightness,
                Mark::Unknown,
                "No tag in view",
            ),
            (
                "soft",
                |p| p.verdict = Verdict::Judged(Sharpness::Soft),
                Topic::Focus,
                Mark::Attention,
                "Soft",
            ),
            (
                "focus not judged",
                |p| p.verdict = Verdict::CantJudge(CantJudge::NotInMiddle),
                Topic::Focus,
                Mark::Unknown,
                "near the middle",
            ),
            (
                "clock not synced",
                |p| p.clock = Clock::NotSynced,
                Topic::Clock,
                Mark::Attention,
                "not synced",
            ),
            ("sharp", |_| {}, Topic::Focus, Mark::Fine, "Sharp"),
        ];
        for (what, change, topic, mark, says) in rows {
            let mut p = picture();
            change(&mut p);
            // What the configuration gave is what's in effect: nothing changed here.
            let list = checks(&camera(), Some(&p), Some(p.in_effect));
            assert_eq!(list.len(), 6, "{what}");
            let (got, say) = said(&list, topic);
            assert_eq!(got, mark, "{what}: {say}");
            assert!(say.contains(says), "{what}: {say}");
            let marks: Vec<Mark> = list.iter().map(|c| c.mark).collect();
            let mut sorted = marks.clone();
            sorted.sort();
            assert_eq!(marks, sorted, "{what}: attention first");
        }
        // The lens: missing is attention, with what it costs; present is fine.
        let list = checks(&camera(), Some(&picture()), Some(picture().in_effect));
        let (mark, say) = said(&list, Topic::Lens);
        assert_eq!(mark, Mark::Attention);
        assert!(say.contains("no distance or pose"), "{say}");
        // No frames yet: nothing about the camera itself is known.
        let list = checks(&camera(), None, None);
        assert_eq!(said(&list, Topic::Clock).0, Mark::Unknown);
        assert!(said(&list, Topic::Connection).1.contains("plugged in"));
        // Settings changed here: a note, after any fault and before what's fine.
        let mut changed = picture();
        changed.in_effect.gain = Some(52);
        let configured = InEffect {
            gain: Some(160),
            ..picture().in_effect
        };
        let list = checks(&camera(), Some(&changed), Some(configured));
        let (mark, say) = said(&list, Topic::Settings);
        assert_eq!(mark, Mark::Note);
        assert!(
            say.contains("Gain 52, changed here from the configured 160"),
            "{say}"
        );
        let at = list
            .iter()
            .position(|c| c.topic == Topic::Settings)
            .unwrap();
        assert!(list[..at].iter().all(|c| c.mark != Mark::Fine));
        assert!(list[at + 1..].iter().all(|c| c.mark == Mark::Fine));
    }
}
