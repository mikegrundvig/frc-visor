//! Whether a camera is in focus: an absolute verdict from the width of a tag's edge ([`super::focus`])
//! — sharp, soft, or needs focusing — or why it can't be judged. It goes by the tag the view goes
//! by, and only one whose centre is in the middle part of the frame, since a lens is softer toward
//! its edges; it needs the tag lit well; and it changes only once a reading has held on the other
//! side of a line for a while, so a reading wandering round a line doesn't flap it.

use crate::stream::focus::{Measured, Reading};

/// An edge this wide or narrower is sharp. In this repository's renderer an edge with no blur reads
/// about 1.55 px and with half a pixel of blur about 1.8 (tests/focus.rs), and the detection-limit
/// rows (tests/detection_limits/main.rs) show a pixel of blur, which reads about 2.9 px, costs
/// little even at 6 m. Provisional: from rendered frames, until a real lens has been turned
/// through focus against it.
pub const SHARP_UP_TO_PX: f64 = 2.0;

/// An edge wider than this needs focusing. Two pixels of blur reads about 5.0 px, and at 6 m it
/// makes a far tag's corners about eight times worse than one pixel does (0.493 px against 0.060
/// in tests/detection_limits/main.rs); three pixels loses it. A pixel of blur, about 2.9 px, is
/// still soft rather than unfocused. Provisional, as [`SHARP_UP_TO_PX`].
pub const NEEDS_FOCUS_ABOVE_PX: f64 = 3.0;

/// A tag counts for the verdict when its centre is in this share of the frame round the middle,
/// across and down. A lens is softer toward its edges: a ThriftyCam read tags there at 1.7 to 1.9
/// px while the middle read 1.5.
pub const MIDDLE_SHARE: f64 = 0.5;

/// A reading must be this share past a line to count as across it: a still lens's smoothed reading
/// wanders a few percent.
const ACROSS_BY: f64 = 0.05;

/// And held across it for this many pictures in a row (half a second at the view's 10 a second)
/// before the verdict changes.
const HOLD_PICTURES: u32 = 5;

/// How sharp a camera sees a tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sharpness {
    Sharp,
    Soft,
    NeedsFocusing,
}

/// Why focus can't be judged now.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CantJudge {
    NoTag,
    /// Tags in view, but none with its centre in the middle part of the frame.
    NotInMiddle,
    TooDim,
    TooBright,
    /// Well lit, but too little of its edge could be read.
    Unreadable,
}

/// The focus verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    Judged(Sharpness),
    CantJudge(CantJudge),
}

impl Sharpness {
    /// Which side of the lines a width is.
    pub fn of(width_px: f64) -> Sharpness {
        if width_px <= SHARP_UP_TO_PX {
            Sharpness::Sharp
        } else if width_px <= NEEDS_FOCUS_ABOVE_PX {
            Sharpness::Soft
        } else {
            Sharpness::NeedsFocusing
        }
    }

    /// The word for it in the page's status document.
    pub fn word(&self) -> &'static str {
        match self {
            Sharpness::Sharp => "sharp",
            Sharpness::Soft => "soft",
            Sharpness::NeedsFocusing => "needs_focusing",
        }
    }

    /// Whether a width is clearly outside this verdict: past one of its lines by [`ACROSS_BY`].
    fn left_by(&self, width_px: f64) -> bool {
        let above = |line: f64| width_px > line * (1.0 + ACROSS_BY);
        let below = |line: f64| width_px < line * (1.0 - ACROSS_BY);
        match self {
            Sharpness::Sharp => above(SHARP_UP_TO_PX),
            Sharpness::Soft => below(SHARP_UP_TO_PX) || above(NEEDS_FOCUS_ABOVE_PX),
            Sharpness::NeedsFocusing => below(NEEDS_FOCUS_ABOVE_PX),
        }
    }
}

/// Whether a tag's centre is in the middle part of a frame of this size ([`MIDDLE_SHARE`]).
pub fn in_middle(corners: &[f32; 8], width: u32, height: u32) -> bool {
    let centre = |axis: usize| {
        (0..4)
            .map(|i| f64::from(corners[2 * i + axis]))
            .sum::<f64>()
            / 4.0
    };
    let near = |at: f64, size: u32| {
        let middle = (f64::from(size) - 1.0) / 2.0;
        (at - middle).abs() <= f64::from(size) * MIDDLE_SHARE / 2.0
    };
    near(centre(0), width) && near(centre(1), height)
}

/// Why a frame's reading can't be judged, if it can't: no tag, the tag the view goes by off the
/// middle, or its light or edge.
pub fn why_not(measured: &Measured, centred: bool) -> Option<CantJudge> {
    match measured.reading() {
        Reading::NoTag => Some(CantJudge::NoTag),
        _ if !centred => Some(CantJudge::NotInMiddle),
        Reading::TooDim => Some(CantJudge::TooDim),
        Reading::TooBright => Some(CantJudge::TooBright),
        Reading::Unreadable => Some(CantJudge::Unreadable),
        Reading::Edge(_) => None,
    }
}

/// The verdict as it's shown, steadied: a change waits until the reading has held clearly on the
/// other side of a line for [`HOLD_PICTURES`] pictures.
#[derive(Debug, Default)]
pub struct Judge {
    shown: Option<Sharpness>,
    /// The verdict the reading has been on the other side for, and for how many pictures.
    pending: Option<(Sharpness, u32)>,
}

impl Judge {
    /// Takes a picture's smoothed width, or why there is none, and says the verdict.
    pub fn add(&mut self, reading: Result<f64, CantJudge>) -> Verdict {
        let width = match reading {
            Ok(width) => width,
            Err(why) => {
                // A gap: the next reading is judged afresh.
                *self = Judge::default();
                return Verdict::CantJudge(why);
            }
        };
        let Some(shown) = self.shown else {
            let first = Sharpness::of(width);
            self.shown = Some(first);
            return Verdict::Judged(first);
        };
        if !shown.left_by(width) {
            self.pending = None;
            return Verdict::Judged(shown);
        }
        let other = Sharpness::of(width);
        let held = match self.pending {
            Some((was, pictures)) if was == other => pictures + 1,
            _ => 1,
        };
        if held >= HOLD_PICTURES {
            self.shown = Some(other);
            self.pending = None;
            return Verdict::Judged(other);
        }
        self.pending = Some((other, held));
        Verdict::Judged(shown)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn judged(judge: &mut Judge, widths: &[f64]) -> Vec<Sharpness> {
        widths
            .iter()
            .map(|w| match judge.add(Ok(*w)) {
                Verdict::Judged(s) => s,
                Verdict::CantJudge(why) => panic!("{why:?}"),
            })
            .collect()
    }

    #[test]
    fn a_width_s_verdict_is_by_the_lines() {
        // Each row: a width, and its verdict.
        let rows = [
            (1.5, Sharpness::Sharp),
            (2.0, Sharpness::Sharp),
            (2.01, Sharpness::Soft),
            (2.9, Sharpness::Soft),
            (3.0, Sharpness::Soft),
            (3.01, Sharpness::NeedsFocusing),
            (5.0, Sharpness::NeedsFocusing),
        ];
        for (width, verdict) in rows {
            assert_eq!(Sharpness::of(width), verdict, "{width}");
        }
    }

    #[test]
    fn a_reading_wandering_round_a_line_doesn_t_flap_the_verdict_but_a_real_change_turns_it() {
        use Sharpness::*;
        // Each row: what it's about, the widths in turn, and the verdict after the last.
        let wander = |line: f64, by: f64| -> Vec<f64> {
            (0..60)
                .map(|i| line * (1.0 + by * (f64::from(i) * 0.7).sin()))
                .collect()
        };
        let rows: Vec<(&str, Vec<f64>, Sharpness)> = vec![
            (
                "4% round the sharp line, from sharp",
                {
                    let mut w = vec![1.9];
                    w.extend(wander(SHARP_UP_TO_PX, 0.04));
                    w
                },
                Sharp,
            ),
            (
                "4% round the sharp line, from soft",
                {
                    let mut w = vec![2.3];
                    w.extend(wander(SHARP_UP_TO_PX, 0.04));
                    w
                },
                Soft,
            ),
            (
                "4% round the focus line, from soft",
                {
                    let mut w = vec![2.8];
                    w.extend(wander(NEEDS_FOCUS_ABOVE_PX, 0.04));
                    w
                },
                Soft,
            ),
            (
                "4% round the focus line, from unfocused",
                {
                    let mut w = vec![3.3];
                    w.extend(wander(NEEDS_FOCUS_ABOVE_PX, 0.04));
                    w
                },
                NeedsFocusing,
            ),
            (
                "turned into focus and held",
                {
                    let mut w = vec![4.0; 3];
                    w.extend([3.0, 2.5, 2.0, 1.8, 1.6, 1.55, 1.55, 1.55, 1.55, 1.55]);
                    w
                },
                Sharp,
            ),
            (
                "past it for a moment only",
                {
                    let mut w = vec![1.6; 3];
                    w.extend([2.4, 2.4, 2.4, 1.6, 1.6]);
                    w
                },
                Sharp,
            ),
        ];
        for (what, widths, after) in rows {
            let mut judge = Judge::default();
            let verdicts = judged(&mut judge, &widths);
            assert_eq!(*verdicts.last().unwrap(), after, "{what}: {verdicts:?}");
            // Never flipped back and forth: each change is to a new verdict that then holds.
            let changes = verdicts.windows(2).filter(|p| p[0] != p[1]).count();
            assert!(changes <= 3, "{what}: {verdicts:?}");
            if what.starts_with("4%") {
                assert_eq!(changes, 0, "{what}: {verdicts:?}");
            }
        }
        // The change itself waits the hold: four pictures across say nothing yet, the fifth does.
        let mut judge = Judge::default();
        let verdicts = judged(&mut judge, &[1.5, 2.5, 2.5, 2.5, 2.5, 2.5]);
        assert_eq!(verdicts, [Sharp, Sharp, Sharp, Sharp, Sharp, Soft]);
        // A gap starts afresh: the next reading is the verdict at once.
        assert_eq!(
            judge.add(Err(CantJudge::TooBright)),
            Verdict::CantJudge(CantJudge::TooBright)
        );
        assert_eq!(judge.add(Ok(4.0)), Verdict::Judged(NeedsFocusing));
    }

    #[test]
    fn only_a_tag_whose_centre_is_in_the_middle_part_counts() {
        let square = |x: f32, y: f32| {
            [
                x - 20.0,
                y - 20.0,
                x + 20.0,
                y - 20.0,
                x + 20.0,
                y + 20.0,
                x - 20.0,
                y + 20.0,
            ]
        };
        // Each row: the tag's centre in a 1600 x 1304 frame, and whether it counts.
        let rows = [
            ((800.0, 652.0), true),
            ((400.0, 652.0), true),
            ((399.0, 652.0), false),
            ((1199.0, 652.0), true),
            ((1210.0, 652.0), false),
            ((800.0, 320.0), false),
            ((800.0, 330.0), true),
            ((100.0, 100.0), false),
        ];
        for ((x, y), counts) in rows {
            assert_eq!(in_middle(&square(x, y), 1600, 1304), counts, "{x}, {y}");
        }
    }
}
