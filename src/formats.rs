//! Which pixel format a camera runs: the configured one, or else the first of GREY, YUYV and MJPG
//! that gives the configured size at the configured frame rate.
//!
//! GREY is the luma itself; YUYV carries it in twice the bytes; MJPG must be decoded first (about
//! 8.5 ms a 1600 x 1304 frame on a coprocessor's fast core, on the capture thread). They're
//! equal for detection, so the order is the cost. But a camera doesn't offer every format at every
//! rate: a USB 3 camera on USB 2 offers its full size uncompressed only at a few frames a second,
//! and compressed at its full rate. So the rate decides first, then the order.

use v4l::fraction::Fraction;

use crate::config::PixelFormat;

/// Visor's formats, in the order it prefers them.
pub const PREFERRED: [PixelFormat; 3] = [PixelFormat::Grey, PixelFormat::Yuyv, PixelFormat::Mjpg];

/// The frame rates a camera offers for one format at the configured size.
#[derive(Debug, Clone, PartialEq)]
pub enum Rates {
    /// It doesn't offer the format at this size.
    NotAtThisSize,
    /// It offers the size, but not which rates (its driver doesn't list them).
    Unknown,
    /// These frame rates, each in frames a second; a range offered as its two ends.
    Offered(Vec<f64>),
}

impl Rates {
    /// The rates of the frame intervals a driver lists, each a fraction of a second.
    pub fn of_intervals(intervals: &[Fraction]) -> Rates {
        Rates::Offered(intervals.iter().copied().filter_map(fps).collect())
    }

    /// The fastest rate offered, if any is known.
    fn fastest(&self) -> Option<f64> {
        match self {
            Rates::Offered(rates) => rates.iter().copied().reduce(f64::max),
            Rates::NotAtThisSize | Rates::Unknown => None,
        }
    }

    /// Whether it offers `wanted` frames a second or faster. A camera's own rates are rounded
    /// (1/60 s is 166666/10000000), so within 1% counts.
    fn keeps_up_with(&self, wanted: u32) -> bool {
        self.fastest()
            .is_some_and(|fastest| fastest >= f64::from(wanted) * 0.99)
    }
}

/// The frame rate of an interval; none for an interval of no time.
pub fn fps(interval: Fraction) -> Option<f64> {
    if interval.numerator == 0 {
        return None;
    }
    Some(f64::from(interval.denominator) / f64::from(interval.numerator))
}

/// A format chosen, and why.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Choice {
    pub format: PixelFormat,
    pub why: Why,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Why {
    /// The configuration names it.
    Configured,
    /// It's the first that gives the size at the configured rate.
    AtTheRate,
    /// None gives the configured rate at this size: it's the fastest that does, at this rate.
    Fastest(f64),
    /// No rates are known: the first offered, as the camera's driver doesn't say.
    FirstOffered,
}

/// Chooses the pixel format from what the camera offers (`all`, every format by its four
/// characters; `rated`, each of Visor's formats among them with its rates at the configured size),
/// the configured format (`wanted`), and the configured rate (`fps`).
///
/// A configured format wins whatever its rate, if it's offered. Otherwise the first of
/// [`PREFERRED`] that offers the size at `fps` or faster; else the fastest at that size (the
/// earlier on a tie); else, with no rates known, the first offered. A camera that offers the size
/// in none of them still gets the first offered, so its refusal of the size is what's said.
pub fn choose(
    all: &[[u8; 4]],
    rated: &[(PixelFormat, Rates)],
    wanted: Option<PixelFormat>,
    fps: u32,
) -> Result<Choice, String> {
    if let Some(format) = wanted {
        if rated.iter().any(|(each, _)| *each == format) {
            return Ok(Choice {
                format,
                why: Why::Configured,
            });
        }
        return Err(format!("{}, not {}", offers(all), format.name()));
    }
    let in_order: Vec<&(PixelFormat, Rates)> = PREFERRED
        .iter()
        .filter_map(|format| rated.iter().find(|(each, _)| each == format))
        .collect();
    if let Some((format, _)) = in_order.iter().find(|(_, rates)| rates.keeps_up_with(fps)) {
        return Ok(Choice {
            format: *format,
            why: Why::AtTheRate,
        });
    }
    let mut fastest: Option<(PixelFormat, f64)> = None;
    for (format, rates) in &in_order {
        let Some(rate) = rates.fastest() else {
            continue;
        };
        if fastest.is_none_or(|(_, best)| rate > best) {
            fastest = Some((*format, rate));
        }
    }
    if let Some((format, rate)) = fastest {
        return Ok(Choice {
            format,
            why: Why::Fastest(rate),
        });
    }
    match in_order.first() {
        Some((format, _)) => Ok(Choice {
            format: *format,
            why: Why::FirstOffered,
        }),
        None => Err(format!("{}, not GREY, YUYV or MJPG", offers(all))),
    }
}

/// What the camera offers, as a refusal says it.
fn offers(all: &[[u8; 4]]) -> String {
    if all.is_empty() {
        return "the camera offers no formats".into();
    }
    let names: Vec<String> = all
        .iter()
        .map(|f| String::from_utf8_lossy(f).into_owned())
        .collect();
    format!("the camera offers {}", names.join(", "))
}

#[cfg(test)]
mod tests {
    use super::*;
    use PixelFormat::{Grey, Mjpg, Yuyv};

    fn at(rates: &[f64]) -> Rates {
        Rates::Offered(rates.to_vec())
    }

    /// A row: its name, what the camera offers, the format configured, the rate configured, and
    /// the choice or part of the refusal.
    type ChoiceCase<'a> = (
        &'a str,
        &'a [(PixelFormat, Rates)],
        Option<PixelFormat>,
        u32,
        Result<Choice, &'a str>,
    );

    #[test]
    fn the_format_is_the_configured_one_else_the_first_at_the_rate_else_the_fastest() {
        // A USB 3 camera at USB 3, and the same camera on USB 2, at 1600 x 1304.
        let usb_3 = vec![
            (Mjpg, at(&[70.0, 60.0])),
            (Grey, at(&[70.0, 60.0])),
            (Yuyv, at(&[70.0, 60.0, 30.0])),
        ];
        let usb_2 = vec![(Mjpg, at(&[60.0])), (Grey, at(&[5.0]))];
        let slow_everywhere = vec![(Grey, at(&[5.0])), (Yuyv, at(&[10.0])), (Mjpg, at(&[10.0]))];
        let size_only_in_mjpg = vec![(Grey, Rates::NotAtThisSize), (Mjpg, at(&[30.0]))];
        let rates_unknown = vec![(Yuyv, Rates::Unknown), (Grey, Rates::Unknown)];
        let size_nowhere = vec![(Yuyv, Rates::NotAtThisSize), (Grey, Rates::NotAtThisSize)];
        let rounded = vec![(
            Grey,
            Rates::of_intervals(&[Fraction::new(166_666, 10_000_000)]),
        )];
        let cases: [ChoiceCase; 11] = [
            (
                "USB 3: GREY",
                &usb_3,
                None,
                60,
                Ok(Choice {
                    format: Grey,
                    why: Why::AtTheRate,
                }),
            ),
            (
                "USB 3 at 70: GREY",
                &usb_3,
                None,
                70,
                Ok(Choice {
                    format: Grey,
                    why: Why::AtTheRate,
                }),
            ),
            (
                "USB 2: MJPG, the only one at 60",
                &usb_2,
                None,
                60,
                Ok(Choice {
                    format: Mjpg,
                    why: Why::AtTheRate,
                }),
            ),
            (
                "USB 2 at 5: GREY keeps up",
                &usb_2,
                None,
                5,
                Ok(Choice {
                    format: Grey,
                    why: Why::AtTheRate,
                }),
            ),
            (
                "USB 2, GREY configured: GREY",
                &usb_2,
                Some(Grey),
                60,
                Ok(Choice {
                    format: Grey,
                    why: Why::Configured,
                }),
            ),
            (
                "none at the rate: the fastest, the earlier on a tie",
                &slow_everywhere,
                None,
                60,
                Ok(Choice {
                    format: Yuyv,
                    why: Why::Fastest(10.0),
                }),
            ),
            (
                "the size only compressed",
                &size_only_in_mjpg,
                None,
                60,
                Ok(Choice {
                    format: Mjpg,
                    why: Why::Fastest(30.0),
                }),
            ),
            (
                "rates unknown: the first in order",
                &rates_unknown,
                None,
                60,
                Ok(Choice {
                    format: Grey,
                    why: Why::FirstOffered,
                }),
            ),
            (
                "the size in none: the first, to be refused by size",
                &size_nowhere,
                None,
                60,
                Ok(Choice {
                    format: Grey,
                    why: Why::FirstOffered,
                }),
            ),
            (
                "a rate rounded by the camera",
                &rounded,
                None,
                60,
                Ok(Choice {
                    format: Grey,
                    why: Why::AtTheRate,
                }),
            ),
            (
                "YUYV configured, not offered",
                &usb_2,
                Some(Yuyv),
                60,
                Err("the camera offers MJPG, GREY, not YUYV"),
            ),
        ];
        for (name, offered, wanted, fps, expected) in cases {
            let all: Vec<[u8; 4]> = offered.iter().map(|(f, _)| f.fourcc()).collect();
            match (choose(&all, offered, wanted, fps), expected) {
                (Ok(chosen), Ok(choice)) => assert_eq!(chosen, choice, "{name}"),
                (Err(why), Err(part)) => assert!(why.contains(part), "{name}: {why}"),
                (got, expected) => panic!("{name}: {got:?}, expected {expected:?}"),
            }
        }
        assert_eq!(
            choose(&[*b"H264"], &[], None, 60).unwrap_err(),
            "the camera offers H264, not GREY, YUYV or MJPG"
        );
        assert_eq!(
            choose(&[], &[], Some(Grey), 60).unwrap_err(),
            "the camera offers no formats, not GREY"
        );
    }
}
