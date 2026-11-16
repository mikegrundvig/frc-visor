//! A camera's link state, as every frame carries it in one byte (docs/wire.md, offset 27): the
//! USB link's speed class, the pixel format in use, whether the camera looks able to do better
//! than its link, and whether a recovery was tried and left it slow ([`crate::recovery`]). 0 is
//! "not known" throughout, so a frame from a Visor before this byte reads as unknown.
//!
//! | Bits | Field | Values |
//! |---|---|---|
//! | 0-2 | Speed | 0 unknown, 1 low or full speed (12 Mb/s or less), 2 high speed (480), 3 SuperSpeed (5000), 4 faster |
//! | 3-4 | Format | 0 unknown, 1 GREY, 2 YUYV, 3 MJPG |
//! | 5 | Headroom | 1: below SuperSpeed and able to do better (a USB 3 camera on USB 2) |
//! | 6 | Recovery | 1: a reset or power cycle was tried since the camera last appeared, and left it below SuperSpeed |
//! | 7 | | 0 |

use crate::config::PixelFormat;
use crate::recovery::Recovered;
use crate::usb::{Link, Outlook, SUPER_SPEED_MBPS};

const SPEED_MASK: u8 = 0b111;
const FORMAT_SHIFT: u8 = 3;
const FORMAT_MASK: u8 = 0b11 << FORMAT_SHIFT;
const HEADROOM: u8 = 1 << 5;
const RECOVERY: u8 = 1 << 6;

/// A camera's link and format as its frames carry them.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LinkState {
    pub speed: Speed,
    /// The pixel format in use; none when not known.
    pub format: Option<PixelFormat>,
    pub headroom: Headroom,
    pub recovered: Recovered,
}

/// The USB link's speed class.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Speed {
    /// Not known: not on USB, unreadable, or an older Visor.
    #[default]
    Unknown,
    /// USB 1's low or full speed: 12 Mb/s or less.
    LowOrFull,
    /// USB 2's high speed, 480 Mb/s.
    High,
    /// USB 3's SuperSpeed, 5000 Mb/s.
    Super,
    /// Faster than SuperSpeed.
    Faster,
}

/// Whether the camera looks able to do better than the link it's on.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Headroom {
    /// At its best, or not known.
    #[default]
    NoneKnown,
    /// Below SuperSpeed while reporting USB 2.10 or later: a USB 3 camera on USB 2.
    MayDoBetter,
}

impl Speed {
    /// The class of a link at `mbps`.
    pub fn of_mbps(mbps: u32) -> Speed {
        match mbps {
            0 => Speed::Unknown,
            1..=12 => Speed::LowOrFull,
            13..SUPER_SPEED_MBPS => Speed::High,
            SUPER_SPEED_MBPS => Speed::Super,
            _ => Speed::Faster,
        }
    }

    fn bits(self) -> u8 {
        match self {
            Speed::Unknown => 0,
            Speed::LowOrFull => 1,
            Speed::High => 2,
            Speed::Super => 3,
            Speed::Faster => 4,
        }
    }

    fn from_bits(bits: u8) -> Speed {
        match bits {
            1 => Speed::LowOrFull,
            2 => Speed::High,
            3 => Speed::Super,
            4 => Speed::Faster,
            _ => Speed::Unknown,
        }
    }
}

impl LinkState {
    /// The state of a camera on `link` (none: not known) in `format`, with what recovery left.
    pub fn of(link: Option<&Link>, format: PixelFormat, recovered: Recovered) -> LinkState {
        let Some(link) = link else {
            return LinkState {
                format: Some(format),
                ..LinkState::default()
            };
        };
        let headroom = match link.outlook() {
            Outlook::MayDoBetter => Headroom::MayDoBetter,
            Outlook::Fast | Outlook::AtItsBest => Headroom::NoneKnown,
        };
        LinkState {
            speed: Speed::of_mbps(link.mbps),
            format: Some(format),
            headroom,
            recovered,
        }
    }

    /// Its byte.
    pub fn byte(self) -> u8 {
        let format = match self.format {
            None => 0,
            Some(PixelFormat::Grey) => 1,
            Some(PixelFormat::Yuyv) => 2,
            Some(PixelFormat::Mjpg) => 3,
        };
        let mut byte = self.speed.bits() | format << FORMAT_SHIFT;
        if self.headroom == Headroom::MayDoBetter {
            byte |= HEADROOM;
        }
        if self.recovered == Recovered::TriedStillSlow {
            byte |= RECOVERY;
        }
        byte
    }

    /// The state a byte says. A value it doesn't know reads as unknown.
    pub fn from_byte(byte: u8) -> LinkState {
        let format = match (byte & FORMAT_MASK) >> FORMAT_SHIFT {
            1 => Some(PixelFormat::Grey),
            2 => Some(PixelFormat::Yuyv),
            3 => Some(PixelFormat::Mjpg),
            _ => None,
        };
        LinkState {
            speed: Speed::from_bits(byte & SPEED_MASK),
            format,
            headroom: if byte & HEADROOM != 0 {
                Headroom::MayDoBetter
            } else {
                Headroom::NoneKnown
            },
            recovered: if byte & RECOVERY != 0 {
                Recovered::TriedStillSlow
            } else {
                Recovered::NotTried
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::Version;

    fn link(mbps: u32, major: u8, minor: u8) -> Link {
        Link {
            name: "8-1".into(),
            mbps,
            version: Version { major, minor },
            bus: 8,
            address: 2,
            controller: None,
        }
    }

    #[test]
    fn a_camera_s_link_state_is_one_byte_with_zero_unknown() {
        use PixelFormat::{Grey, Mjpg, Yuyv};
        use Recovered::{NotTried, TriedStillSlow};
        let (usb_3, stuck, usb_2, full) = (
            link(5000, 3, 10),
            link(480, 3, 10),
            link(480, 2, 0),
            link(12, 3, 10),
        );
        // (the link, the format, what recovery left, the byte: recovery 0x40, headroom 0x20,
        // format 0x08 to 0x18, speed 0x01 to 0x04)
        let cases = [
            (Some(&usb_3), Grey, NotTried, 0x0b),
            (Some(&link(10000, 3, 20)), Grey, NotTried, 0x0c),
            (Some(&stuck), Mjpg, NotTried, 0x3a),
            (Some(&stuck), Mjpg, TriedStillSlow, 0x7a),
            (Some(&usb_2), Yuyv, NotTried, 0x12),
            (Some(&full), Mjpg, TriedStillSlow, 0x79),
            (None, Grey, NotTried, 0x08),
        ];
        for (link, format, recovered, byte) in cases {
            let state = LinkState::of(link, format, recovered);
            assert_eq!(state.byte(), byte, "{state:?}");
            assert_eq!(LinkState::from_byte(byte), state, "{byte:#010b}");
        }
        // An older Visor's 0, and values this one doesn't know, read as unknown.
        assert_eq!(LinkState::from_byte(0), LinkState::default());
        assert_eq!(LinkState::from_byte(0b1000_0111).speed, Speed::Unknown);
        let speeds = [
            (0, Speed::Unknown),
            (1, Speed::LowOrFull),
            (12, Speed::LowOrFull),
            (480, Speed::High),
            (5000, Speed::Super),
            (20000, Speed::Faster),
        ];
        for (mbps, speed) in speeds {
            assert_eq!(Speed::of_mbps(mbps), speed, "{mbps} Mb/s");
        }
    }
}
