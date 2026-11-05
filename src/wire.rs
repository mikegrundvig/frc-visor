//! The datagram Visor sends the robot for each frame. The layout is in docs/wire.md: little-endian,
//! fixed sizes, no padding, a 48-byte header and 96 bytes per tag.

use std::fmt;

use crate::brightness::Brightness;

/// "VISR" in byte order, so a stray packet on the port is dropped, not misread.
pub const MAGIC: u32 = 0x5253_4956;
pub const HEADER_BYTES: usize = 48;
pub const TAG_BYTES: usize = 96;
/// One Ethernet packet's UDP payload, in bytes: a frame is never split across packets.
pub const PACKET_BYTES: usize = 1472;
/// The most tags one frame carries: as many as fit one packet.
pub const MAX_TAGS: usize = 14;
// Checked as the program builds: MAX_TAGS fit, and one more wouldn't.
const _: () = assert!(HEADER_BYTES + MAX_TAGS * TAG_BYTES <= PACKET_BYTES);
const _: () = assert!(HEADER_BYTES + (MAX_TAGS + 1) * TAG_BYTES > PACKET_BYTES);

/// This build's version ID, from its version: 2026.11.2 is 20261102.
pub const VERSION_ID: u32 = parse_version_id(env!("VISOR_VERSION_ID"));

const FLAG_CLOCK_SYNCED: u8 = 1 << 0;
const FLAG_DEQUEUE_TIME: u8 = 1 << 1;
const FLAG_EXPOSURE: u8 = 1 << 2;
const FLAG_GAIN: u8 = 1 << 3;
const FLAG_AUTO_EXPOSURE: u8 = 1 << 4;
const TAG_FLAG_POSES: u8 = 1 << 0;

/// One frame from one camera, and every tag NAPA found in it.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub version_id: u32,
    /// Counts this camera's frames from program start; a gap is a dropped frame.
    pub sequence: u32,
    pub width: u16,
    pub height: u16,
    /// When the image was captured, in nanoseconds on the robot's clock.
    pub capture_nanos: i64,
    /// The camera's index, unique on the robot.
    pub camera: u8,
    /// Whether chrony had this computer's clock synced to the robot's.
    pub clock_synced: bool,
    /// Whether the capture time is when Visor got the frame, because the driver gave no timestamp.
    pub dequeue_time: bool,
    /// The exposure the image was taken at, in microseconds, as read back from the camera; none
    /// while the camera chose it (its automatic exposure on), or where it couldn't be read.
    pub exposure_us: Option<u32>,
    /// The camera's `gain` control as read back; none where it has none.
    pub gain: Option<i32>,
    /// Whether the camera's own automatic exposure was on.
    pub auto_exposure: bool,
    /// How bright the image is.
    pub brightness: Brightness,
    /// How long finding this frame's tags and solving their poses took, in microseconds.
    pub detect_us: u32,
    /// The camera's frames Visor never sent, counted from its start: replaced by a newer one before
    /// detection got to them, or broken. It wraps at `u32::MAX`, as the sequence does.
    pub dropped: u32,
    pub tags: Vec<Tag>,
}

/// One tag in a frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tag {
    pub id: u16,
    /// Payload bits NAPA corrected.
    pub hamming: u8,
    pub decision_margin: f32,
    /// x0, y0 … x3, y3: OpenCV's corner order, in pixel-centre coordinates.
    pub corners: [f32; 8],
    /// The tag's two poses, best first, when the camera has a lens calibration and the corners
    /// could be solved.
    pub poses: Option<[Pose; 2]>,
}

/// One of a tag's two poses: the tag in the camera's frame, OpenCV's axes (x right, y down, z out
/// of the lens).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose {
    /// The rotation as an axis scaled by its angle, radians (OpenCV's rvec).
    pub rvec: [f32; 3],
    /// The tag's centre, metres (OpenCV's tvec).
    pub tvec: [f32; 3],
    /// Root-mean-square reprojection error per coordinate, pixels.
    pub error_px: f32,
}

/// Why a datagram isn't a frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WireError {
    TooShort(usize),
    NotVisor(u32),
    TooManyTags(usize),
    WrongLength { expected: usize, actual: usize },
}

impl fmt::Display for WireError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            WireError::TooShort(n) => write!(f, "{n} bytes is shorter than a frame's header"),
            WireError::NotVisor(magic) => write!(f, "not a Visor frame (magic {magic:#010x})"),
            WireError::TooManyTags(n) => write!(f, "{n} tags is more than a frame carries"),
            WireError::WrongLength { expected, actual } => {
                write!(
                    f,
                    "a frame of its tag count is {expected} bytes, not {actual}"
                )
            }
        }
    }
}

impl std::error::Error for WireError {}

impl Frame {
    /// The frame's bytes, written into `out` (cleared first, so one buffer serves every frame).
    ///
    /// # Panics
    /// If the frame has more than [`MAX_TAGS`] tags: the sender trims them first.
    pub fn encode(&self, out: &mut Vec<u8>) {
        assert!(self.tags.len() <= MAX_TAGS, "{} tags", self.tags.len());
        out.clear();
        out.extend_from_slice(&MAGIC.to_le_bytes());
        out.extend_from_slice(&self.version_id.to_le_bytes());
        out.extend_from_slice(&self.sequence.to_le_bytes());
        out.extend_from_slice(&self.width.to_le_bytes());
        out.extend_from_slice(&self.height.to_le_bytes());
        out.extend_from_slice(&self.capture_nanos.to_le_bytes());
        out.push(self.camera);
        out.push(self.flags());
        out.push(self.tags.len() as u8);
        out.push(0);
        out.extend_from_slice(&self.exposure_us.unwrap_or(0).to_le_bytes());
        out.extend_from_slice(&self.gain.unwrap_or(0).to_le_bytes());
        out.push(self.brightness.median);
        out.push(self.brightness.high);
        out.extend_from_slice(&self.brightness.clipped.to_le_bytes());
        out.extend_from_slice(&self.detect_us.to_le_bytes());
        out.extend_from_slice(&self.dropped.to_le_bytes());
        for tag in &self.tags {
            out.extend_from_slice(&tag.id.to_le_bytes());
            out.push(tag.hamming);
            out.push(if tag.poses.is_some() {
                TAG_FLAG_POSES
            } else {
                0
            });
            out.extend_from_slice(&tag.decision_margin.to_le_bytes());
            // Without poses, their bytes are zero.
            let [best, alt] = tag.poses.unwrap_or_default();
            let values = tag
                .corners
                .iter()
                .chain(&best.rvec)
                .chain(&best.tvec)
                .chain(&alt.rvec)
                .chain(&alt.tvec)
                .chain([&best.error_px, &alt.error_px]);
            for value in values {
                out.extend_from_slice(&value.to_le_bytes());
            }
        }
    }

    /// Reads a frame from a datagram's bytes.
    pub fn decode(bytes: &[u8]) -> Result<Frame, WireError> {
        if bytes.len() < HEADER_BYTES {
            return Err(WireError::TooShort(bytes.len()));
        }
        let mut read = Reader { bytes, at: 0 };
        let magic = read.u32();
        if magic != MAGIC {
            return Err(WireError::NotVisor(magic));
        }
        let version_id = read.u32();
        let sequence = read.u32();
        let width = read.u16();
        let height = read.u16();
        let capture_nanos = read.i64();
        let camera = read.u8();
        let flags = read.u8();
        let tag_count = read.u8() as usize;
        read.u8();
        let exposure_us = read.u32();
        let gain = read.i32();
        let brightness = Brightness {
            median: read.u8(),
            high: read.u8(),
            clipped: read.u16(),
        };
        let detect_us = read.u32();
        let dropped = read.u32();
        if tag_count > MAX_TAGS {
            return Err(WireError::TooManyTags(tag_count));
        }
        let expected = HEADER_BYTES + tag_count * TAG_BYTES;
        if bytes.len() != expected {
            return Err(WireError::WrongLength {
                expected,
                actual: bytes.len(),
            });
        }
        let tags = (0..tag_count)
            .map(|_| {
                let id = read.u16();
                let hamming = read.u8();
                let tag_flags = read.u8();
                let decision_margin = read.f32();
                let corners = std::array::from_fn(|_| read.f32());
                let mut pose = || Pose {
                    rvec: std::array::from_fn(|_| read.f32()),
                    tvec: std::array::from_fn(|_| read.f32()),
                    error_px: 0.0,
                };
                let (mut best, mut alt) = (pose(), pose());
                best.error_px = read.f32();
                alt.error_px = read.f32();
                Tag {
                    id,
                    hamming,
                    decision_margin,
                    corners,
                    poses: (tag_flags & TAG_FLAG_POSES != 0).then_some([best, alt]),
                }
            })
            .collect();
        Ok(Frame {
            version_id,
            sequence,
            width,
            height,
            capture_nanos,
            camera,
            clock_synced: flags & FLAG_CLOCK_SYNCED != 0,
            dequeue_time: flags & FLAG_DEQUEUE_TIME != 0,
            exposure_us: (flags & FLAG_EXPOSURE != 0).then_some(exposure_us),
            gain: (flags & FLAG_GAIN != 0).then_some(gain),
            auto_exposure: flags & FLAG_AUTO_EXPOSURE != 0,
            brightness,
            detect_us,
            dropped,
            tags,
        })
    }

    fn flags(&self) -> u8 {
        let mut flags = 0;
        if self.clock_synced {
            flags |= FLAG_CLOCK_SYNCED;
        }
        if self.dequeue_time {
            flags |= FLAG_DEQUEUE_TIME;
        }
        if self.exposure_us.is_some() {
            flags |= FLAG_EXPOSURE;
        }
        if self.gain.is_some() {
            flags |= FLAG_GAIN;
        }
        if self.auto_exposure {
            flags |= FLAG_AUTO_EXPOSURE;
        }
        flags
    }
}

/// Reads little-endian values in order. The caller has checked the length.
struct Reader<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl Reader<'_> {
    fn take<const N: usize>(&mut self) -> [u8; N] {
        let value = self.bytes[self.at..self.at + N]
            .try_into()
            .expect("N bytes");
        self.at += N;
        value
    }
    fn u8(&mut self) -> u8 {
        u8::from_le_bytes(self.take())
    }
    fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take())
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take())
    }
    fn i32(&mut self) -> i32 {
        i32::from_le_bytes(self.take())
    }
    fn i64(&mut self) -> i64 {
        i64::from_le_bytes(self.take())
    }
    fn f32(&mut self) -> f32 {
        f32::from_le_bytes(self.take())
    }
}

const fn parse_version_id(text: &str) -> u32 {
    let bytes = text.as_bytes();
    let mut value = 0u32;
    let mut i = 0;
    while i < bytes.len() {
        value = value * 10 + (bytes[i] - b'0') as u32;
        i += 1;
    }
    value
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The frame testdata/golden-frame.bin holds. The robot client's tests decode the same file.
    fn golden() -> Frame {
        Frame {
            version_id: 20261102,
            sequence: 4_000_000_001,
            width: 1600,
            height: 1304,
            capture_nanos: 1_234_567_890_123,
            camera: 3,
            clock_synced: true,
            dequeue_time: false,
            exposure_us: Some(1000),
            gain: Some(40),
            auto_exposure: false,
            brightness: Brightness {
                median: 37,
                high: 201,
                clipped: 125,
            },
            detect_us: 4250,
            dropped: 17,
            tags: vec![
                Tag {
                    id: 7,
                    hamming: 0,
                    decision_margin: 61.25,
                    corners: [100.5, 200.25, 180.0, 201.0, 179.75, 280.5, 99.0, 279.0],
                    poses: Some([
                        Pose {
                            rvec: [0.125, -2.5, 0.0625],
                            tvec: [-0.5, 0.25, 3.75],
                            error_px: 0.1875,
                        },
                        Pose {
                            rvec: [-0.25, 2.0, -0.5],
                            tvec: [-0.5, 0.25, 3.5],
                            error_px: 1.5,
                        },
                    ]),
                },
                Tag {
                    id: 586,
                    hamming: 2,
                    decision_margin: 12.5,
                    corners: [
                        1500.125, 1200.0, 1580.0, 1201.5, 1579.0, 1290.25, 1499.5, 1289.0,
                    ],
                    poses: None,
                },
            ],
        }
    }

    #[test]
    fn the_layout_matches_the_golden_file() {
        let mut bytes = Vec::new();
        golden().encode(&mut bytes);
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/testdata/golden-frame.bin");
        if std::env::var_os("VISOR_WRITE_GOLDEN").is_some() {
            std::fs::write(path, &bytes).unwrap();
        }
        let file = std::fs::read(path).expect("testdata/golden-frame.bin");
        assert_eq!(
            bytes, file,
            "the layout changed: update docs/wire.md and the robot client"
        );
        assert_eq!(bytes.len(), HEADER_BYTES + 2 * TAG_BYTES);
    }

    #[test]
    fn frames_survive_a_round_trip() {
        let mut bytes = Vec::new();
        let mut cases = vec![golden()];
        let mut empty = golden();
        empty.tags.clear();
        empty.clock_synced = false;
        empty.dequeue_time = true;
        empty.capture_nanos = -5;
        empty.detect_us = u32::MAX;
        empty.dropped = u32::MAX;
        cases.push(empty);
        // The camera's own exposure, and no gain control.
        let mut automatic = golden();
        automatic.exposure_us = None;
        automatic.gain = None;
        automatic.auto_exposure = true;
        automatic.brightness = Brightness {
            median: 255,
            high: 255,
            clipped: 10_000,
        };
        cases.push(automatic);
        let mut full = golden();
        full.tags = vec![golden().tags[0]; MAX_TAGS];
        cases.push(full);
        for frame in cases {
            frame.encode(&mut bytes);
            assert_eq!(Frame::decode(&bytes), Ok(frame));
        }
    }

    #[test]
    fn bad_datagrams_are_refused() {
        let mut good = Vec::new();
        golden().encode(&mut good);
        let mut wrong_magic = good.clone();
        wrong_magic[0] ^= 0xff;
        let mut too_many = good.clone();
        too_many[26] = 15;
        let cases: Vec<(&str, Vec<u8>, WireError)> = vec![
            ("empty", vec![], WireError::TooShort(0)),
            ("header cut", good[..47].to_vec(), WireError::TooShort(47)),
            (
                "wrong magic",
                wrong_magic,
                WireError::NotVisor(MAGIC ^ 0xff),
            ),
            ("too many tags", too_many, WireError::TooManyTags(15)),
            (
                "a tag cut off",
                good[..good.len() - 1].to_vec(),
                WireError::WrongLength {
                    expected: 240,
                    actual: 239,
                },
            ),
        ];
        for (name, bytes, expected) in cases {
            assert_eq!(Frame::decode(&bytes), Err(expected), "{name}");
        }
    }

    #[test]
    fn the_version_id_is_the_package_date() {
        let parts: Vec<u32> = env!("CARGO_PKG_VERSION")
            .split('.')
            .map(|p| p.parse().unwrap())
            .collect();
        assert_eq!(VERSION_ID, parts[0] * 10_000 + parts[1] * 100 + parts[2]);
    }
}
