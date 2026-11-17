//! One frame at full size, as detection read it, for `GET /frame/<camera>`: its pixels as a PGM and
//! what it was taken at and what Visor found in it as one line of JSON (docs/live-view.md).

use axum::body::Bytes;
use serde::Serialize;

use crate::config::Camera;
use crate::stream::feed::Detected;
use crate::stream::focus::Measured;

/// One frame at full size, as detection had it: not made smaller, not compressed, nothing drawn on
/// it.
#[derive(Debug)]
pub struct Still {
    /// The frame as a binary PGM (`P5`, 8-bit), which `tools/calibrate.py` and `visor detect` read.
    pub pgm: Bytes,
    /// What it was taken at and what Visor found in it, as one line of JSON in ASCII
    /// (docs/live-view.md, "A frame at full size").
    pub about: String,
}

impl Still {
    pub fn of(camera: &Camera, frame: &Detected, focus: &Measured) -> Still {
        let image = &frame.image;
        let mut pgm = format!("P5\n{} {}\n255\n", image.width, image.height).into_bytes();
        pgm.extend_from_slice(&image.pixels);
        Still {
            pgm: Bytes::from(pgm),
            about: ascii_json(&About::of(camera, frame, focus)),
        }
    }
}

/// A frame's facts: the camera, the frame's number as sent to the robot, its size, the exposure
/// and gain in effect, its brightness, the width of a tag's edge in it (the view's focus number,
/// [`crate::stream::focus`]: the tag nearest the frame's middle) and that tag's light, and each
/// tag with its corners, its own edge's width and its black and white. Decimals are to a
/// thousandth; a value that isn't known, or isn't a number, is `null`.
#[derive(Debug, Serialize)]
struct About<'a> {
    camera: &'a str,
    sequence: u32,
    width: u32,
    height: u32,
    exposure_us: Option<u32>,
    gain: Option<i32>,
    auto_exposure: Option<bool>,
    brightness_median: u8,
    brightness_high: u8,
    clipped_percent: f64,
    edge_px: Option<f64>,
    light: Option<&'static str>,
    tags: Vec<AboutTag>,
}

#[derive(Debug, Serialize)]
struct AboutTag {
    id: u16,
    hamming: u8,
    decision_margin: Option<f64>,
    corners: Vec<Option<f64>>,
    distance_m: Option<f64>,
    pose_error_px: Option<f64>,
    edge_px: Option<f64>,
    black: Option<u8>,
    white: Option<u8>,
}

impl<'a> About<'a> {
    fn of(camera: &'a Camera, frame: &Detected, focus: &Measured) -> About<'a> {
        let settings = frame.settings;
        let tags = frame
            .tags
            .iter()
            .enumerate()
            .map(|(i, tag)| {
                let best = tag.poses.map(|[best, _]| best);
                AboutTag {
                    id: tag.id,
                    hamming: tag.hamming,
                    decision_margin: thousandths(f64::from(tag.decision_margin)),
                    corners: tag
                        .corners
                        .iter()
                        .map(|c| thousandths(f64::from(*c)))
                        .collect(),
                    distance_m: best.and_then(|best| {
                        let [x, y, z] = best.tvec.map(f64::from);
                        thousandths((x * x + y * y + z * z).sqrt())
                    }),
                    pose_error_px: best.and_then(|best| thousandths(f64::from(best.error_px))),
                    edge_px: focus.edges[i].and_then(thousandths),
                    black: focus.lights[i].map(|light| light.black),
                    white: focus.lights[i].map(|light| light.white),
                }
            })
            .collect();
        About {
            camera: &camera.name,
            sequence: frame.sequence,
            width: frame.image.width,
            height: frame.image.height,
            exposure_us: settings.exposure_us,
            gain: settings.gain,
            auto_exposure: settings.auto_exposure,
            brightness_median: frame.brightness.median,
            brightness_high: frame.brightness.high,
            clipped_percent: f64::from(frame.brightness.clipped) / 100.0,
            edge_px: focus.edge_px().and_then(thousandths),
            light: focus.light().map(|light| light.verdict().word()),
            tags,
        }
    }
}

/// A number to a thousandth, as the facts give it; none for one that isn't finite.
pub fn thousandths(value: f64) -> Option<f64> {
    value.is_finite().then(|| (value * 1000.0).round() / 1000.0)
}

/// JSON in ASCII alone, everything else escaped as `\uXXXX`, so it can go in an HTTP header.
fn ascii_json(value: &impl Serialize) -> String {
    let mut out = Vec::new();
    let mut serializer = serde_json::Serializer::with_formatter(&mut out, Ascii);
    value
        .serialize(&mut serializer)
        .expect("plain structs serialize");
    String::from_utf8(out).expect("ASCII")
}

/// serde_json's compact form, with its strings' non-ASCII characters escaped as UTF-16 units: the
/// one thing its own formatters don't offer.
struct Ascii;

impl serde_json::ser::Formatter for Ascii {
    fn write_string_fragment<W: ?Sized + std::io::Write>(
        &mut self,
        writer: &mut W,
        fragment: &str,
    ) -> std::io::Result<()> {
        for c in fragment.chars() {
            if c.is_ascii() {
                writer.write_all(&[c as u8])?;
            } else {
                let mut units = [0u16; 2];
                for unit in c.encode_utf16(&mut units) {
                    write!(writer, "\\u{unit:04x}")?;
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_still_is_the_frame_s_pixels_as_a_pgm_and_its_facts_as_ascii_json() {
        use crate::brightness::Brightness;
        use crate::image::GrayImage;
        use crate::settings::InEffect;
        use crate::wire::{Pose, Tag};

        let camera = Camera {
            name: "front \"lef\u{e9}\"".into(),
            index: 0,
            device: "/dev/null".into(),
            width: 3,
            height: 2,
            fps: 60,
            format: None,
            min_tag_px: 16,
            time_offset_nanos: 0,
            exposure_us: Some(1000),
            gain: None,
            controls: Vec::new(),
            lens: None,
            tag_size_m: 0.1651,
            stream: crate::config::Stream::default(),
            config_id: 0,
        };
        let mut image = GrayImage::new(3, 2);
        image.pixels.copy_from_slice(&[1, 2, 3, 4, 5, 6]);
        let solved = Pose {
            rvec: [0.0; 3],
            tvec: [0.0, 3.0, 4.0],
            error_px: 0.25,
        };
        let frame = Detected {
            image,
            tags: vec![
                Tag {
                    id: 7,
                    hamming: 1,
                    decision_margin: 81.5,
                    corners: [10.0, 20.5, 30.0, 20.5, 30.0, 40.0, 10.0, 40.0],
                    poses: Some([solved, solved]),
                },
                Tag {
                    id: 9,
                    hamming: 0,
                    decision_margin: f32::NAN,
                    corners: [0.0; 8],
                    poses: None,
                },
            ],
            settings: InEffect {
                exposure_us: Some(1000),
                gain: None,
                auto_exposure: Some(false),
            },
            brightness: Brightness {
                median: 37,
                high: 201,
                clipped: 125,
            },
            sequence: 42,
            monotonic_nanos: 0,
            link: crate::link_state::LinkState::default(),
            clock: crate::stream::feed::Clock::Synced,
        };
        // The second tag is the one the view's focus number goes by; the first couldn't be measured.
        let focus = Measured {
            edges: vec![None, Some(2.3456)],
            lights: vec![
                None,
                Some(crate::stream::light::TagLight {
                    black: 30,
                    white: 140,
                }),
            ],
            nearest: Some(1),
        };
        let still = Still::of(&camera, &frame, &focus);
        assert_eq!(&still.pgm[..], b"P5\n3 2\n255\n\x01\x02\x03\x04\x05\x06");
        assert!(still.about.is_ascii(), "{}", still.about);
        assert!(!still.about.contains('\n'));
        let about: serde_json::Value = serde_json::from_str(&still.about).unwrap();
        assert_eq!(about["camera"], "front \"lef\u{e9}\"");
        assert_eq!(about["sequence"], 42);
        assert_eq!((&about["width"], &about["height"]), (&3.into(), &2.into()));
        assert_eq!(about["exposure_us"], 1000);
        assert!(about["gain"].is_null(), "a camera with no gain control");
        assert_eq!(about["auto_exposure"], false);
        assert_eq!(about["brightness_median"], 37);
        assert_eq!(about["brightness_high"], 201);
        assert_eq!(about["clipped_percent"], 1.25);
        assert_eq!(about["edge_px"], 2.346);
        let tags = about["tags"].as_array().unwrap();
        assert_eq!(tags.len(), 2);
        assert_eq!(tags[0]["id"], 7);
        assert_eq!(tags[0]["hamming"], 1);
        assert_eq!(tags[0]["decision_margin"], 81.5);
        assert_eq!(tags[0]["corners"][1], 20.5);
        assert_eq!(tags[0]["corners"].as_array().unwrap().len(), 8);
        assert_eq!(tags[0]["distance_m"], 5.0);
        assert_eq!(tags[0]["pose_error_px"], 0.25);
        assert!(
            tags[0]["edge_px"].is_null(),
            "a tag that couldn't be measured"
        );
        assert_eq!(tags[1]["edge_px"], 2.346);
        // Each tag's black and white, and the measured one's verdict, so the result can be checked
        // from the numbers alone.
        assert!(tags[0]["black"].is_null() && tags[0]["white"].is_null());
        assert_eq!(
            (&tags[1]["black"], &tags[1]["white"]),
            (&30.into(), &140.into())
        );
        assert_eq!(about["light"], "good");
        // No lens, so no pose; and a number that isn't one is null, never invalid JSON.
        assert!(tags[1]["distance_m"].is_null());
        assert!(tags[1]["decision_margin"].is_null());
    }
}
