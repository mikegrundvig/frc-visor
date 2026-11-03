//! A camera's capture thread: opens the V4L2 device, sets its format, frame rate and controls,
//! and streams frames into the camera's [`Latest`] slot. On any error it closes the camera and
//! tries again every 300 ms, so an unplugged camera comes back by itself.

use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use log::{info, warn};
use v4l::buffer::{Flags, Type};
use v4l::control::{self, Control, Value};
use v4l::io::mmap::Stream;
use v4l::io::traits::CaptureStream;
use v4l::video::Capture;
use v4l::video::capture::Parameters;
use v4l::{Device, Format, FourCC};

use crate::clock::Clocks;
use crate::config::{Camera, PixelFormat};
use crate::latest::{Captured, Latest};

const RETRY: Duration = Duration::from_millis(300);
/// No frame for this long counts as a lost camera.
const STALL: Duration = Duration::from_secs(1);
const BUFFERS: u32 = 4;

/// Captures until `stop` is set, reconnecting as needed.
pub fn run(camera: &Camera, latest: &Latest, clocks: &impl Clocks, stop: &AtomicBool) {
    let mut last_error = None;
    while !stop.load(Ordering::Relaxed) {
        match capture(camera, latest, clocks, stop, &mut last_error) {
            Ok(()) => return,
            Err(error) => {
                // Said once per cause, not every 300 ms.
                let text = error.to_string();
                if last_error.as_deref() != Some(text.as_str()) {
                    warn!("{}: {text}; trying again every 300 ms", camera.name);
                    last_error = Some(text);
                }
            }
        }
        thread::sleep(RETRY);
    }
}

fn capture(
    camera: &Camera,
    latest: &Latest,
    clocks: &impl Clocks,
    stop: &AtomicBool,
    last_error: &mut Option<String>,
) -> io::Result<()> {
    let device = Device::with_path(&camera.device)
        .map_err(|e| io::Error::new(e.kind(), format!("{}: {e}", camera.device.display())))?;
    let (format, pixels) = set_format(&device, camera)?;
    let params = device.set_params(&Parameters::with_fps(camera.fps))?;
    apply_controls(&device, &camera.controls)?;
    let mut stream = Stream::with_buffers(&device, Type::VideoCapture, BUFFERS)?;
    stream.set_timeout(STALL);
    info!(
        "{}: streaming {}x{} {} at {} s per frame",
        camera.name, format.width, format.height, format.fourcc, params.interval
    );
    *last_error = None;

    let mut said_time_source = false;
    while !stop.load(Ordering::Relaxed) {
        let (data, meta) = stream.next()?;
        if meta.flags.contains(Flags::ERROR) {
            continue;
        }
        let stamped = meta.flags.contains(Flags::TIMESTAMP_MONOTONIC);
        let monotonic_nanos = if stamped {
            meta.timestamp.sec * 1_000_000_000 + meta.timestamp.usec * 1_000
        } else {
            clocks.monotonic_nanos()
        };
        if !said_time_source {
            info!(
                "{}: capture times from {}",
                camera.name,
                time_source(meta.flags)
            );
            said_time_source = true;
        }
        let used = &data[..(meta.bytesused as usize).min(data.len())];
        let mut image = latest.spare(camera.width, camera.height);
        let copied = match pixels {
            PixelFormat::Grey => image.copy_grey(used, format.stride as usize),
            PixelFormat::Yuyv => image.copy_yuyv_luma(used, format.stride as usize),
        };
        match copied {
            Ok(()) => latest.put(Captured {
                image,
                monotonic_nanos,
                dequeue_time: !stamped,
            }),
            // A short frame is skipped, not a reason to reconnect.
            Err(error) => {
                warn!("{}: skipped a frame: {error}", camera.name);
                latest.recycle(image);
            }
        }
    }
    Ok(())
}

/// Sets the configured size and pixel format, and checks the camera took them.
fn set_format(device: &Device, camera: &Camera) -> io::Result<(Format, PixelFormat)> {
    let offered: Vec<[u8; 4]> = device
        .enum_formats()?
        .iter()
        .map(|f| f.fourcc.repr)
        .collect();
    let pixels = choose_format(&offered, camera.format)
        .map_err(|why| io::Error::new(io::ErrorKind::Unsupported, why))?;
    let wanted = Format::new(camera.width, camera.height, FourCC::new(&pixels.fourcc()));
    let actual = device.set_format(&wanted)?;
    if (actual.width, actual.height, actual.fourcc) != (wanted.width, wanted.height, wanted.fourcc)
    {
        return Err(io::Error::new(
            io::ErrorKind::Unsupported,
            format!(
                "asked for {}x{} {}, the camera gives {}x{} {}",
                wanted.width,
                wanted.height,
                wanted.fourcc,
                actual.width,
                actual.height,
                actual.fourcc
            ),
        ));
    }
    Ok((actual, pixels))
}

/// The pixel format to use, from those the camera offers: the configured one, or else `GREY` if
/// offered, then `YUYV`.
pub fn choose_format(
    offered: &[[u8; 4]],
    wanted: Option<PixelFormat>,
) -> Result<PixelFormat, String> {
    let candidates = match wanted {
        Some(format) => vec![format],
        None => vec![PixelFormat::Grey, PixelFormat::Yuyv],
    };
    candidates
        .into_iter()
        .find(|format| offered.contains(&format.fourcc()))
        .ok_or_else(|| {
            let names: Vec<String> = offered
                .iter()
                .map(|f| String::from_utf8_lossy(f).into_owned())
                .collect();
            let asked = match wanted {
                Some(format) => String::from_utf8_lossy(&format.fourcc()).into_owned(),
                None => "GREY or YUYV".into(),
            };
            format!("the camera offers {}, not {asked}", names.join(", "))
        })
}

/// Sets each control, in order, by its `v4l2-ctl` name.
fn apply_controls(device: &Device, controls: &[(String, i64)]) -> io::Result<()> {
    if controls.is_empty() {
        return Ok(());
    }
    let known = device.query_controls()?;
    for (name, value) in controls {
        let found = known
            .iter()
            .filter(|c| c.typ != control::Type::CtrlClass)
            .find(|c| control_key(&c.name) == *name);
        let Some(description) = found else {
            let names: Vec<String> = known
                .iter()
                .filter(|c| c.typ != control::Type::CtrlClass)
                .map(|c| control_key(&c.name))
                .collect();
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("no control {name}; the camera has {}", names.join(", ")),
            ));
        };
        let value = match description.typ {
            control::Type::Boolean => Value::Boolean(*value != 0),
            _ => Value::Integer(*value),
        };
        device
            .set_control(Control {
                id: description.id,
                value,
            })
            .map_err(|e| io::Error::new(e.kind(), format!("setting {name}: {e}")))?;
    }
    Ok(())
}

/// A control's name as `v4l2-ctl` prints it: "Exposure Time, Absolute" is
/// `exposure_time_absolute`.
pub fn control_key(name: &str) -> String {
    let mut key = String::new();
    for c in name.chars() {
        if c.is_ascii_alphanumeric() {
            key.push(c.to_ascii_lowercase());
        } else if !key.is_empty() && !key.ends_with('_') {
            key.push('_');
        }
    }
    key.trim_end_matches('_').to_string()
}

fn time_source(flags: Flags) -> &'static str {
    if !flags.contains(Flags::TIMESTAMP_MONOTONIC) {
        return "when Visor got each frame (the driver gave no timestamp)";
    }
    if flags.contains(Flags::TSTAMP_SRC_SOE) {
        "the driver, at the start of exposure"
    } else {
        "the driver, at the end of the frame"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_names_match_v4l2_ctl() {
        let cases = [
            ("Exposure Time, Absolute", "exposure_time_absolute"),
            ("Auto Exposure", "auto_exposure"),
            (
                "White Balance Temperature, Auto",
                "white_balance_temperature_auto",
            ),
            ("Gain", "gain"),
            ("  Power Line Frequency ", "power_line_frequency"),
        ];
        for (name, key) in cases {
            assert_eq!(control_key(name), key);
        }
    }

    /// What the camera offers, what's configured, and the choice or part of the refusal.
    type FormatCase<'a> = (
        &'a [[u8; 4]],
        Option<PixelFormat>,
        Result<PixelFormat, &'a str>,
    );

    #[test]
    fn the_pixel_format_is_chosen_from_what_the_camera_offers() {
        let grey_and_yuyv = [*b"MJPG", *b"YUYV", *b"GREY"];
        let only_yuyv = [*b"YUYV"];
        let only_mjpg = [*b"MJPG"];
        let cases: [FormatCase; 5] = [
            (&grey_and_yuyv, None, Ok(PixelFormat::Grey)),
            (&only_yuyv, None, Ok(PixelFormat::Yuyv)),
            (
                &grey_and_yuyv,
                Some(PixelFormat::Yuyv),
                Ok(PixelFormat::Yuyv),
            ),
            (
                &only_yuyv,
                Some(PixelFormat::Grey),
                Err("offers YUYV, not GREY"),
            ),
            (&only_mjpg, None, Err("offers MJPG, not GREY or YUYV")),
        ];
        for (offered, wanted, expected) in cases {
            match (choose_format(offered, wanted), expected) {
                (Ok(chosen), Ok(format)) => assert_eq!(chosen, format),
                (Err(why), Err(part)) => assert!(why.contains(part), "{why}"),
                (got, expected) => panic!("{got:?}, expected {expected:?}"),
            }
        }
    }
}
