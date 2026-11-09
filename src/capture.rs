//! A camera's capture thread: opens the V4L2 device, sets its format, frame rate, exposure, gain
//! and controls, and streams frames into the camera's [`Latest`] slot, each with the exposure and
//! gain in effect. On any error it closes the camera and tries again every 300 ms, so an unplugged
//! camera comes back by itself, its settings set again.

use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use log::{info, warn};
use v4l::buffer::{Flags, Type};
use v4l::io::mmap::Stream;
use v4l::io::traits::CaptureStream;
use v4l::video::Capture;
use v4l::video::capture::Parameters;
use v4l::{Device, Format, FourCC};

use crate::clock::Clocks;
use crate::config::{Camera, PixelFormat};
use crate::latest::{Captured, Latest};
use crate::settings::{self, InEffect, Said};

const RETRY: Duration = Duration::from_millis(300);
/// No frame for this long counts as a lost camera.
const STALL: Duration = Duration::from_secs(1);
const BUFFERS: u32 = 4;

/// Captures until `stop` is set, reconnecting as needed.
pub fn run(camera: &Camera, latest: &Latest, clocks: &impl Clocks, stop: &AtomicBool) {
    let mut last_error = None;
    // Across reopens, so a camera that keeps coming back says each warning once.
    let mut said = Said::default();
    while !stop.load(Ordering::Relaxed) {
        match capture(camera, latest, clocks, stop, &mut last_error, &mut said) {
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
    said: &mut Said,
) -> io::Result<()> {
    let device = Device::with_path(&camera.device)
        .map_err(|e| io::Error::new(e.kind(), format!("{}: {e}", camera.device.display())))?;
    let (format, pixels) = set_format(&device, camera)?;
    let params = device.set_params(&Parameters::with_fps(camera.fps))?;
    let applied = settings::apply(&device, camera)?;
    for warning in &applied.warnings {
        if said.first_time(warning) {
            warn!("{}: {warning}", camera.name);
        }
    }
    let in_effect = applied.in_effect;
    info!("{}: {}", camera.name, described(in_effect));
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
            latest.broken();
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
                settings: in_effect,
            }),
            // A short frame is skipped, not a reason to reconnect, and counted for the detect
            // thread's report rather than logged: a struggling camera can send dozens a second.
            Err(error) => {
                latest.short(error.to_string());
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

/// The exposure and gain in effect, as the log says it once the camera is open.
fn described(in_effect: InEffect) -> String {
    let exposure = match (in_effect.auto_exposure, in_effect.exposure_us) {
        (Some(true), _) => "the camera's automatic exposure".to_string(),
        (_, Some(us)) => format!("exposure {:.1} ms", f64::from(us) / 1000.0),
        (_, None) => "exposure unknown".to_string(),
    };
    match in_effect.gain {
        Some(gain) => format!("{exposure}, gain {gain}"),
        None => format!("{exposure}, no gain control"),
    }
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
