//! A camera's capture thread: opens the camera, sets its format, frame rate, exposure, gain and
//! controls, and streams frames into the camera's [`Latest`] slot, each with the exposure and gain
//! in effect and the camera's link state. On any error it closes the camera and tries again every
//! 300 ms, so an unplugged camera comes back by itself, its settings set again.
//!
//! Before each open, a camera below the USB link it can do, or one gone too long, may get one
//! recovery ([`crate::recovery`]): never while it streams.
//!
//! The format is the configured one, or the first of GREY, YUYV and MJPG that gives the size at
//! the frame rate ([`crate::formats`]). An MJPG frame is decoded to its luma here, on the capture
//! thread, so detection's thread never spends its frame time on it.
//!
//! Not every frame the camera sends is given to Visor. Linux's USB camera driver throws away an
//! uncompressed frame that came incomplete or that the camera flagged bad, and tells no one; only
//! the number it gives each frame shows it, by skipping. Those are counted as dropped
//! ([`Latest::withheld`]), so a camera sending 60 frames a second of which 53 are whole shows as
//! 53 and 7 dropped, not as 53 and none. A compressed frame that doesn't decode is counted the same
//! way ([`Latest::undecoded`]).
//!
//! The camera is a [`Source`]: a V4L2 device ([`V4l2`]), memory-mapped, or a test's.

mod v4l2;

use std::io;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use log::{info, warn};
use v4l::buffer::{Flags, Metadata};
use v4l::fraction::Fraction;
use v4l::{Format, FourCC};

use crate::clock::Clocks;
use crate::config::{Camera, PixelFormat};
use crate::formats::{self, Choice, Rates, Why};
use crate::latest::{Captured, Latest};
use crate::link_state::LinkState;
use crate::recovery::{self, Outcome, Recovery};
use crate::settings::{self, Controls, InEffect, Said, Trial, Wanted};
use crate::usb::Usb;

pub use v4l2::{V4l2, V4l2Camera};

/// How long a camera that failed waits before it's opened again.
pub const RETRY: Duration = Duration::from_millis(300);
/// No frame for this long counts as a lost camera, at a normal frame rate.
const STALL: Duration = Duration::from_secs(1);
/// At a slow one, no frame for this many frame times. A camera offering its full size raw on USB 2
/// at 5 frames a second sent 2.5, its first 1.8 s (9 frame times) after it started.
const STALL_FRAMES: u32 = 15;

/// Where a camera's frames come from: a V4L2 device, or a test's camera. Its USB side
/// ([`Usb`]) is what recovery reads and acts on.
pub trait Source: Usb {
    /// The camera once it's open.
    type Opened<'a>: Opened
    where
        Self: 'a;

    /// Opens the camera at `device`, or says why it can't: not plugged in, or busy.
    fn open(&mut self, device: &Path) -> io::Result<Self::Opened<'_>>;
}

/// An open camera: its formats and controls, then its frames. It's closed when dropped.
pub trait Opened: Controls {
    /// The pixel formats it offers, each by its four characters.
    fn formats(&self) -> io::Result<Vec<[u8; 4]>>;
    /// The frame rates it offers for a format at a size.
    fn rates(&self, fourcc: [u8; 4], width: u32, height: u32) -> Rates;
    /// Asks for a size and pixel format, and gives what the camera took instead, if it did.
    fn set_format(&self, wanted: &Format) -> io::Result<Format>;
    /// Asks for a frame rate, and gives the time per frame the camera took.
    fn set_fps(&self, fps: u32) -> io::Result<Fraction>;
    /// Starts it streaming; no frame for `stall` is an error.
    fn start(&mut self, stall: Duration) -> io::Result<()>;
    /// The next frame's bytes and the driver's description of them. An error of kind
    /// [`io::ErrorKind::TimedOut`] once none has come for the stall time, or another when the
    /// camera has gone.
    fn next(&mut self) -> io::Result<(&[u8], &Metadata)>;
}

/// What the capture loop keeps across a camera's reopens.
#[derive(Default)]
struct Reopens {
    /// The last open's error, said once per cause, not every 300 ms.
    last_error: Option<String>,
    /// Warnings said, so a camera that keeps coming back says each once.
    said: Said,
    recovery: Recovery,
    /// What a recovery came to, said once the camera streams again.
    outcome: Option<Outcome>,
}

/// Captures from a V4L2 device until `stop` is set, reconnecting as needed.
pub fn run(camera: &Camera, latest: &Latest, clocks: &impl Clocks, stop: &AtomicBool) {
    // Nothing asks this camera to try settings: `visor record`'s.
    let untried = Trial::default();
    run_from(
        &mut V4l2::default(),
        camera,
        latest,
        clocks,
        &untried,
        stop,
        RETRY,
    );
}

/// Captures from `source` until `stop` is set, opening the camera again `retry` after it fails.
/// An exposure or gain asked for through `trial` is set before the next frame, and lasts until the
/// camera is opened again.
pub fn run_from(
    source: &mut impl Source,
    camera: &Camera,
    latest: &Latest,
    clocks: &impl Clocks,
    trial: &Trial,
    stop: &AtomicBool,
    retry: Duration,
) {
    let mut reopens = Reopens::default();
    while !stop.load(Ordering::Relaxed) {
        match capture(source, camera, latest, clocks, trial, stop, &mut reopens) {
            Ok(()) => return,
            Err(error) => {
                let text = error.to_string();
                if reopens.last_error.as_deref() != Some(text.as_str()) {
                    warn!(
                        "{}: {text}; trying again every {} ms",
                        camera.name,
                        retry.as_millis()
                    );
                    reopens.last_error = Some(text);
                }
            }
        }
        thread::sleep(retry);
    }
}

/// The camera as it streams: what each frame is converted with and stamped with.
struct Streaming {
    format: Format,
    pixels: PixelFormat,
    link: LinkState,
}

fn capture(
    source: &mut impl Source,
    camera: &Camera,
    latest: &Latest,
    clocks: &impl Clocks,
    trial: &Trial,
    stop: &AtomicBool,
    reopens: &mut Reopens,
) -> io::Result<()> {
    let opening = reopens
        .recovery
        .before_open(source, &camera.name, &camera.device);
    if let Some(outcome) = opening.outcome {
        reopens.outcome = Some(outcome);
    }
    let mut opened = source
        .open(&camera.device)
        .map_err(|e| io::Error::new(e.kind(), format!("{}: {e}", camera.device.display())))?;
    let (format, choice) = set_format(&opened, camera, &mut reopens.said)?;
    let interval = opened.set_fps(camera.fps)?;
    let applied = settings::apply(&opened, camera)?;
    for warning in &applied.warnings {
        if reopens.said.first_time(warning) {
            warn!("{}: {warning}", camera.name);
        }
    }
    let mut in_effect = applied.in_effect;
    trial.offer(settings::ranges(&opened));
    trial.offer_configured(applied.in_effect);
    trial.offer_serial(
        opening
            .link
            .as_ref()
            .and_then(|link| link.identity.serial.clone()),
    );
    info!("{}: {}", camera.name, described(in_effect));
    // The camera as tried: the configuration's, with each exposure and gain asked for since.
    let mut tried: Option<Camera> = None;
    let stall = stall_after(interval);
    opened.start(stall)?;
    let recovered = opening
        .link
        .as_ref()
        .map(|link| reopens.recovery.recovered(link))
        .unwrap_or_default();
    let streaming = Streaming {
        format,
        pixels: choice.format,
        link: LinkState::of(opening.link.as_ref(), choice.format, recovered),
    };
    let running = format!(
        "{}x{} {} at {} s per frame",
        format.width, format.height, format.fourcc, interval
    );
    let on = opening
        .link
        .as_ref()
        .map_or("a link that isn't known".to_string(), ToString::to_string);
    info!("{}: streaming {running} on {on}", camera.name);
    if let Some(outcome) = reopens.outcome.take() {
        recovery::say(&camera.name, &outcome, &running);
    }
    reopens.last_error = None;

    let mut said_time_source = false;
    // The driver's number of the last frame it gave, since the camera was opened.
    let mut numbered: Option<u32> = None;
    while !stop.load(Ordering::Relaxed) {
        if let Some(wanted) = trial.take() {
            let with = match wanted {
                Wanted::Values(asked) => {
                    let with = tried.get_or_insert_with(|| camera.clone());
                    with.exposure_us = asked.exposure_us.or(with.exposure_us);
                    with.gain = asked.gain.or(with.gain);
                    with
                }
                // The configuration's again: what was tried is forgotten.
                Wanted::Configuration => tried.insert(camera.clone()),
            };
            // A trial that can't be set is said, and the camera keeps what it had.
            match settings::apply(&opened, with) {
                Ok(applied) => {
                    for warning in &applied.warnings {
                        warn!("{}: trying settings: {warning}", camera.name);
                    }
                    in_effect = applied.in_effect;
                    info!(
                        "{}: trying {} until it's opened again",
                        camera.name,
                        described(in_effect)
                    );
                }
                Err(error) => warn!("{}: trying settings: {error}", camera.name),
            }
        }
        let (data, meta) = opened.next().map_err(|error| stalled(error, stall))?;
        let missing = withheld(numbered, meta.sequence);
        if missing > 0 {
            latest.withheld(missing);
        }
        numbered = Some(meta.sequence);
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
        let frame = Captured {
            image: latest.spare(camera.width, camera.height),
            monotonic_nanos,
            dequeue_time: !stamped,
            settings: in_effect,
            link: streaming.link,
        };
        put(latest, &streaming, used, frame);
    }
    Ok(())
}

/// Converts a frame's bytes into its image and puts it in the slot; or counts it as dropped: a
/// short frame and one that doesn't decode are skipped, not a reason to reconnect, and counted
/// for the detect thread's report rather than logged, since a struggling camera can send dozens a
/// second.
fn put(latest: &Latest, streaming: &Streaming, used: &[u8], mut frame: Captured) {
    let stride = streaming.format.stride as usize;
    let image = &mut frame.image;
    let converted = match streaming.pixels {
        PixelFormat::Grey => image.copy_grey(used, stride).map_err(Unusable::Short),
        PixelFormat::Yuyv => image.copy_yuyv_luma(used, stride).map_err(Unusable::Short),
        PixelFormat::Mjpg => image
            .decode_jpeg_luma(used)
            .map_err(|_| Unusable::Undecodable),
    };
    match converted {
        Ok(()) => latest.put(frame),
        Err(Unusable::Short(error)) => {
            latest.short(error.to_string());
            latest.recycle(frame.image);
        }
        Err(Unusable::Undecodable) => {
            latest.undecoded();
            latest.recycle(frame.image);
        }
    }
}

/// Why a frame's bytes made no image.
enum Unusable {
    Short(io::Error),
    Undecodable,
}

/// How long a camera granted `interval` per frame may send nothing before it's lost: a second,
/// or [`STALL_FRAMES`] frame times for a slow camera, so one that's merely slow is never
/// declared stalled before it could have sent a frame.
fn stall_after(interval: Fraction) -> Duration {
    if interval.denominator == 0 {
        return STALL;
    }
    let frames =
        Duration::from_secs(u64::from(interval.numerator)) * STALL_FRAMES / interval.denominator;
    frames.max(STALL)
}

/// A stream's wait running out, said with how long it waited.
fn stalled(error: io::Error, stall: Duration) -> io::Error {
    if error.kind() != io::ErrorKind::TimedOut {
        return error;
    }
    io::Error::new(
        io::ErrorKind::TimedOut,
        format!("no frame for {} s", seconds(stall)),
    )
}

/// A duration in seconds as the log says it: whole, or to a tenth.
fn seconds(duration: Duration) -> String {
    if duration.subsec_millis() == 0 {
        duration.as_secs().to_string()
    } else {
        format!("{:.1}", duration.as_secs_f64())
    }
}

/// Sets the chosen size and pixel format, and checks the camera took them.
fn set_format(
    device: &impl Opened,
    camera: &Camera,
    said: &mut Said,
) -> io::Result<(Format, Choice)> {
    let offered = device.formats()?;
    // Visor's formats among them, with their rates at the size: not asked for when the
    // configuration names the format.
    let rated: Vec<(PixelFormat, Rates)> = offered
        .iter()
        .filter_map(|fourcc| PixelFormat::from_fourcc(*fourcc))
        .map(|format| match camera.format {
            Some(_) => (format, Rates::Unknown),
            None => (
                format,
                device.rates(format.fourcc(), camera.width, camera.height),
            ),
        })
        .collect();
    let choice = formats::choose(&offered, &rated, camera.format, camera.fps)
        .map_err(|why| io::Error::new(io::ErrorKind::Unsupported, why))?;
    if let Why::Fastest(rate) = choice.why {
        let warning = format!(
            "no format gives {}x{} at {} fps: {}, the fastest at that size, at {rate:.1} fps",
            camera.width,
            camera.height,
            camera.fps,
            choice.format.name()
        );
        if said.first_time(&warning) {
            warn!("{}: {warning}", camera.name);
        }
    }
    let wanted = Format::new(
        camera.width,
        camera.height,
        FourCC::new(&choice.format.fourcc()),
    );
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
    Ok((actual, choice))
}

/// How many frames the driver numbered after `last`, the one it gave before, and never gave: the
/// numbers skipped on the way to `now`. None before a stream's first frame, and none from a driver
/// that doesn't number its frames, or whose numbers go back.
fn withheld(last: Option<u32>, now: u32) -> u32 {
    match last {
        Some(last) if now > last => now - last - 1,
        _ => 0,
    }
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
mod tests;
