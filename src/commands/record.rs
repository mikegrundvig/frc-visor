//! `visor record`: frames from one camera saved as PGM files, for lens calibration.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use visor::capture;
use visor::clock::SystemClocks;
use visor::config::{Camera, Config, PixelFormat};
use visor::latest::Latest;

use super::{Result, find_camera};

/// Records from the camera the configuration names `name`.
pub fn record_configured(
    path: &Path,
    name: &str,
    out: &Path,
    count: u32,
    every: Duration,
) -> Result<()> {
    let config = Config::load(path)?;
    let camera = find_camera(&config, path, name)?;
    record(camera, out, count, every)
}

/// A camera by its device alone, before any robot has configured the board: its own exposure and
/// gain, no controls, no lens. Its frames are named after the device.
pub fn unconfigured(
    device: PathBuf,
    width: u32,
    height: u32,
    fps: u32,
    format: Option<&str>,
) -> Camera {
    let name = device
        .file_name()
        .map_or("camera".into(), |n| n.to_string_lossy().into_owned());
    Camera {
        name,
        index: 0,
        device,
        width,
        height,
        fps,
        format: match format {
            Some("GREY") => Some(PixelFormat::Grey),
            Some("YUYV") => Some(PixelFormat::Yuyv),
            _ => None,
        },
        min_tag_px: visor::config::DEFAULT_MIN_TAG_PX,
        time_offset_nanos: 0,
        exposure_us: None,
        gain: None,
        controls: Vec::new(),
        lens: None,
        tag_size_m: visor::config::DEFAULT_TAG_SIZE_M,
        stream: visor::config::Stream::default(),
        // Recording sends no frames, so there's no configuration to name.
        config_id: 0,
    }
}

/// Saves `count` frames from `camera` in `out`, `every` apart.
pub fn record(camera: &Camera, out: &Path, count: u32, every: Duration) -> Result<()> {
    std::fs::create_dir_all(out)?;
    let latest = Latest::new();
    let stop = AtomicBool::new(false);
    thread::scope(|scope| -> Result<()> {
        scope.spawn(|| capture::run(camera, &latest, &SystemClocks, &stop));
        let saved = save_frames(camera, &latest, out, count, every);
        stop.store(true, Ordering::Relaxed);
        saved
    })
}

/// Takes a frame from the camera's slot each `every`, and writes it as a PGM file.
fn save_frames(
    camera: &Camera,
    latest: &Latest,
    out: &Path,
    count: u32,
    every: Duration,
) -> Result<()> {
    for n in 0..count {
        thread::sleep(every);
        let frame = latest.take(Duration::from_secs(5)).ok_or(
            "no frame from the camera for 5 s: if Visor's service is running, it has the \
                 camera (systemctl stop visor first)",
        )?;
        let file = out.join(format!("{}-{n:03}.pgm", camera.name));
        frame.image.write_pgm(&file)?;
        println!("{}", file.display());
        latest.recycle(frame.image);
    }
    Ok(())
}
