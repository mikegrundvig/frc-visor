//! `visor check`: a configuration checked as `visor run` reads it, and what it sets up.

use std::path::Path;

use visor::config::Config;
use visor::{stream, wire};

use super::Result;

/// Reads the configuration at `path` as `visor run` would, and prints what it sets up.
pub fn check(path: &Path) -> Result<()> {
    let config = Config::load(path)?;
    println!(
        "{}: visor {} (version ID {}), frames to {}",
        path.display(),
        env!("CARGO_PKG_VERSION"),
        wire::VERSION_ID,
        config.robot
    );
    for camera in &config.cameras {
        let exposure = match camera.exposure_us {
            Some(us) => format!("exposure {:.1} ms", f64::from(us) / 1000.0),
            None => "the camera's own exposure".to_string(),
        };
        let gain = match camera.gain {
            Some(gain) => format!("gain {gain}"),
            None => "its own gain".to_string(),
        };
        let (width, height) = camera.stream.size(camera.width, camera.height);
        println!(
            "  camera {} {:?}: {}x{} at {} fps, {}, {exposure}, {gain}, {}; live view {width}x{height} \
             at {} fps, quality {}",
            camera.index,
            camera.name,
            camera.width,
            camera.height,
            camera.fps,
            camera.device.display(),
            if camera.lens.is_some() {
                "lens calibrated"
            } else {
                "no lens: corners only"
            },
            camera.stream.fps,
            camera.stream.quality
        );
    }
    println!(
        "  live view on port {}: /stream/<camera>, and / lists them",
        stream::PORT
    );
    Ok(())
}
