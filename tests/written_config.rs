//! The configuration the robot client writes for a deploy (testdata/written-config.toml, from
//! VisorConfigFileTest), read as `visor run` reads it: every setting lands where it was meant to.
//! And the client's files with every setting at the least and the most it lets through
//! (least-config.toml, most-config.toml): `visor check` passes both, so the client refuses nothing
//! Visor takes, and takes nothing Visor refuses. Each gives the tags' size, as a robot's does.

use std::path::Path;
use std::process::Command;

use visor::config::{Config, Stream};

#[test]
fn the_clients_file_reads_setting_by_setting() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("testdata/written-config.toml");
    let config = Config::load(&path).unwrap();
    assert_eq!(config.robot, "10.12.34.2:5807".parse().unwrap());
    assert_eq!(config.cameras.len(), 2);

    let front = &config.cameras[0];
    assert_eq!(front.name, "front \"left\" \\ 1");
    assert_eq!(front.index, 0);
    assert_eq!(
        front.device,
        Path::new("/dev/v4l/by-id/usb-cam-video-index0")
    );
    assert_eq!((front.width, front.height, front.fps), (1600, 1304, 60));
    assert_eq!(front.min_tag_px, 18);
    // The field's tag size, which the file gives for every camera.
    assert_eq!(front.tag_size_m, 0.15);
    assert_eq!(front.time_offset_nanos, -1_500_000);
    assert_eq!((front.exposure_us, front.gain), (Some(1000), Some(40)));
    assert_eq!(
        front.stream,
        Stream {
            scale: 0.25,
            fps: 15,
            quality: 80
        }
    );
    let controls: Vec<(&str, i64)> = front
        .controls
        .iter()
        .map(|(k, v)| (k.as_str(), *v))
        .collect();
    assert_eq!(controls, [("power_line_frequency", 2), ("brightness", -10)]);
    let lens = front.lens.as_ref().unwrap();
    assert_eq!(
        (lens.fx, lens.fy, lens.cx, lens.cy),
        (1229.75, 1230.125, 801.5, 650.25)
    );
    assert_eq!(
        lens.distortion[..5],
        [-0.3125, 0.125, 1.0e-4, -2.5e-4, -0.03]
    );

    let back = &config.cameras[1];
    assert_eq!(
        (back.name.as_str(), back.index, back.width, back.height),
        ("back", 3, 1280, 800)
    );
    // Left out, so Visor's defaults: the camera's own exposure and gain.
    assert_eq!(
        (back.fps, back.min_tag_px, back.time_offset_nanos),
        (60, 16, 0)
    );
    assert_eq!((back.exposure_us, back.gain), (None, None));
    assert_eq!(back.stream, Stream::default());
    assert!(back.controls.is_empty() && back.lens.is_none());
    assert_eq!(back.tag_size_m, 0.15);
}

#[test]
fn visor_check_passes_the_clients_files_and_says_their_tag_size() {
    // (the file, part of what `visor check` prints)
    let rows = [
        ("written-config.toml", "tags 0.15 m"),
        ("least-config.toml", "frames to 10.0.1.2:1"),
        ("most-config.toml", "tags 9.999999999999998 m"),
    ];
    for (golden, said) in rows {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("testdata")
            .join(golden);
        let output = Command::new(env!("CARGO_BIN_EXE_visor"))
            .arg("check")
            .arg("--config")
            .arg(&path)
            .output()
            .unwrap();
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(output.status.success(), "{golden}: {stderr}");
        let stdout = String::from_utf8_lossy(&output.stdout);
        assert!(stdout.contains(said), "{golden}: {stdout}");
    }
}
