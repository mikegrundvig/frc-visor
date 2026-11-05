//! The configuration the robot client writes for a deploy (testdata/written-config.toml, from
//! VisorConfigFileTest), read as `visor run` reads it: every setting lands where it was meant to.

use std::path::Path;

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
}
