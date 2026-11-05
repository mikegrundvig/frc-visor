//! The configuration file, deployed with the program. An example is deploy/visor.example.toml.

use std::collections::HashSet;
use std::fmt;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::lens::Lens;

/// The port frames go to on the robot.
pub const DEFAULT_PORT: u16 = 5807;
pub const DEFAULT_FPS: u32 = 60;
pub const DEFAULT_MIN_TAG_PX: i32 = 16;
/// An FRC AprilTag's black square since 2024: 6.5 inches.
pub const DEFAULT_TAG_SIZE_M: f64 = 0.1651;
/// The live view's size, as a share of the camera's across and down: 800 x 652 of a 1600 x 1304
/// camera, enough to aim it and read its tags' IDs.
pub const DEFAULT_STREAM_SCALE: f64 = 0.5;
/// The live view's frames a second: enough to aim a camera by, at a sixth of a 60 fps camera's.
pub const DEFAULT_STREAM_FPS: u32 = 10;
/// The live view's JPEG quality, 1 to 100: its tags' edges and IDs stay sharp, at 20 to 25 KB a
/// picture of 800 x 652, about 2 Mbit/s at 10 a second.
pub const DEFAULT_STREAM_QUALITY: u8 = 60;

/// The checked configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    pub robot: SocketAddr,
    pub cameras: Vec<Camera>,
}

/// One camera.
#[derive(Debug, Clone, PartialEq)]
pub struct Camera {
    /// For logs.
    pub name: String,
    /// Sent in every frame; unique on the robot.
    pub index: u8,
    /// As `visor cameras` names it: its `/dev/v4l/by-id/` path when it has a USB serial, so it's
    /// found again in any port; without one, its `/dev/v4l/by-path/` path, its USB port's.
    pub device: PathBuf,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    /// The pixel format to ask for; without one, `GREY` if offered, else `YUYV`.
    pub format: Option<PixelFormat>,
    pub min_tag_px: i32,
    /// Added to every capture time: the spin test's result.
    pub time_offset_nanos: i64,
    /// A fixed exposure, in microseconds: the camera's automatic exposure off, and this set. None
    /// leaves the exposure to the camera's own automatic exposure, turned back on.
    pub exposure_us: Option<u32>,
    /// The camera's `gain` control, in its own units. None puts back the camera's default.
    pub gain: Option<i64>,
    /// V4L2 controls by the names `v4l2-ctl --list-ctrls` prints, set in this order, after the
    /// exposure and gain.
    pub controls: Vec<(String, i64)>,
    /// The lens calibration at this resolution. Without one, tags are sent with their corners
    /// only, no poses.
    pub lens: Option<Lens>,
    /// The side of the tags' black square, in metres.
    pub tag_size_m: f64,
    /// Its live view, served over HTTP while someone watches.
    pub stream: Stream,
}

/// A camera's live view: its frames, made smaller, as JPEGs ([`crate::stream`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stream {
    /// Its size as a share of the camera's, across and down: 0.5 is half.
    pub scale: f64,
    /// Frames a second, at most the camera's.
    pub fps: u32,
    /// JPEG quality, 1 to 100: higher is sharper, and more bytes a frame.
    pub quality: u8,
}

impl Default for Stream {
    fn default() -> Stream {
        Stream {
            scale: DEFAULT_STREAM_SCALE,
            fps: DEFAULT_STREAM_FPS,
            quality: DEFAULT_STREAM_QUALITY,
        }
    }
}

impl Stream {
    /// The view's size for a camera's frames of this size: at least a pixel each way.
    pub fn size(&self, width: u32, height: u32) -> (u32, u32) {
        let scaled = |n: u32| ((f64::from(n) * self.scale).round() as u32).max(1);
        (scaled(width), scaled(height))
    }
}

/// The pixel formats Visor reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub enum PixelFormat {
    /// 8-bit mono.
    #[serde(rename = "GREY")]
    Grey,
    /// 4:2:2 colour; Visor takes the luma.
    #[serde(rename = "YUYV")]
    Yuyv,
}

impl PixelFormat {
    pub fn fourcc(self) -> [u8; 4] {
        match self {
            PixelFormat::Grey => *b"GREY",
            PixelFormat::Yuyv => *b"YUYV",
        }
    }
}

/// Why a configuration is refused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigError(pub String);

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ConfigError {}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct File {
    robot: Option<IpAddr>,
    team: Option<u32>,
    port: Option<u16>,
    tag_size_m: Option<f64>,
    #[serde(default, rename = "camera")]
    cameras: Vec<CameraFile>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CameraFile {
    name: String,
    index: u8,
    device: PathBuf,
    width: u32,
    height: u32,
    fps: Option<u32>,
    format: Option<PixelFormat>,
    min_tag_px: Option<i32>,
    time_offset_us: Option<i64>,
    exposure_us: Option<u32>,
    gain: Option<i64>,
    stream_scale: Option<f64>,
    stream_fps: Option<u32>,
    stream_quality: Option<u8>,
    #[serde(default)]
    controls: toml::Table,
    lens: Option<LensFile>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct LensFile {
    /// The frame size it was calibrated at, which must be the camera's.
    width: u32,
    height: u32,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    #[serde(default)]
    distortion: Vec<f64>,
}

impl Config {
    pub fn load(path: &Path) -> Result<Config, ConfigError> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| ConfigError(format!("{}: {e}", path.display())))?;
        Config::parse(&text).map_err(|e| ConfigError(format!("{}: {e}", path.display())))
    }

    pub fn parse(text: &str) -> Result<Config, ConfigError> {
        let file: File = toml::from_str(text).map_err(|e| ConfigError(e.message().to_string()))?;
        let address = match (file.robot, file.team) {
            (Some(robot), None) => robot,
            (None, Some(team)) => team_address(team)?,
            _ => {
                return Err(ConfigError(
                    "give either `robot` (an address) or `team`".into(),
                ));
            }
        };
        let tag_size_m = file.tag_size_m.unwrap_or(DEFAULT_TAG_SIZE_M);
        if !(tag_size_m > 0.0 && tag_size_m < 10.0) {
            return Err(ConfigError(format!(
                "tag_size_m is the tags' black square in metres, not {tag_size_m}"
            )));
        }
        if file.cameras.is_empty() {
            return Err(ConfigError("no [[camera]]".into()));
        }
        let mut names = HashSet::new();
        let mut indices = HashSet::new();
        let mut cameras = Vec::new();
        for camera in file.cameras {
            let camera = check_camera(camera, tag_size_m)?;
            if !names.insert(camera.name.clone()) {
                return Err(ConfigError(format!(
                    "two cameras are named {:?}",
                    camera.name
                )));
            }
            if !indices.insert(camera.index) {
                return Err(ConfigError(format!(
                    "two cameras have index {}",
                    camera.index
                )));
            }
            cameras.push(camera);
        }
        Ok(Config {
            robot: SocketAddr::new(address, file.port.unwrap_or(DEFAULT_PORT)),
            cameras,
        })
    }
}

/// The robot controller's address for a team number: 10.TE.AM.2.
fn team_address(team: u32) -> Result<IpAddr, ConfigError> {
    if !(1..=25_599).contains(&team) {
        return Err(ConfigError(format!("team {team} isn't a team number")));
    }
    Ok(IpAddr::V4(Ipv4Addr::new(
        10,
        (team / 100) as u8,
        (team % 100) as u8,
        2,
    )))
}

fn check_camera(file: CameraFile, tag_size_m: f64) -> Result<Camera, ConfigError> {
    let name = file.name;
    let fail = |why: String| ConfigError(format!("camera {name:?}: {why}"));
    if name.trim().is_empty() {
        return Err(ConfigError("a camera needs a name".into()));
    }
    if !(1..=u16::MAX as u32).contains(&file.width) || !(1..=u16::MAX as u32).contains(&file.height)
    {
        return Err(fail(format!(
            "{}x{} isn't a frame size",
            file.width, file.height
        )));
    }
    let fps = file.fps.unwrap_or(DEFAULT_FPS);
    if !(1..=1000).contains(&fps) {
        return Err(fail(format!("fps {fps} isn't a frame rate")));
    }
    let min_tag_px = file.min_tag_px.unwrap_or(DEFAULT_MIN_TAG_PX);
    if !(8..=30_000).contains(&min_tag_px) {
        return Err(fail(format!(
            "min_tag_px is from 8 to 30000, not {min_tag_px}"
        )));
    }
    if let Some(us) = file.exposure_us
        && !(1..=10_000_000).contains(&us)
    {
        return Err(fail(format!(
            "exposure_us is the exposure in microseconds, 1 to 10000000, not {us}"
        )));
    }
    let stream =
        check_stream(file.stream_scale, file.stream_fps, file.stream_quality, fps).map_err(fail)?;
    let mut controls = Vec::new();
    for (key, value) in file.controls {
        let value = match value {
            toml::Value::Integer(n) => n,
            toml::Value::Boolean(b) => b as i64,
            other => {
                return Err(fail(format!(
                    "control {key} is a number or true/false, not {other}"
                )));
            }
        };
        controls.push((key, value));
    }
    let lens = match file.lens {
        Some(l) if (l.width, l.height) != (file.width, file.height) => {
            return Err(fail(format!(
                "lens: calibrated at {}x{}, but the camera runs at {}x{}: calibrate at the size it runs",
                l.width, l.height, file.width, file.height
            )));
        }
        Some(l) => Some(
            Lens::new(l.fx, l.fy, l.cx, l.cy, &l.distortion)
                .map_err(|e| fail(format!("lens: {e}")))?,
        ),
        None => None,
    };
    Ok(Camera {
        index: file.index,
        device: file.device,
        width: file.width,
        height: file.height,
        fps,
        format: file.format,
        min_tag_px,
        time_offset_nanos: file.time_offset_us.unwrap_or(0) * 1_000,
        exposure_us: file.exposure_us,
        gain: file.gain,
        controls,
        lens,
        tag_size_m,
        stream,
        name,
    })
}

/// The live view's settings, each Visor's default where it's left out.
fn check_stream(
    scale: Option<f64>,
    stream_fps: Option<u32>,
    quality: Option<u8>,
    fps: u32,
) -> Result<Stream, String> {
    let defaults = Stream::default();
    let scale = scale.unwrap_or(defaults.scale);
    if !(0.05..=1.0).contains(&scale) {
        return Err(format!(
            "stream_scale is the live view's share of the camera's size, 0.05 to 1, not {scale}"
        ));
    }
    let stream_fps = stream_fps.unwrap_or(defaults.fps.min(fps));
    if !(1..=fps).contains(&stream_fps) {
        return Err(format!(
            "stream_fps is from 1 to the camera's fps ({fps}), not {stream_fps}"
        ));
    }
    let quality = quality.unwrap_or(defaults.quality);
    if !(1..=100).contains(&quality) {
        return Err(format!(
            "stream_quality is a JPEG quality, 1 to 100, not {quality}"
        ));
    }
    Ok(Stream {
        scale,
        fps: stream_fps,
        quality,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const CAMERA: &str = r#"
        [[camera]]
        name = "front-left"
        index = 0
        device = "/dev/v4l/by-id/usb-cam-video-index0"
        width = 1600
        height = 1304
    "#;

    #[test]
    fn a_minimal_file_gets_the_defaults() {
        let config = Config::parse(&format!("team = 1234\n{CAMERA}")).unwrap();
        assert_eq!(config.robot, "10.12.34.2:5807".parse().unwrap());
        let camera = &config.cameras[0];
        assert_eq!(
            (camera.fps, camera.min_tag_px, camera.format),
            (60, 16, None)
        );
        assert_eq!(camera.time_offset_nanos, 0);
        // No opinion: the camera's own exposure and gain.
        assert_eq!((camera.exposure_us, camera.gain), (None, None));
        assert!(camera.controls.is_empty());
        assert_eq!((camera.lens.as_ref(), camera.tag_size_m), (None, 0.1651));
        // Half the camera's size, at 10 fps.
        assert_eq!(camera.stream, Stream::default());
        assert_eq!(camera.stream.size(1600, 1304), (800, 652));
        assert_eq!((camera.stream.fps, camera.stream.quality), (10, 60));
    }

    #[test]
    fn a_slow_camera_s_live_view_is_at_most_its_rate() {
        let config = Config::parse(&format!("team = 1234\n{CAMERA}fps = 5\n")).unwrap();
        assert_eq!(config.cameras[0].stream.fps, 5);
    }

    #[test]
    fn every_setting_reads() {
        let text = r#"
            robot = "127.0.0.1"
            port = 6000
            tag_size_m = 0.15
            [[camera]]
            name = "back"
            index = 3
            device = "/dev/video2"
            width = 1280
            height = 720
            fps = 100
            format = "YUYV"
            min_tag_px = 24
            time_offset_us = -1500
            exposure_us = 1000
            gain = 40
            stream_scale = 0.25
            stream_fps = 15
            stream_quality = 80
            [camera.controls]
            power_line_frequency = 2
            brightness = -10
            exposure_dynamic_framerate = false
            [camera.lens]
            width = 1280
            height = 720
            fx = 1200.0
            fy = 1201.0
            cx = 799.5
            cy = 651.5
            distortion = [0.1, -0.2, 0.001, 0.002, 0.05]
        "#;
        let config = Config::parse(text).unwrap();
        assert_eq!(config.robot, "127.0.0.1:6000".parse().unwrap());
        let camera = &config.cameras[0];
        assert_eq!(camera.format, Some(PixelFormat::Yuyv));
        assert_eq!(camera.time_offset_nanos, -1_500_000);
        assert_eq!(camera.tag_size_m, 0.15);
        assert_eq!((camera.exposure_us, camera.gain), (Some(1000), Some(40)));
        assert_eq!(
            camera.stream,
            Stream {
                scale: 0.25,
                fps: 15,
                quality: 80
            }
        );
        assert_eq!(camera.stream.size(1280, 720), (320, 180));
        let lens = camera.lens.as_ref().unwrap();
        assert_eq!(
            (lens.fx, lens.cy, lens.distortion[4], lens.distortion[5]),
            (1200.0, 651.5, 0.05, 0.0)
        );
        // In the file's order, which matters where one control depends on another.
        let controls: Vec<(&str, i64)> = camera
            .controls
            .iter()
            .map(|(k, v)| (k.as_str(), *v))
            .collect();
        assert_eq!(
            controls,
            [
                ("power_line_frequency", 2),
                ("brightness", -10),
                ("exposure_dynamic_framerate", 0)
            ]
        );
    }

    #[test]
    fn mistakes_are_named() {
        let second = CAMERA.replace("front-left", "front-right");
        let cases = [
            ("".to_string(), "either `robot`"),
            (
                format!("team = 1234\nrobot = \"10.12.34.2\"\n{CAMERA}"),
                "either `robot`",
            ),
            ("team = 0\n".to_string() + CAMERA, "isn't a team number"),
            ("team = 1234\n".to_string(), "no [[camera]]"),
            (
                format!("team = 1234\n{CAMERA}{CAMERA}"),
                "named \"front-left\"",
            ),
            (format!("team = 1234\n{CAMERA}{second}"), "index 0"),
            (
                format!("team = 1234\n{}", CAMERA.replace("1304", "70000")),
                "isn't a frame size",
            ),
            (
                format!("team = 1234\n{CAMERA}min_tag_px = 4\n"),
                "min_tag_px",
            ),
            (
                format!("team = 1234\n{CAMERA}format = \"MJPG\"\n"),
                "unknown variant",
            ),
            (
                format!("team = 1234\n{CAMERA}colour = true\n"),
                "unknown field",
            ),
            (
                format!("team = 1234\n{CAMERA}[camera.controls]\ngain = \"high\"\n"),
                "control gain",
            ),
            (
                format!("team = 1234\n{CAMERA}exposure_us = 0\n"),
                "exposure_us is the exposure in microseconds",
            ),
            (
                format!("team = 1234\n{CAMERA}exposure_us = 1.5\n"),
                "invalid type",
            ),
            (
                format!(
                    "team = 1234\n{CAMERA}[camera.lens]\nwidth = 1600\nheight = 1304\nfx = 1.0\nfy = 1.0\ncx = 0.0\ncy = 0.0\ndistortion = [0.1]\n"
                ),
                "lens: OpenCV's distortion has 4, 5 or 8",
            ),
            (
                format!(
                    "team = 1234\n{CAMERA}[camera.lens]\nwidth = 1280\nheight = 800\nfx = 1.0\nfy = 1.0\ncx = 0.0\ncy = 0.0\n"
                ),
                "calibrated at 1280x800, but the camera runs at 1600x1304",
            ),
            (
                format!("tag_size_m = 0.0\nteam = 1234\n{CAMERA}"),
                "tag_size_m",
            ),
            (
                format!("team = 1234\n{CAMERA}stream_scale = 2.0\n"),
                "stream_scale is the live view's share of the camera's size, 0.05 to 1, not 2",
            ),
            (
                format!("team = 1234\n{CAMERA}stream_fps = 90\n"),
                "stream_fps is from 1 to the camera's fps (60), not 90",
            ),
            (
                format!("team = 1234\n{CAMERA}stream_fps = 0\n"),
                "stream_fps is from 1",
            ),
            (
                format!("team = 1234\n{CAMERA}stream_quality = 0\n"),
                "stream_quality is a JPEG quality, 1 to 100, not 0",
            ),
            (
                format!("team = 1234\n{CAMERA}stream_quality = 101\n"),
                "stream_quality is a JPEG quality",
            ),
        ];
        for (text, why) in cases {
            let error = Config::parse(&text).unwrap_err().0;
            assert!(error.contains(why), "{why:?} not in {error:?}");
        }
    }

    #[test]
    fn the_example_file_is_valid() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("deploy/visor.example.toml");
        let config = Config::load(&path).unwrap();
        assert!(!config.cameras.is_empty());
    }
}
