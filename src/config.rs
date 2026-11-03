//! The configuration file, deployed with the program. An example is deploy/visor.example.toml.

use std::collections::HashSet;
use std::fmt;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::lens::Lens;

/// The port frames go to on the robot.
pub const DEFAULT_PORT: u16 = 5807;
const DEFAULT_FPS: u32 = 60;
const DEFAULT_MIN_TAG_PX: i32 = 16;
/// An FRC AprilTag's black square since 2024: 6.5 inches.
pub const DEFAULT_TAG_SIZE_M: f64 = 0.1651;

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
    /// Best a `/dev/v4l/by-id/` path, so a replugged camera is found again.
    pub device: PathBuf,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    /// The pixel format to ask for; without one, `GREY` if offered, else `YUYV`.
    pub format: Option<PixelFormat>,
    pub min_tag_px: i32,
    /// Added to every capture time: the spin test's result.
    pub time_offset_nanos: i64,
    /// V4L2 controls by the names `v4l2-ctl --list-ctrls` prints, set in this order.
    pub controls: Vec<(String, i64)>,
    /// The lens calibration at this resolution. Without one, tags are sent with their corners
    /// only, no poses.
    pub lens: Option<Lens>,
    /// The side of the tags' black square, in metres.
    pub tag_size_m: f64,
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
    #[serde(default)]
    controls: toml::Table,
    lens: Option<LensFile>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct LensFile {
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
        controls,
        lens,
        tag_size_m,
        name,
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
        assert!(camera.controls.is_empty());
        assert_eq!((camera.lens.as_ref(), camera.tag_size_m), (None, 0.1651));
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
            [camera.controls]
            auto_exposure = 1
            exposure_time_absolute = 10
            exposure_dynamic_framerate = false
            [camera.lens]
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
        let lens = camera.lens.as_ref().unwrap();
        assert_eq!(
            (lens.fx, lens.cy, lens.distortion[4], lens.distortion[5]),
            (1200.0, 651.5, 0.05, 0.0)
        );
        // In the file's order, which matters: manual exposure has to be on before exposure is set.
        let controls: Vec<(&str, i64)> = camera
            .controls
            .iter()
            .map(|(k, v)| (k.as_str(), *v))
            .collect();
        assert_eq!(
            controls,
            [
                ("auto_exposure", 1),
                ("exposure_time_absolute", 10),
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
                format!(
                    "team = 1234\n{CAMERA}[camera.lens]\nfx = 1.0\nfy = 1.0\ncx = 0.0\ncy = 0.0\ndistortion = [0.1]\n"
                ),
                "lens: OpenCV's distortion has 4, 5 or 8",
            ),
            (
                format!("tag_size_m = 0.0\nteam = 1234\n{CAMERA}"),
                "tag_size_m",
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
