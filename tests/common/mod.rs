// Each test file uses its own part of this module.
#![allow(dead_code)]

//! What the integration tests share: the rendered frames in testdata/ and their known corners.

use std::path::{Path, PathBuf};

use nalgebra::{Rotation3, Vector3};
use visor::image::GrayImage;

/// A rendered frame and the tags in it.
pub struct Scene {
    pub name: &'static str,
    pub image: GrayImage,
    pub tags: Vec<KnownTag>,
}

/// A tag's true corners, in NAPA's convention, and its true pose (see tools/render_frames.py).
pub struct KnownTag {
    pub id: i32,
    pub corners: [f64; 8],
    pub pose: TruePose,
}

/// Where the renderer put a tag: its rotation and centre in the camera's frame, in the tag frame
/// Visor solves for (src/pose.rs).
pub struct TruePose {
    pub rotation: Rotation3<f64>,
    pub translation_m: Vector3<f64>,
}

/// The renderer's lens (tools/render_frames.py): no distortion, an 80 degree diagonal, centred.
/// Its centre is half a pixel up and left in pixel-centre coordinates.
pub fn rendered_lens() -> visor::lens::Lens {
    let focal = 1600f64.hypot(1304.0) / 2.0 / 40f64.to_radians().tan();
    visor::lens::Lens::new(focal, focal, 799.5, 651.5, &[]).unwrap()
}

pub const SCENES: [&str; 4] = ["one-tag", "two-tags", "steep-far", "empty"];

fn testdata(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("testdata")
        .join(name)
}

pub fn scene(name: &'static str) -> Scene {
    let png = image::open(testdata(&format!("{name}.png")))
        .expect("a rendered frame")
        .to_luma8();
    let image = GrayImage {
        width: png.width(),
        height: png.height(),
        pixels: png.into_raw(),
    };
    let json: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(testdata(&format!("{name}.json"))).unwrap())
            .unwrap();
    let tags = json["tags"]
        .as_array()
        .unwrap()
        .iter()
        .map(|tag| {
            let points: Vec<f64> = tag["corners"]
                .as_array()
                .unwrap()
                .iter()
                .flat_map(|p| p.as_array().unwrap().iter().map(|v| v.as_f64().unwrap()))
                .collect();
            KnownTag {
                id: tag["id"].as_i64().unwrap() as i32,
                corners: points.try_into().unwrap(),
                pose: TruePose {
                    rotation: Rotation3::new(vector(&tag["pose"]["rvec"])),
                    translation_m: vector(&tag["pose"]["tvec"]),
                },
            }
        })
        .collect();
    Scene { name, image, tags }
}

fn vector(value: &serde_json::Value) -> Vector3<f64> {
    let numbers: Vec<f64> = value
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap())
        .collect();
    Vector3::from_column_slice(&numbers)
}

/// The worst corner's distance between two sets of corners, in pixels, in the order given.
pub fn worst_corner(a: &[f64; 8], b: &[f64; 8]) -> f64 {
    (0..4)
        .map(|i| (a[2 * i] - b[2 * i]).hypot(a[2 * i + 1] - b[2 * i + 1]))
        .fold(0.0, f64::max)
}
