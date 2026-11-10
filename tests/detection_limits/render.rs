//! A small renderer for the detection sweep: tag36h11 tags seen through a pinhole lens, over a
//! shaded background, then blurred, exposed and given sensor noise, all from a seed so every
//! machine renders the same frames.
//!
//! It works as tools/render_frames.py does, in Rust so the sweep's many frames are made as the
//! test runs rather than kept in testdata/: each pixel is the average of a 3 x 3 grid of samples
//! over it, as a sensor's pixels average the light that falls on them. Coordinates are NAPA's,
//! pixel-centre: pixel (0, 0) spans -0.5 to 0.5. The `the_renderer_draws_tags_as_opencv_does` test
//! checks it against the OpenCV-rendered frames.

use std::ffi::c_int;

use nalgebra::{Matrix3, Rotation3, Vector3};
use visor::image::GrayImage;

/// An FRC tag's black square, in metres.
pub const TAG_M: f64 = 0.1651;
/// The renderer's focal length in pixels: a ThriftyCam's 1600 x 1304 behind an 80 degree diagonal
/// lens, as tools/render_frames.py has it.
pub fn focal() -> f64 {
    1600f64.hypot(1304.0) / 2.0 / 40f64.to_radians().tan()
}

/// Samples per pixel along each axis.
const SUPERSAMPLE: usize = 3;
/// The sensor's read noise, one standard deviation in grey levels, as the rendered frames have.
const NOISE: f64 = 1.5;

unsafe extern "C" {
    fn napa_tag36h11_code(id: c_int, code: *mut u64) -> c_int;
}

/// A tag's 36 payload bits as NAPA's napa_core.h gives them: the 6 x 6 cells inside the black
/// border, row by row from the printed tag's top-left, most significant bit first, set for white.
pub fn tag36h11_code(id: i32) -> u64 {
    let mut code = 0;
    // SAFETY: `code` is a writable u64, which is all napa_tag36h11_code writes.
    let status = unsafe { napa_tag36h11_code(id, &mut code) };
    assert_eq!(status, 0, "tag36h11 has no tag {id}");
    code
}

/// What is printed inside a tag's black border.
#[derive(Debug, Clone, Copy)]
pub enum Payload {
    /// 36 cells, as `tag36h11_code` lays them out.
    Bits(u64),
    /// The same cells mirrored left to right, as a tag seen in a reflection.
    Mirrored(u64),
}

impl Payload {
    fn white(self, row: usize, column: usize) -> bool {
        let (bits, column) = match self {
            Payload::Bits(bits) => (bits, column),
            Payload::Mirrored(bits) => (bits, 5 - column),
        };
        (bits >> (35 - (row * 6 + column))) & 1 == 1
    }
}

/// A printed tag, where it is, and how its print looks at an exposure of 1.
#[derive(Debug, Clone, Copy)]
pub struct Placed {
    pub payload: Payload,
    /// The tag in the camera's frame, in Visor's tag frame (src/pose.rs): x right and y up as
    /// printed, z out of its face.
    pub rotation: Rotation3<f64>,
    pub translation_m: Vector3<f64>,
    /// The grey levels of the print's black and white at an exposure of 1.
    pub black: f64,
    pub white: f64,
}

/// What happens to the light between the scene and the frame.
#[derive(Debug, Clone, Copy)]
pub struct Optics {
    /// A Gaussian blur's standard deviation in pixels, for defocus or motion.
    pub blur_px: f64,
    /// How much brighter than the scene the frame is: 0.5 is half the exposure time or gain.
    pub exposure: f64,
}

/// A frame of a given size, through the renderer's lens centred on it.
pub struct Camera {
    pub width: u32,
    pub height: u32,
}

impl Camera {
    fn centre(&self) -> (f64, f64) {
        (
            (f64::from(self.width) - 1.0) / 2.0,
            (f64::from(self.height) - 1.0) / 2.0,
        )
    }

    /// Where a point in the camera's frame lands, in pixel-centre coordinates.
    pub fn project(&self, point: &Vector3<f64>) -> (f64, f64) {
        let (cx, cy) = self.centre();
        (
            focal() * point.x / point.z + cx,
            focal() * point.y / point.z + cy,
        )
    }

    /// The camera-frame point at `depth` that lands on pixel (`u`, `v`).
    pub fn point_at(&self, u: f64, v: f64, depth: f64) -> Vector3<f64> {
        let (cx, cy) = self.centre();
        Vector3::new(
            (u - cx) / focal() * depth,
            (v - cy) / focal() * depth,
            depth,
        )
    }

    /// A tag's black-square corners as NAPA gives them: the printed tag's top-left, top-right,
    /// bottom-right and bottom-left.
    pub fn corners(&self, tag: &Placed) -> [f64; 8] {
        let half = TAG_M / 2.0;
        let model = [(-half, half), (half, half), (half, -half), (-half, -half)];
        let mut corners = [0.0; 8];
        for (i, (x, y)) in model.into_iter().enumerate() {
            let (u, v) =
                self.project(&(tag.rotation * Vector3::new(x, y, 0.0) + tag.translation_m));
            corners[2 * i] = u;
            corners[2 * i + 1] = v;
        }
        corners
    }

    /// The frame: the background, the tags drawn over it in order, then the optics and the
    /// sensor's noise, from `seed`.
    pub fn render(&self, tags: &[Placed], optics: Optics, seed: u64) -> GrayImage {
        let mut random = SplitMix64(seed);
        let mut light = background(self.width as usize, self.height as usize, &mut random);
        for tag in tags {
            self.draw(&mut light, tag);
        }
        let light = blur(&light, self.width as usize, optics.blur_px);
        let pixels = light
            .iter()
            .map(|&level| {
                let noisy = level * optics.exposure + NOISE * random.gaussian();
                noisy.round().clamp(0.0, 255.0) as u8
            })
            .collect();
        GrayImage {
            width: self.width,
            height: self.height,
            pixels,
        }
    }

    /// Draws one tag, with its one-cell white quiet zone, over `light`.
    fn draw(&self, light: &mut [f64], tag: &Placed) {
        let cell = TAG_M / 8.0;
        let outer = 5.0 * cell;
        // The homography from the tag's plane to the frame, and back.
        let (cx, cy) = self.centre();
        let k = Matrix3::new(focal(), 0.0, cx, 0.0, focal(), cy, 0.0, 0.0, 1.0);
        let r = tag.rotation.matrix();
        let plane_to_frame = k * Matrix3::from_columns(&[
            r.column(0).into_owned(),
            r.column(1).into_owned(),
            tag.translation_m,
        ]);
        let frame_to_plane = plane_to_frame.try_inverse().expect("a tag the camera sees");

        // Only the pixels the tag's outer edge can touch.
        let reach = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].map(|(x, y)| {
            self.project(&(r * Vector3::new(x * outer, y * outer, 0.0) + tag.translation_m))
        });
        let (width, height) = (self.width as usize, self.height as usize);
        let low = |a: f64| (a.floor() as isize - 1).max(0) as usize;
        let x0 = low(reach.iter().map(|p| p.0).fold(f64::INFINITY, f64::min));
        let y0 = low(reach.iter().map(|p| p.1).fold(f64::INFINITY, f64::min));
        let x1 = (reach
            .iter()
            .map(|p| p.0)
            .fold(f64::NEG_INFINITY, f64::max)
            .ceil() as usize
            + 2)
        .min(width);
        let y1 = (reach
            .iter()
            .map(|p| p.1)
            .fold(f64::NEG_INFINITY, f64::max)
            .ceil() as usize
            + 2)
        .min(height);

        let samples = (SUPERSAMPLE * SUPERSAMPLE) as f64;
        for y in y0..y1 {
            for x in x0..x1 {
                let behind = light[y * width + x];
                let mut sum = 0.0;
                for sy in 0..SUPERSAMPLE {
                    for sx in 0..SUPERSAMPLE {
                        let u = x as f64 + (sx as f64 + 0.5) / SUPERSAMPLE as f64 - 0.5;
                        let v = y as f64 + (sy as f64 + 0.5) / SUPERSAMPLE as f64 - 0.5;
                        let p = frame_to_plane * Vector3::new(u, v, 1.0);
                        let (px, py) = (p.x / p.z, p.y / p.z);
                        sum += match printed(tag, px, py, cell) {
                            Some(level) => level,
                            None => behind,
                        };
                    }
                }
                light[y * width + x] = sum / samples;
            }
        }
    }
}

/// The print's grey level at (`x`, `y`) on the tag's plane, or `None` outside its quiet zone.
fn printed(tag: &Placed, x: f64, y: f64, cell: f64) -> Option<f64> {
    // Cells from the printed tag's top-left, ten across: the quiet zone, the black border, and the
    // payload's six.
    let column = ((x + 5.0 * cell) / cell).floor();
    let row = ((5.0 * cell - y) / cell).floor();
    if !(0.0..10.0).contains(&column) || !(0.0..10.0).contains(&row) {
        return None;
    }
    let (row, column) = (row as usize, column as usize);
    let white = match (row, column) {
        (0 | 9, _) | (_, 0 | 9) => true,
        (1 | 8, _) | (_, 1 | 8) => false,
        _ => tag.payload.white(row - 2, column - 2),
    };
    Some(if white { tag.white } else { tag.black })
}

/// Low-frequency shading with dark and light blocks over it, as a field wall and bumpers give.
fn background(width: usize, height: usize, random: &mut SplitMix64) -> Vec<f64> {
    const GRID: (usize, usize) = (11, 9);
    let coarse: Vec<f64> = (0..GRID.0 * GRID.1)
        .map(|_| 60.0 + 140.0 * random.unit())
        .collect();
    let mut light = vec![0.0; width * height];
    for y in 0..height {
        let gy = y as f64 / height as f64 * (GRID.1 - 1) as f64;
        let (row, fy) = (gy.floor() as usize, gy.fract());
        for x in 0..width {
            let gx = x as f64 / width as f64 * (GRID.0 - 1) as f64;
            let (column, fx) = (gx.floor() as usize, gx.fract());
            let at = |r: usize, c: usize| coarse[r * GRID.0 + c];
            let top = at(row, column) * (1.0 - fx) + at(row, column + 1) * fx;
            let bottom = at(row + 1, column) * (1.0 - fx) + at(row + 1, column + 1) * fx;
            light[y * width + x] = top * (1.0 - fy) + bottom * fy;
        }
    }
    for _ in 0..12 {
        let (x, y) = (random.below(width), random.below(height));
        let (w, h) = (20 + random.below(100), 20 + random.below(100));
        let level = 10.0 + 235.0 * random.unit();
        for row in y..(y + h).min(height) {
            light[row * width + x..row * width + (x + w).min(width)].fill(level);
        }
    }
    light
}

/// A Gaussian blur of standard deviation `sigma` pixels, along rows then down columns, the edges
/// repeated.
fn blur(light: &[f64], width: usize, sigma: f64) -> Vec<f64> {
    if sigma <= 0.0 {
        return light.to_vec();
    }
    let radius = (3.0 * sigma).ceil() as isize;
    let weights: Vec<f64> = (-radius..=radius)
        .map(|i| (-(i * i) as f64 / (2.0 * sigma * sigma)).exp())
        .collect();
    let total: f64 = weights.iter().sum();
    let weights: Vec<f64> = weights.iter().map(|w| w / total).collect();
    let height = light.len() / width;
    let along_rows = blur_lines(light, &weights, width, height, |line, i| line * width + i);
    blur_lines(&along_rows, &weights, height, width, |line, i| {
        i * width + line
    })
}

/// One pass of the blur over `lines` lines of `length` pixels each, `at` giving each pixel's index.
fn blur_lines(
    from: &[f64],
    weights: &[f64],
    length: usize,
    lines: usize,
    at: impl Fn(usize, usize) -> usize,
) -> Vec<f64> {
    let radius = (weights.len() / 2) as isize;
    let mut out = vec![0.0; from.len()];
    for line in 0..lines {
        for i in 0..length {
            let mut sum = 0.0;
            for (k, weight) in weights.iter().enumerate() {
                let j = (i as isize + k as isize - radius).clamp(0, length as isize - 1);
                sum += weight * from[at(line, j as usize)];
            }
            out[at(line, i)] = sum;
        }
    }
    out
}

/// SplitMix64: small, fast, and the same on every machine.
struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn below(&mut self, n: usize) -> usize {
        (self.unit() * n as f64) as usize
    }

    /// A standard normal, by Box and Muller.
    fn gaussian(&mut self) -> f64 {
        let u = 1.0 - self.unit();
        let v = self.unit();
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}
