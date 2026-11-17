//! How a tag is lit in a frame: the grey of its black and of its white, and whether that's light a
//! focus reading can be trusted in. The focus number (the width of the tag's edge, [`super::focus`])
//! is the step's height over its steepest slope, which holds only while the camera's response is
//! straight across the step. A real camera's isn't near the top of its range, nor at the very
//! bottom, so a tag lit too brightly reads falsely sharp and one too dim reads soft.

use crate::image::GrayImage;

/// A tag's white above this is too bright to trust. Measured on a ThriftyCam at 1600 x 1304, the
/// least gain, exposure stepped from 2 to 10 ms: a sharp tag near the middle read 1.46 to 1.55 px
/// with its white from 92 to 168, then falsely sharper as the white rose (1.37 px at 175, 1.34 at
/// 183, 1.18 at 190, 1.04 at 201, 0.97 at 208); a second sharp tag held 1.43 to 1.57 to a white of
/// 192, then read 1.21 at 200; soft tags (about 1.9 px) held to 203 and read 1.63 at 210. A sharp
/// edge is the first to go, from about 175, and readings are flat within 3% from about 90 to 170.
/// So the line is drawn at 170.
pub const TOO_BRIGHT_ABOVE: u8 = 170;

/// A tag's white below this is too dim to trust: the same camera read a sharp tag's edge soft, 2.03
/// px, with the white at 46 (its black 11), and right from 86 up; a soft tag read 1.97 px at 64
/// and 1.95 at 77, no worse than brighter. Kept at 80, between 46 and 86, nearer the good side.
pub const TOO_DIM_BELOW: u8 = 80;

/// The share of a tag's pixels darker than its black, and brighter than its white: its 10th and
/// 90th percentiles. Inside the black square about half the payload's cells are white, so the 90th
/// percentile is well inside them and the 10th well inside the black, and a few pixels of edge or
/// glare move neither.
const BLACK_PERCENTILE: usize = 10;
const WHITE_PERCENTILE: usize = 90;

/// At most about this many pixels are read inside a tag: enough for its percentiles, few enough that
/// a tag filling the frame costs the view's thread no more than a small one.
const MOST_SAMPLES: f64 = 10_000.0;

/// A tag's black and white in a frame, 0 to 255.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TagLight {
    pub black: u8,
    pub white: u8,
}

/// Whether a tag is lit well enough for its focus reading to count.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Light {
    /// Its white is below [`TOO_DIM_BELOW`]: the reading would be too soft.
    TooDim,
    /// Its white is between the two: the reading counts.
    Good,
    /// Its white is above [`TOO_BRIGHT_ABOVE`]: the reading would be falsely sharp.
    TooBright,
}

impl TagLight {
    /// Whether this light is one a focus reading can be trusted in.
    pub fn verdict(&self) -> Light {
        if self.white > TOO_BRIGHT_ABOVE {
            Light::TooBright
        } else if self.white < TOO_DIM_BELOW {
            Light::TooDim
        } else {
            Light::Good
        }
    }
}

impl Light {
    /// The word for it in a frame's facts (docs/live-view.md).
    pub fn word(&self) -> &'static str {
        match self {
            Light::TooDim => "too_dim",
            Light::Good => "good",
            Light::TooBright => "too_bright",
        }
    }
}

/// The light of the tag whose black square has these corners: the 10th and 90th percentiles of the
/// pixels inside the square, on the frame. None when none of its pixels are in the frame.
pub fn of(image: &GrayImage, corners: &[f32; 8]) -> Option<TagLight> {
    let point = |i: usize| (f64::from(corners[2 * i]), f64::from(corners[2 * i + 1]));
    let points = [point(0), point(1), point(2), point(3)];
    let (width, height) = (f64::from(image.width), f64::from(image.height));
    let low = |a: f64| a.floor().clamp(0.0, (width - 1.0).max(0.0));
    let lowest =
        |pick: fn(&(f64, f64)) -> f64| points.iter().map(pick).fold(f64::INFINITY, f64::min);
    let highest =
        |pick: fn(&(f64, f64)) -> f64| points.iter().map(pick).fold(f64::NEG_INFINITY, f64::max);
    if image.width == 0 || image.height == 0 || !lowest(|p| p.0).is_finite() {
        return None;
    }
    let (left, right) = (
        low(lowest(|p| p.0)),
        highest(|p| p.0).ceil().min(width - 1.0),
    );
    let (top, bottom) = (
        lowest(|p| p.1).floor().clamp(0.0, (height - 1.0).max(0.0)),
        highest(|p| p.1).ceil().min(height - 1.0),
    );
    if right < left || bottom < top {
        return None;
    }
    // Every so many pixels across and down, so a large tag is sampled, not read whole.
    let area = (right - left + 1.0) * (bottom - top + 1.0);
    let every = (area / MOST_SAMPLES).sqrt().ceil().max(1.0) as usize;
    let mut counts = [0u32; 256];
    let mut sampled = 0u32;
    for y in (top as usize..=bottom as usize).step_by(every) {
        for x in (left as usize..=right as usize).step_by(every) {
            if inside(&points, (x as f64, y as f64)) {
                counts[image.pixels[y * image.width as usize + x] as usize] += 1;
                sampled += 1;
            }
        }
    }
    if sampled == 0 {
        return None;
    }
    Some(TagLight {
        black: percentile(&counts, sampled, BLACK_PERCENTILE),
        white: percentile(&counts, sampled, WHITE_PERCENTILE),
    })
}

/// Whether a point is inside the quadrilateral: on the same side of each of its four sides. A tag's
/// square seen through a lens is convex, whichever way round its corners go.
fn inside(points: &[(f64, f64); 4], at: (f64, f64)) -> bool {
    let mut sides = [0.0; 4];
    for (i, side) in sides.iter_mut().enumerate() {
        let (from, to) = (points[i], points[(i + 1) % 4]);
        *side = (to.0 - from.0) * (at.1 - from.1) - (to.1 - from.1) * (at.0 - from.0);
    }
    sides.iter().all(|s| *s >= 0.0) || sides.iter().all(|s| *s <= 0.0)
}

/// The grey level below which `share` percent of the counted pixels are.
fn percentile(counts: &[u32; 256], total: u32, share: usize) -> u8 {
    let wanted = (u64::from(total) * share as u64).div_ceil(100).max(1);
    let mut seen = 0u64;
    for (level, count) in counts.iter().enumerate() {
        seen += u64::from(*count);
        if seen >= wanted {
            return level as u8;
        }
    }
    255
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame of `background` with a square from (100, 100) to (300, 300): its left half `black`,
    /// its right half `white`, as a tag's square is about half each.
    fn frame(background: u8, black: u8, white: u8) -> (GrayImage, [f32; 8]) {
        let mut image = GrayImage::new(400, 400);
        for y in 0..400 {
            for x in 0..400 {
                let level = match (x, y) {
                    (100..=199, 100..=300) => black,
                    (200..=300, 100..=300) => white,
                    _ => background,
                };
                image.pixels[y * 400 + x] = level;
            }
        }
        (
            image,
            [100.0, 100.0, 300.0, 100.0, 300.0, 300.0, 100.0, 300.0],
        )
    }

    #[test]
    fn a_tag_s_light_is_its_inside_s_black_and_white_and_its_verdict_follows_its_white() {
        // Each row: the frame round the tag, its black and white, and the verdict.
        let cases = [
            (255, 20, 140, Light::Good),
            (0, 20, 140, Light::Good),
            (128, 20, TOO_BRIGHT_ABOVE, Light::Good),
            (128, 60, TOO_BRIGHT_ABOVE + 1, Light::TooBright),
            (128, 76, 250, Light::TooBright),
            (128, 50, 236, Light::TooBright),
            (128, 11, 46, Light::TooDim),
            (128, 11, TOO_DIM_BELOW - 1, Light::TooDim),
            (128, 11, TOO_DIM_BELOW, Light::Good),
        ];
        for (background, black, white, verdict) in cases {
            let (image, corners) = frame(background, black, white);
            let light = of(&image, &corners).unwrap();
            assert_eq!(
                light,
                TagLight { black, white },
                "{background} {black} {white}"
            );
            assert_eq!(light.verdict(), verdict, "{black} {white}");
        }
    }

    #[test]
    fn a_tag_off_the_frame_is_read_where_it_is_on_it_and_not_at_all_when_it_isn_t() {
        let (image, corners) = frame(128, 20, 140);
        // Its right half moved off the frame's right edge: what's left is mostly white.
        let mut moved = corners;
        for i in 0..4 {
            moved[2 * i] += 150.0;
        }
        let light = of(&image, &moved).unwrap();
        assert_eq!(light.white, 140);
        // Wholly off it, turned the other way round, and in a frame with no pixels.
        let mut gone = corners;
        for i in 0..4 {
            gone[2 * i] -= 500.0;
        }
        assert_eq!(of(&image, &gone), None);
        let reversed = [100.0, 300.0, 300.0, 300.0, 300.0, 100.0, 100.0, 100.0];
        assert_eq!(of(&image, &reversed), of(&image, &corners));
        assert_eq!(of(&GrayImage::new(0, 0), &corners), None);
        assert_eq!(of(&image, &[f32::NAN; 8]), None);
    }
}
