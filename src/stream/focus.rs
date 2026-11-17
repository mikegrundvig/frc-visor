//! How sharply a camera sees a tag: how many pixels wide the edge of the tag's black square is in
//! the frame. A printed tag's edge is a clean step from black to white, so in a frame it is as wide
//! as the lens and the sensor make it: about a pixel and a half at best, more the further the lens
//! is from focus. Smaller is sharper.
//!
//! The width is the step's height over its steepest slope, so it doesn't change with the exposure,
//! the gain, the light or what else is in view, and little with how far away the tag is. It is read
//! on lines across each side of the square, and a tag's width is their median.

use crate::image::GrayImage;
use crate::wire::Tag;

/// A tag is measured when its black square is at least this many pixels along each side and across
/// from each side to the one opposite: under it, the square's border and the white round it are
/// each under four pixels, too few to show a step's two levels.
const SHORTEST_SIDE_PX: f64 = 32.0;

/// The distance between samples along a line across an edge, in pixels.
const SPACING_PX: f64 = 0.5;

/// How far each side of an edge a line reaches, as a share of one of the tag's cells (an eighth of
/// its side): the black border is one cell wide and so is the white round a tag, so this stays
/// inside both. No further than [`FURTHEST_PX`]: a large tag's edge needs no more.
const REACH_CELLS: f64 = 0.8;
const FURTHEST_PX: f64 = 12.0;

/// The part of each side lines cross: its middle, clear of the corners, where two edges meet.
const ALONG: (f64, f64) = (0.2, 0.8);

/// A line counts when the tag's white is at least this many grey levels above its black there:
/// under it, noise is too much of the step.
const LEAST_STEP: f64 = 24.0;

/// A tag's width is given when at least this many of its lines counted.
const FEWEST_LINES: usize = 8;

/// The width of a tag's edges in the frame, in pixels: the median over lines across the four sides
/// of its black square. None for a tag too small, too dim or too near the frame's edge to measure.
pub fn edge_px(image: &GrayImage, corners: &[f32; 8]) -> Option<f64> {
    let point = |i: usize| (f64::from(corners[2 * i]), f64::from(corners[2 * i + 1]));
    let points = [point(0), point(1), point(2), point(3)];
    let middle = (
        points.iter().map(|p| p.0).sum::<f64>() / 4.0,
        points.iter().map(|p| p.1).sum::<f64>() / 4.0,
    );
    let mut widths = Vec::new();
    for side in 0..4 {
        let (from, to) = (points[side], points[(side + 1) % 4]);
        let length = (to.0 - from.0).hypot(to.1 - from.1);
        if length < SHORTEST_SIDE_PX {
            return None;
        }
        // Across the side, pointing into the square.
        let mut inward = (-(to.1 - from.1) / length, (to.0 - from.0) / length);
        let mid = ((from.0 + to.0) / 2.0, (from.1 + to.1) / 2.0);
        if inward.0 * (middle.0 - mid.0) + inward.1 * (middle.1 - mid.1) < 0.0 {
            inward = (-inward.0, -inward.1);
        }
        // A cell's depth across this side: an eighth of the way to the opposite side. A tag seen at
        // an angle has sides of different lengths, and its cells are as deep as the other pair's.
        let (far_from, far_to) = (points[(side + 2) % 4], points[(side + 3) % 4]);
        let opposite = ((far_from.0 + far_to.0) / 2.0, (far_from.1 + far_to.1) / 2.0);
        let depth = (opposite.0 - mid.0) * inward.0 + (opposite.1 - mid.1) * inward.1;
        let reach = (REACH_CELLS * depth / 8.0).min(FURTHEST_PX);
        if depth < SHORTEST_SIDE_PX {
            return None;
        }
        let lines = ((length / 6.0) as usize).clamp(4, 24);
        for line in 0..lines {
            let along = ALONG.0 + (ALONG.1 - ALONG.0) * (line as f64 + 0.5) / lines as f64;
            let on_edge = (
                from.0 + along * (to.0 - from.0),
                from.1 + along * (to.1 - from.1),
            );
            if let Some(width) = across(image, on_edge, inward, reach) {
                widths.push(width);
            }
        }
    }
    if widths.len() < FEWEST_LINES {
        return None;
    }
    widths.sort_by(f64::total_cmp);
    Some(widths[widths.len() / 2])
}

/// One line across an edge, from the white outside it to the black inside: the step's height over
/// its steepest slope. None when the line leaves the frame or the step is too small to trust.
fn across(image: &GrayImage, on_edge: (f64, f64), inward: (f64, f64), reach: f64) -> Option<f64> {
    let count = (2.0 * reach / SPACING_PX) as usize + 1;
    let mut levels = Vec::with_capacity(count);
    for i in 0..count {
        let offset = i as f64 * SPACING_PX - reach;
        levels.push(level_at(
            image,
            on_edge.0 + offset * inward.0,
            on_edge.1 + offset * inward.1,
        )?);
    }
    // The two levels, each the mean of the line's end furthest from the edge.
    let ends = (count / 6).max(2);
    let mean = |levels: &[f64]| levels.iter().sum::<f64>() / levels.len() as f64;
    let step = mean(&levels[..ends]) - mean(&levels[count - ends..]);
    if step < LEAST_STEP {
        return None;
    }
    // The slope over a pixel, centred on each sample.
    let per_pixel = (1.0 / SPACING_PX) as usize;
    let steepest = (0..count - per_pixel)
        .map(|i| levels[i] - levels[i + per_pixel])
        .fold(0.0, f64::max);
    (steepest > 0.0).then(|| step / steepest)
}

/// The frame's grey level at a point between pixels, from the four round it. None outside the frame.
fn level_at(image: &GrayImage, x: f64, y: f64) -> Option<f64> {
    let (width, height) = (image.width as usize, image.height as usize);
    if x < 0.0 || y < 0.0 || x > (width as f64 - 1.0) || y > (height as f64 - 1.0) {
        return None;
    }
    let (left, top) = (
        (x as usize).min(width.saturating_sub(2)),
        (y as usize).min(height.saturating_sub(2)),
    );
    if width < 2 || height < 2 {
        return None;
    }
    let (across, down) = (x - left as f64, y - top as f64);
    let at = |column: usize, row: usize| f64::from(image.pixels[row * width + column]);
    let upper = at(left, top) * (1.0 - across) + at(left + 1, top) * across;
    let lower = at(left, top + 1) * (1.0 - across) + at(left + 1, top + 1) * across;
    Some(upper * (1.0 - down) + lower * down)
}

/// A frame's focus: each tag's edge width, and which tag the view goes by.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Measured {
    /// Each tag's edge width, in the frame's order of tags: None for one that can't be measured.
    pub edges: Vec<Option<f64>>,
    /// The measured tag nearest the middle of the frame, the one a camera being focused is pointed
    /// at: its place among the frame's tags.
    pub nearest: Option<usize>,
}

impl Measured {
    /// The edge width the view goes by: the measured tag nearest the frame's middle.
    pub fn edge_px(&self) -> Option<f64> {
        self.edges[self.nearest?]
    }
}

/// Measures every tag in a frame.
pub fn measure(image: &GrayImage, tags: &[Tag]) -> Measured {
    let edges: Vec<Option<f64>> = tags
        .iter()
        .map(|tag| edge_px(image, &tag.corners))
        .collect();
    let centre = (
        (f64::from(image.width) - 1.0) / 2.0,
        (f64::from(image.height) - 1.0) / 2.0,
    );
    let from_centre = |tag: &Tag| {
        let x = (0..4).map(|i| f64::from(tag.corners[2 * i])).sum::<f64>() / 4.0;
        let y = (0..4)
            .map(|i| f64::from(tag.corners[2 * i + 1]))
            .sum::<f64>()
            / 4.0;
        (x - centre.0).hypot(y - centre.1)
    };
    let nearest = (0..tags.len())
        .filter(|&i| edges[i].is_some())
        .min_by(|&a, &b| from_centre(&tags[a]).total_cmp(&from_centre(&tags[b])));
    Measured { edges, nearest }
}

/// How much of each new reading goes into the smoothed one: the rest is what it was.
const NEW_SHARE: f64 = 0.3;

/// The best is taken from the smoothed reading once this many readings have come in a row, so one
/// frame's noise is never the best.
const SETTLED_AFTER: u32 = 3;

/// A camera's focus as someone turning its lens watches it: each reading smoothed over the last
/// few pictures, and the best (smallest) the smoothed reading has been.
#[derive(Debug, Default)]
pub struct Tracker {
    smoothed: Option<f64>,
    in_a_row: u32,
    best: Option<f64>,
}

/// What the view's line says of focus.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Shown {
    /// The edge width now, smoothed: None while no tag in view can be measured.
    pub now_px: Option<f64>,
    /// The smallest the smoothed width has been since the best was last forgotten.
    pub best_px: Option<f64>,
}

impl Tracker {
    /// Takes a picture's reading: None when no tag in it could be measured.
    pub fn add(&mut self, edge_px: Option<f64>) -> Shown {
        match edge_px {
            Some(reading) => {
                let smoothed = match self.smoothed {
                    Some(was) => was + NEW_SHARE * (reading - was),
                    None => reading,
                };
                self.smoothed = Some(smoothed);
                self.in_a_row += 1;
                if self.in_a_row >= SETTLED_AFTER && self.best.is_none_or(|best| smoothed < best) {
                    self.best = Some(smoothed);
                }
            }
            // The next reading starts afresh: it isn't averaged with one from before the gap.
            None => {
                self.smoothed = None;
                self.in_a_row = 0;
            }
        }
        Shown {
            now_px: self.smoothed,
            best_px: self.best,
        }
    }

    /// Forgets everything, the best included.
    pub fn forget(&mut self) {
        *self = Tracker::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame with a black square on white whose edges ramp over `width_px` pixels: `turn` turns
    /// the square about its middle, in degrees. Each pixel is the mean of a 4 x 4 grid of samples.
    fn square(
        side: f64,
        width_px: f64,
        turn: f64,
        black: f64,
        white: f64,
    ) -> (GrayImage, [f32; 8]) {
        let size = 400usize;
        let (sin, cos) = turn.to_radians().sin_cos();
        let centre = (size as f64 - 1.0) / 2.0;
        let mut image = GrayImage::new(size as u32, size as u32);
        for y in 0..size {
            for x in 0..size {
                let mut sum = 0.0;
                for sample in 0..16 {
                    let sx = x as f64 + ((sample % 4) as f64 + 0.5) / 4.0 - 0.5 - centre;
                    let sy = y as f64 + ((sample / 4) as f64 + 0.5) / 4.0 - 0.5 - centre;
                    // In the square's own frame, how far inside its nearest edge: negative outside.
                    let (u, v) = (sx * cos + sy * sin, -sx * sin + sy * cos);
                    let inside = side / 2.0 - u.abs().max(v.abs());
                    let share = (inside / width_px + 0.5).clamp(0.0, 1.0);
                    sum += white + (black - white) * share;
                }
                image.pixels[y * size + x] = (sum / 16.0).round() as u8;
            }
        }
        let half = side / 2.0;
        let mut corners = [0f32; 8];
        for (i, (u, v)) in [(-half, -half), (half, -half), (half, half), (-half, half)]
            .into_iter()
            .enumerate()
        {
            corners[2 * i] = (centre + u * cos - v * sin) as f32;
            corners[2 * i + 1] = (centre + u * sin + v * cos) as f32;
        }
        (image, corners)
    }

    #[test]
    fn an_edge_s_width_is_read_whatever_the_tag_s_size_turn_or_contrast() {
        // Each row: the square's side, its edges' true width, its turn in degrees, its black and
        // white. A ramp's width is its step over its slope, so each should read its own width, a
        // little more for the pixels' own width.
        let cases = [
            (160.0, 2.0, 7.0, 30.0, 215.0),
            (160.0, 3.0, 7.0, 30.0, 215.0),
            (160.0, 5.0, 7.0, 30.0, 215.0),
            (160.0, 3.0, 33.0, 30.0, 215.0),
            (160.0, 3.0, 45.0, 30.0, 215.0),
            (60.0, 3.0, 7.0, 30.0, 215.0),
            (280.0, 3.0, 7.0, 30.0, 215.0),
            // Half the exposure, and a dim tag.
            (160.0, 3.0, 7.0, 15.0, 108.0),
            (160.0, 3.0, 7.0, 40.0, 75.0),
        ];
        for (side, width, turn, black, white) in cases {
            let (image, corners) = square(side, width, turn, black, white);
            let read = edge_px(&image, &corners).unwrap();
            assert!(
                (width - 0.1..width + 0.6).contains(&read),
                "side {side}, width {width}, turned {turn}, {black} to {white}: read {read:.3}"
            );
        }
    }

    #[test]
    fn a_tag_too_small_too_faint_or_off_the_frame_isn_t_measured() {
        let (image, corners) = square(160.0, 3.0, 7.0, 30.0, 215.0);
        assert!(edge_px(&image, &corners).is_some());
        let cases = [
            ("too small", square(28.0, 2.0, 7.0, 30.0, 215.0)),
            ("too faint", square(160.0, 3.0, 7.0, 100.0, 115.0)),
        ];
        for (what, (image, corners)) in cases {
            assert_eq!(edge_px(&image, &corners), None, "{what}");
        }
        // Off the frame: the same square, its corners said to be 200 px further left than they are.
        let mut moved = corners;
        for i in 0..4 {
            moved[2 * i] -= 320.0;
        }
        assert_eq!(edge_px(&image, &moved), None);
        // A frame too small to have a pixel's neighbours: nothing, not a panic.
        assert_eq!(edge_px(&GrayImage::new(1, 1), &corners), None);
        assert_eq!(edge_px(&GrayImage::new(0, 0), &corners), None);
    }

    fn tag(corners: [f32; 8]) -> Tag {
        Tag {
            id: 1,
            hamming: 0,
            decision_margin: 50.0,
            corners,
            poses: None,
        }
    }

    #[test]
    fn the_view_goes_by_the_measured_tag_nearest_the_middle() {
        let (image, middle) = square(160.0, 3.0, 7.0, 30.0, 215.0);
        // A second "tag" that isn't in the frame's pixels at all, and one off the frame.
        let blank = [20.0, 20.0, 60.0, 20.0, 60.0, 60.0, 20.0, 60.0];
        let off = [-80.0, 150.0, 0.0, 150.0, 0.0, 230.0, -80.0, 230.0];
        let cases = [
            (vec![], None),
            (vec![tag(blank), tag(off)], None),
            (vec![tag(blank), tag(middle)], Some(1)),
            (vec![tag(middle), tag(off)], Some(0)),
        ];
        for (tags, nearest) in cases {
            let measured = measure(&image, &tags);
            assert_eq!(measured.nearest, nearest);
            assert_eq!(measured.edges.len(), tags.len());
            assert_eq!(measured.edge_px().is_some(), nearest.is_some());
        }
    }

    #[test]
    fn readings_are_smoothed_and_the_best_is_the_smallest_smoothed_one() {
        // Each row: a picture's reading, then the smoothed reading and the best after it.
        let steps = [
            (Some(4.0), Some(4.0), None),
            (Some(4.0), Some(4.0), None),
            // The third in a row: settled, so it's the best so far.
            (Some(4.0), Some(4.0), Some(4.0)),
            (Some(2.0), Some(3.4), Some(3.4)),
            (Some(2.0), Some(2.98), Some(2.98)),
            // Past focus: the reading rises, the best stays.
            (Some(5.0), Some(3.586), Some(2.98)),
            // No tag: nothing now, the best kept.
            (None, None, Some(2.98)),
            // Back: afresh, not averaged with before, and not the best until it has settled.
            (Some(1.0), Some(1.0), Some(2.98)),
            (Some(1.0), Some(1.0), Some(2.98)),
            (Some(1.0), Some(1.0), Some(1.0)),
        ];
        let mut tracker = Tracker::default();
        for (i, (reading, now, best)) in steps.into_iter().enumerate() {
            let shown = tracker.add(reading);
            let near = |a: Option<f64>, b: Option<f64>| match (a, b) {
                (Some(a), Some(b)) => (a - b).abs() < 1e-9,
                (None, None) => true,
                _ => false,
            };
            assert!(near(shown.now_px, now), "step {i}: now {:?}", shown.now_px);
            assert!(
                near(shown.best_px, best),
                "step {i}: best {:?}",
                shown.best_px
            );
        }
        tracker.forget();
        assert_eq!(tracker.add(None), Shown::default());
    }
}
