//! A live view's picture: the camera's frame made smaller, each tag outlined with its ID, a line
//! along the top with the detection's frame rate, the exposure and gain and the brightness, a line
//! under it with the focus, and the tag the focus is read from magnified in a corner, as a JPEG. Resizing
//! is fast_image_resize's, drawing embedded-graphics', and the JPEG jpeg-encoder's.

use std::convert::Infallible;

use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_7X13, FONT_9X15_BOLD};
use embedded_graphics::mono_font::{MonoFont, MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Gray8;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Polyline, PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};
use fast_image_resize::images::{Image, ImageRef};
use fast_image_resize::{FilterType, PixelType, ResizeAlg, ResizeOptions, Resizer};
use jpeg_encoder::{ColorType, Encoder};

use crate::brightness::Brightness;
use crate::config::Stream;
use crate::image::GrayImage;
use crate::settings::InEffect;
use crate::stream::feed::Detected;
use crate::stream::focus::Shown;

/// Pictures this wide or wider get the larger lettering.
const LARGE_FROM: u32 = 640;

/// How many times larger than in the camera's frame the magnified part is drawn: each of the
/// frame's pixels as a square this many of the picture's pixels across, so a soft edge is seen.
const MAGNIFIED: u32 = 3;

/// The magnified part's side, as a share of the picture's height.
const MAGNIFIED_SHARE: u32 = 3;

/// A picture shorter than this gets no magnified part: there's no room for one worth looking at.
const MAGNIFIED_FROM: u32 = 180;

/// Makes a camera's pictures, keeping its buffers from one to the next.
pub struct Painter {
    stream: Stream,
    resizer: Resizer,
    small: Vec<u8>,
    /// The point of the frame the magnified part is centred on: a corner of the tag the focus was
    /// last read from. None until one has been.
    looked_at: Option<(f32, f32)>,
}

impl Painter {
    pub fn new(stream: Stream) -> Painter {
        Painter {
            stream,
            resizer: Resizer::new(),
            small: Vec::new(),
            looked_at: None,
        }
    }

    /// Looks at the frame's middle again, until a tag's focus is next read: for a new watcher.
    pub fn look_afresh(&mut self) {
        self.looked_at = None;
    }

    /// A frame's picture, as a JPEG, with its line saying `fps` (none while it's being measured)
    /// and the line under it the camera's `focus`, read from the tag at `measured` among the
    /// frame's (none when no tag could be measured).
    pub fn paint(
        &mut self,
        frame: &Detected,
        fps: Option<f64>,
        focus: Shown,
        measured: Option<usize>,
    ) -> Result<Vec<u8>, String> {
        let image = &frame.image;
        let (width, height) = self.stream.size(image.width, image.height);
        self.small.resize(width as usize * height as usize, 0);
        let source = ImageRef::new(image.width, image.height, &image.pixels, PixelType::U8)
            .map_err(|e| format!("the frame: {e}"))?;
        let mut small = Image::from_slice_u8(width, height, &mut self.small, PixelType::U8)
            .map_err(|e| format!("the picture: {e}"))?;
        // The mean of each pixel's patch of the frame: what a smaller sensor would have seen.
        let options = ResizeOptions::new().resize_alg(ResizeAlg::Convolution(FilterType::Box));
        self.resizer
            .resize(&source, &mut small, &options)
            .map_err(|e| format!("resizing: {e}"))?;

        let mut canvas = Canvas {
            pixels: &mut self.small,
            width,
            height,
        };
        let scale = f64::from(width) / f64::from(image.width);
        let large = width >= LARGE_FROM;
        for tag in &frame.tags {
            outline(&mut canvas, &tag.corners, scale, tag.id, large);
        }
        // The corner of the measured tag's black square that is its top-left as printed: its edge
        // is what the focus is read from. It's still looked at once the tag is lost, as it is
        // when the lens is turned far from focus, so the eye can bring it back.
        if let Some(tag) = measured.and_then(|i| frame.tags.get(i)) {
            self.looked_at = Some((tag.corners[0], tag.corners[1]));
        }
        magnify(&mut canvas, image, scale, self.looked_at);
        let line = caption(fps, frame.settings, frame.brightness);
        let font = if large { &FONT_7X13 } else { &FONT_6X10 };
        Text::with_baseline(&line, Point::new(2, 2), lettering(font), Baseline::Top)
            .draw(&mut canvas)
            .unwrap_or_else(|never| match never {});
        let second = Point::new(2, 4 + font.character_size.height as i32);
        let line = focus_line(focus);
        Text::with_baseline(&line, second, lettering(font), Baseline::Top)
            .draw(&mut canvas)
            .unwrap_or_else(|never| match never {});
        if let Some(share) = share_of_best(focus) {
            let after = second
                + Point::new(
                    (line.len() as u32 * font.character_size.width) as i32 + 8,
                    0,
                );
            bar(&mut canvas, after, font.character_size.height, share);
        }

        let mut jpeg = Vec::with_capacity(self.small.len() / 8);
        Encoder::new(&mut jpeg, self.stream.quality)
            .encode(&self.small, width as u16, height as u16, ColorType::Luma)
            .map_err(|e| format!("encoding: {e}"))?;
        Ok(jpeg)
    }
}

/// The line along the top: `59.8 fps  exposure 1.0 ms  gain 40  brightness 37 / 201`.
pub fn caption(fps: Option<f64>, settings: InEffect, brightness: Brightness) -> String {
    let rate = match fps {
        Some(fps) => format!("{fps:.1} fps"),
        None => "-- fps".to_string(),
    };
    let exposure = match (settings.auto_exposure, settings.exposure_us) {
        (Some(true), _) => "auto exposure".to_string(),
        (_, Some(us)) => format!("exposure {:.1} ms", f64::from(us) / 1000.0),
        (_, None) => "exposure unknown".to_string(),
    };
    let gain = match settings.gain {
        Some(gain) => format!("gain {gain}"),
        None => "no gain".to_string(),
    };
    format!(
        "{rate}  {exposure}  {gain}  brightness {} / {}",
        brightness.median, brightness.high
    )
}

/// The line under it: `focus: a tag's edge is 2.4 px wide, 1.9 at best`. Smaller is sharper.
pub fn focus_line(focus: Shown) -> String {
    let best =
        |best: Option<f64>| best.map_or(String::new(), |best| format!(", {best:.1} at best"));
    match focus.now_px {
        Some(now) => format!(
            "focus: a tag's edge is {now:.1} px wide{}",
            best(focus.best_px)
        ),
        None => format!("focus: no tag to measure{}", best(focus.best_px)),
    }
}

/// How near its best the focus is now, for the bar: 1 at the best, less the wider the edge is.
/// None until there is both a reading and a best.
pub fn share_of_best(focus: Shown) -> Option<f64> {
    let (now, best) = (focus.now_px?, focus.best_px?);
    Some((best / now).clamp(0.0, 1.0))
}

/// A bar `share` full, white in a white outline on black: full when the focus is at its best.
fn bar(canvas: &mut Canvas, at: Point, height: u32, share: f64) {
    let size = Size::new(10 * height, height);
    let inside = Size::new(((size.width - 4) as f64 * share).round() as u32, height - 4);
    let drawn = Rectangle::new(at, size)
        .into_styled(PrimitiveStyle::with_fill(Gray8::BLACK))
        .draw(canvas)
        .and_then(|()| {
            Rectangle::new(at, size)
                .into_styled(PrimitiveStyle::with_stroke(Gray8::WHITE, 1))
                .draw(canvas)
        })
        .and_then(|()| {
            Rectangle::new(at + Point::new(2, 2), inside)
                .into_styled(PrimitiveStyle::with_fill(Gray8::WHITE))
                .draw(canvas)
        });
    drawn.unwrap_or_else(|never| match never {});
}

/// Draws a part of the camera's frame in the picture's bottom-right corner, each of the frame's
/// pixels [`MAGNIFIED`] of the picture's across and nothing smoothed, so the eye sees the frame's
/// own pixels: an edge in focus is a clean step there, and a soft one a ramp. The part is centred
/// on `looked_at`, a point of the frame, as nearly as the frame's edges allow, and on the frame's
/// middle when there's none; a box in the picture marks it. Nothing on a picture too small for it.
fn magnify(canvas: &mut Canvas, image: &GrayImage, scale: f64, looked_at: Option<(f32, f32)>) {
    if canvas.height < MAGNIFIED_FROM {
        return;
    }
    // The part of the frame shown, in the frame's pixels, and its side in the picture's.
    let part = (canvas.height / MAGNIFIED_SHARE / MAGNIFIED)
        .min(image.width)
        .min(image.height);
    let side = part * MAGNIFIED;
    let from_centre = |centre: f32, size: u32| {
        let from = (f64::from(centre) - f64::from(part) / 2.0).round();
        from.clamp(0.0, f64::from(size - part)) as u32
    };
    let from = match looked_at {
        Some((x, y)) => (from_centre(x, image.width), from_centre(y, image.height)),
        None => ((image.width - part) / 2, (image.height - part) / 2),
    };
    let corner = (canvas.width - side, canvas.height - side);
    for y in 0..side {
        let row = (from.1 + y / MAGNIFIED) as usize * image.width as usize;
        let out = (corner.1 + y) as usize * canvas.width as usize + corner.0 as usize;
        for x in 0..side {
            canvas.pixels[out + x as usize] = image.pixels[row + (from.0 + x / MAGNIFIED) as usize];
        }
    }
    let framed = |canvas: &mut Canvas, at: Point, size: Size| {
        let drawn = Rectangle::new(at - Point::new(1, 1), size + Size::new(2, 2))
            .into_styled(PrimitiveStyle::with_stroke(Gray8::BLACK, 1))
            .draw(canvas)
            .and_then(|()| {
                Rectangle::new(at, size)
                    .into_styled(PrimitiveStyle::with_stroke(Gray8::WHITE, 1))
                    .draw(canvas)
            });
        drawn.unwrap_or_else(|never| match never {});
    };
    framed(
        canvas,
        Point::new(corner.0 as i32, corner.1 as i32),
        Size::new(side, side),
    );
    // Where that part is in the picture: the frame's pixel-centre coordinates, scaled.
    let scaled = |v: u32| {
        (
            (f64::from(v) * scale).round() as i32,
            (f64::from(part) * scale).round() as u32,
        )
    };
    let ((left, width), (top, height)) = (scaled(from.0), scaled(from.1));
    framed(canvas, Point::new(left, top), Size::new(width, height));
}

/// A tag's outline, black under white so it shows on any background, and its ID in its middle.
fn outline(canvas: &mut Canvas, corners: &[f32; 8], scale: f64, id: u16, large: bool) {
    // Corners are in pixel-centre coordinates: pixel 0's centre is 0, its left edge -0.5.
    let at = |x: f32, y: f32| {
        let scaled = |v: f32| (f64::from(v) + 0.5) * scale - 0.5;
        Point::new(scaled(x).round() as i32, scaled(y).round() as i32)
    };
    let points: Vec<Point> = (0..5)
        .map(|i| at(corners[2 * (i % 4)], corners[2 * (i % 4) + 1]))
        .collect();
    let line = Polyline::new(&points);
    let drawn = line
        .into_styled(PrimitiveStyle::with_stroke(Gray8::BLACK, 3))
        .draw(canvas)
        .and_then(|()| {
            line.into_styled(PrimitiveStyle::with_stroke(Gray8::WHITE, 1))
                .draw(canvas)
        });
    drawn.unwrap_or_else(|never| match never {});

    let middle = points[..4].iter().fold(Point::zero(), |sum, p| sum + *p) / 4;
    let font = if large { &FONT_9X15_BOLD } else { &FONT_6X10 };
    let centred = TextStyleBuilder::new()
        .alignment(Alignment::Center)
        .baseline(Baseline::Middle)
        .build();
    Text::with_text_style(&id.to_string(), middle, lettering(font), centred)
        .draw(canvas)
        .unwrap_or_else(|never| match never {});
}

/// White on black, readable over any part of a frame.
fn lettering(font: &'static MonoFont<'static>) -> MonoTextStyle<'static, Gray8> {
    MonoTextStyleBuilder::new()
        .font(font)
        .text_color(Gray8::WHITE)
        .background_color(Gray8::BLACK)
        .build()
}

/// A grey picture to draw on, its rows packed.
struct Canvas<'a> {
    pixels: &'a mut [u8],
    width: u32,
    height: u32,
}

impl OriginDimensions for Canvas<'_> {
    fn size(&self) -> Size {
        Size::new(self.width, self.height)
    }
}

impl DrawTarget for Canvas<'_> {
    type Color = Gray8;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Infallible>
    where
        I: IntoIterator<Item = Pixel<Gray8>>,
    {
        for Pixel(point, colour) in pixels {
            let (Ok(x), Ok(y)) = (u32::try_from(point.x), u32::try_from(point.y)) else {
                continue;
            };
            if x < self.width && y < self.height {
                self.pixels[y as usize * self.width as usize + x as usize] = colour.luma();
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::Tag;

    fn frame(tags: Vec<Tag>) -> Detected {
        let mut image = GrayImage::new(1600, 1304);
        image.pixels.fill(90);
        Detected {
            image,
            tags,
            settings: InEffect {
                exposure_us: Some(1000),
                gain: Some(40),
                auto_exposure: Some(false),
            },
            brightness: Brightness {
                median: 37,
                high: 201,
                clipped: 0,
            },
            sequence: 0,
            monotonic_nanos: 0,
        }
    }

    #[test]
    fn the_line_says_the_rate_exposure_gain_and_brightness() {
        let fixed = InEffect {
            exposure_us: Some(1000),
            gain: Some(40),
            auto_exposure: Some(false),
        };
        let lit = Brightness {
            median: 37,
            high: 201,
            clipped: 0,
        };
        let cases = [
            (
                Some(59.84),
                fixed,
                "59.8 fps  exposure 1.0 ms  gain 40  brightness 37 / 201",
            ),
            (
                None,
                fixed,
                "-- fps  exposure 1.0 ms  gain 40  brightness 37 / 201",
            ),
            (
                Some(30.0),
                InEffect {
                    exposure_us: None,
                    gain: None,
                    auto_exposure: Some(true),
                },
                "30.0 fps  auto exposure  no gain  brightness 37 / 201",
            ),
            (
                Some(30.0),
                InEffect::default(),
                "30.0 fps  exposure unknown  no gain  brightness 37 / 201",
            ),
        ];
        for (fps, settings, line) in cases {
            assert_eq!(caption(fps, settings, lit), line);
        }
    }

    #[test]
    fn a_picture_is_a_jpeg_of_the_configured_size_with_its_tags_outlined() {
        let tag = Tag {
            id: 586,
            hamming: 0,
            decision_margin: 50.0,
            corners: [700.0, 500.0, 900.0, 500.0, 900.0, 700.0, 700.0, 700.0],
            poses: None,
        };
        let mut painter = Painter::new(Stream::default());
        let jpeg = painter
            .paint(&frame(vec![tag]), Some(60.0), Shown::default(), None)
            .unwrap();
        assert_eq!(&jpeg[..2], [0xff, 0xd8], "a JPEG's start");
        let small = &painter.small;
        assert_eq!(small.len(), 800 * 652);
        // The outline's white runs along the tag's top edge, 250 px down in the picture, with the
        // black under it beside it; the grey away from it is the frame's, and its ID is lettering
        // on black in its middle, at (400, 300).
        let at = |x: usize, y: usize| small[y * 800 + x];
        assert_eq!(at(400, 250), 255);
        assert_eq!(at(400, 251), 0);
        assert_eq!(at(370, 270), 90);
        let middle = &small[300 * 800 + 380..300 * 800 + 420];
        assert!(middle.contains(&0) && middle.contains(&255));
        // The line along the top is lettering on black.
        let line = &small[2 * 800..15 * 800];
        assert!(line.contains(&0) && line.contains(&255));
    }

    #[test]
    fn a_tag_near_the_edge_is_drawn_as_far_as_the_picture_goes() {
        let tag = Tag {
            id: 1,
            hamming: 0,
            decision_margin: 50.0,
            corners: [-40.0, 1250.0, 60.0, 1250.0, 60.0, 1350.0, -40.0, 1350.0],
            poses: None,
        };
        let mut painter = Painter::new(Stream {
            scale: 0.25,
            fps: 10,
            quality: 50,
        });
        let jpeg = painter
            .paint(&frame(vec![tag]), None, Shown::default(), None)
            .unwrap();
        assert!(jpeg.len() > 100);
        assert_eq!(painter.small.len(), 400 * 326);
    }

    #[test]
    fn the_focus_line_says_the_edge_s_width_and_the_best_and_the_bar_fills_toward_the_best() {
        // Each row: the width now, the best, the line, and how full the bar is.
        let cases = [
            (None, None, "focus: no tag to measure", None),
            (
                None,
                Some(1.94),
                "focus: no tag to measure, 1.9 at best",
                None,
            ),
            (Some(2.44), None, "focus: a tag's edge is 2.4 px wide", None),
            (
                Some(3.0),
                Some(1.5),
                "focus: a tag's edge is 3.0 px wide, 1.5 at best",
                Some(0.5),
            ),
            (
                Some(1.5),
                Some(1.5),
                "focus: a tag's edge is 1.5 px wide, 1.5 at best",
                Some(1.0),
            ),
        ];
        for (now_px, best_px, line, share) in cases {
            let focus = Shown { now_px, best_px };
            assert_eq!(focus_line(focus), line);
            assert_eq!(share_of_best(focus), share);
        }
    }

    #[test]
    fn a_part_of_the_frame_is_drawn_magnified_in_the_corner_pixel_for_pixel() {
        // A frame whose middle has a sharp step down its centre line: black left of x = 800,
        // white from it.
        let mut shown = frame(vec![]);
        for y in 0..1304 {
            for x in 0..1600 {
                shown.image.pixels[y * 1600 + x] = if x < 800 { 20 } else { 220 };
            }
        }
        let mut painter = Painter::new(Stream::default());
        painter
            .paint(&shown, Some(60.0), Shown::default(), None)
            .unwrap();
        let small = &painter.small;
        let at = |x: usize, y: usize| small[y * 800 + x];
        // The picture is 800 x 652: the magnified part is 72 of the frame's pixels drawn 216
        // across, in the bottom-right corner, so its own middle is at x = 800 - 108.
        let (left, top) = (800 - 216, 652 - 216);
        assert_eq!(
            at(left + 106, top + 100),
            20,
            "the frame's pixel 799, drawn three across"
        );
        assert_eq!(at(left + 107, top + 100), 20);
        assert_eq!(
            at(left + 108, top + 100),
            220,
            "the frame's pixel 800: no ramp between"
        );
        assert_eq!(at(left + 110, top + 100), 220);
        // Its outline, and the box in the picture's middle that marks the part magnified: 36 px
        // across from (382, 308).
        assert_eq!(at(left, top + 100), 255);
        assert_eq!(at(382, 320), 255);
        assert_eq!(at(382 + 35, 320), 255);
        assert_eq!(at(382 + 10, 308), 255);

        // With a tag measured, the part is centred on the tag's first corner instead: here at
        // (800, 300), on the step, so the step is still down the part's middle. And it stays there
        // once no tag is measured, until the painter is told to look afresh.
        shown.tags = vec![Tag {
            id: 3,
            hamming: 0,
            decision_margin: 50.0,
            corners: [800.0, 300.0, 1100.0, 300.0, 1100.0, 600.0, 800.0, 600.0],
            poses: None,
        }];
        let marked = |painter: &Painter, x: usize, y: usize| painter.small[y * 800 + x] == 255;
        painter
            .paint(&shown, Some(60.0), Shown::default(), Some(0))
            .unwrap();
        // The frame's 72 px from (764, 264): in the picture, a box 36 px across from (382, 132).
        assert!(marked(&painter, 382, 140) && marked(&painter, 382 + 35, 140));
        assert!(!marked(&painter, 382, 320), "the box has left the middle");
        assert_eq!(painter.small[(top + 100) * 800 + left + 107], 20);
        assert_eq!(painter.small[(top + 100) * 800 + left + 108], 220);
        shown.tags.clear();
        painter
            .paint(&shown, Some(60.0), Shown::default(), None)
            .unwrap();
        assert!(marked(&painter, 382, 140), "still where the tag was");
        painter.look_afresh();
        painter
            .paint(&shown, Some(60.0), Shown::default(), None)
            .unwrap();
        assert!(marked(&painter, 382, 320) && !marked(&painter, 382, 140));
        // A tag in the frame's corner: the part stops at the frame's edges.
        painter
            .paint(&shown, Some(60.0), Shown::default(), None)
            .unwrap();
        shown.tags = vec![Tag {
            id: 3,
            hamming: 0,
            decision_margin: 50.0,
            corners: [1590.0, 5.0, 1700.0, 5.0, 1700.0, 100.0, 1590.0, 100.0],
            poses: None,
        }];
        painter
            .paint(&shown, Some(60.0), Shown::default(), Some(0))
            .unwrap();
        assert!(
            marked(&painter, 764 + 35, 10),
            "the frame's last 72 px across, from its top"
        );

        // A small picture has no room for it: its corner is the frame's own.
        let mut painter = Painter::new(Stream {
            scale: 0.1,
            fps: 10,
            quality: 50,
        });
        painter.paint(&shown, None, Shown::default(), None).unwrap();
        assert_eq!(painter.small.len(), 160 * 130);
        assert_eq!(painter.small[129 * 160 + 159], 220);
        assert_eq!(painter.small[100 * 160 + 120], 220);
    }
}
