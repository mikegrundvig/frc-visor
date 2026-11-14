//! A live view's picture: the camera's frame made smaller, each tag outlined with its ID, and a line
//! along the top with the detection's frame rate, the exposure and gain, the brightness and the
//! focus, as a JPEG. Resizing is fast_image_resize's, drawing embedded-graphics', and the JPEG jpeg-encoder's.

use std::convert::Infallible;

use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_7X13, FONT_9X15_BOLD};
use embedded_graphics::mono_font::{MonoFont, MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Gray8;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Polyline, PrimitiveStyle};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};
use fast_image_resize::images::{Image, ImageRef};
use fast_image_resize::{FilterType, PixelType, ResizeAlg, ResizeOptions, Resizer};
use jpeg_encoder::{ColorType, Encoder};

use crate::brightness::Brightness;
use crate::config::Stream;
use crate::image::GrayImage;
use crate::settings::InEffect;
use crate::stream::feed::Detected;

/// Pictures this wide or wider get the larger lettering.
const LARGE_FROM: u32 = 640;

/// Makes a camera's pictures, keeping its buffers from one to the next.
pub struct Painter {
    stream: Stream,
    resizer: Resizer,
    small: Vec<u8>,
}

impl Painter {
    pub fn new(stream: Stream) -> Painter {
        Painter {
            stream,
            resizer: Resizer::new(),
            small: Vec::new(),
        }
    }

    /// A frame's picture, as a JPEG, with its line saying `fps` (none while it's being measured)
    /// and the frame's `focus` ([`focus`]).
    pub fn paint(
        &mut self,
        frame: &Detected,
        fps: Option<f64>,
        focus: f64,
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
        let line = caption(fps, frame.settings, frame.brightness, focus);
        let font = if large { &FONT_7X13 } else { &FONT_6X10 };
        Text::with_baseline(&line, Point::new(2, 2), lettering(font), Baseline::Top)
            .draw(&mut canvas)
            .unwrap_or_else(|never| match never {});

        let mut jpeg = Vec::with_capacity(self.small.len() / 8);
        Encoder::new(&mut jpeg, self.stream.quality)
            .encode(&self.small, width as u16, height as u16, ColorType::Luma)
            .map_err(|e| format!("encoding: {e}"))?;
        Ok(jpeg)
    }
}

/// How sharp a frame is: the mean square of the step from a pixel to the next one across and the
/// next one down, taken at every fourth pixel of every fourth row. An edge in focus is a steep step,
/// so the number peaks as a lens comes into focus: turn the lens and stop at the peak. Noise is
/// steps too, and so is what's in view, so it compares two frames of one scene at one exposure and
/// gain, and nothing else.
pub fn focus(image: &GrayImage) -> f64 {
    let (width, height) = (image.width as usize, image.height as usize);
    let (mut sum, mut steps) = (0u64, 0u64);
    for y in (0..height.saturating_sub(1)).step_by(4) {
        let row = &image.pixels[y * width..];
        for x in (0..width.saturating_sub(1)).step_by(4) {
            let here = i32::from(row[x]);
            let across = i32::from(row[x + 1]) - here;
            let down = i32::from(row[x + width]) - here;
            sum += (across * across + down * down) as u64;
            steps += 2;
        }
    }
    if steps == 0 {
        return 0.0;
    }
    sum as f64 / steps as f64
}

/// The line along the top: `59.8 fps  exposure 1.0 ms  gain 40  brightness 37 / 201  focus 12.3`.
pub fn caption(fps: Option<f64>, settings: InEffect, brightness: Brightness, focus: f64) -> String {
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
        "{rate}  {exposure}  {gain}  brightness {} / {}  focus {focus:.1}",
        brightness.median, brightness.high
    )
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
                "59.8 fps  exposure 1.0 ms  gain 40  brightness 37 / 201  focus 12.3",
            ),
            (
                None,
                fixed,
                "-- fps  exposure 1.0 ms  gain 40  brightness 37 / 201  focus 12.3",
            ),
            (
                Some(30.0),
                InEffect {
                    exposure_us: None,
                    gain: None,
                    auto_exposure: Some(true),
                },
                "30.0 fps  auto exposure  no gain  brightness 37 / 201  focus 12.3",
            ),
            (
                Some(30.0),
                InEffect::default(),
                "30.0 fps  exposure unknown  no gain  brightness 37 / 201  focus 12.3",
            ),
        ];
        for (fps, settings, line) in cases {
            assert_eq!(caption(fps, settings, lit, 12.34), line);
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
        let jpeg = painter.paint(&frame(vec![tag]), Some(60.0), 0.0).unwrap();
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
        let jpeg = painter.paint(&frame(vec![tag]), None, 0.0).unwrap();
        assert!(jpeg.len() > 100);
        assert_eq!(painter.small.len(), 400 * 326);
    }

    #[test]
    fn focus_is_higher_for_sharp_edges_than_soft_ones_and_nothing_for_a_flat_frame() {
        // Stripes five pixels wide, which the every-fourth-pixel sampling doesn't fall in step
        // with: sharp, each edge one step of 200; soft, the same rise spread over the five.
        let striped = |soft: bool| {
            let mut image = GrayImage::new(64, 32);
            for (i, pixel) in image.pixels.iter_mut().enumerate() {
                let x = (i % 64 % 10) as u8;
                *pixel = match (soft, x < 5) {
                    (false, true) => 220,
                    (false, false) => 20,
                    (true, true) => 20 + 40 * x,
                    (true, false) => 220 - 40 * (x - 5),
                };
            }
            image
        };
        let (sharp, soft) = (focus(&striped(false)), focus(&striped(true)));
        assert!(sharp > 2.0 * soft, "sharp {sharp}, soft {soft}");
        assert!(soft > 0.0);
        let mut flat = GrayImage::new(64, 32);
        flat.pixels.fill(90);
        assert_eq!(focus(&flat), 0.0);
        // Too small to have a neighbour: nothing, not a panic.
        assert_eq!(focus(&GrayImage::new(1, 1)), 0.0);
        assert_eq!(focus(&GrayImage::new(0, 0)), 0.0);
    }
}
