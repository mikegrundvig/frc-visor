//! The magnified corner the page shows beside the focus readout: a small square of the camera's
//! frame round a corner of the tag the focus is read from, each of the frame's pixels drawn as a
//! square [`MAGNIFIED`] across with nothing smoothed, so the eye sees what the number measures: a
//! clean step when in focus, a ramp when not. It's its own small JPEG, `GET /corner/<camera>`,
//! rather than painted over the picture, which other programs show too.

use jpeg_encoder::{ColorType, Encoder};

use crate::image::GrayImage;

/// The part of the frame shown, in the frame's pixels across and down.
pub const PART_PX: u32 = 72;

/// How many times larger each of the frame's pixels is drawn.
pub const MAGNIFIED: u32 = 3;

/// JPEG quality: high, as the point is the edge's few pixels.
const QUALITY: u8 = 90;

/// Where the corner looks: a corner of the tag the focus was last read from. It stays there once
/// the tag is lost, as it is when the lens is turned far from focus, so the eye can bring it back;
/// and it's the frame's middle until a tag has been measured.
#[derive(Debug, Default)]
pub struct Corner {
    looked_at: Option<(f32, f32)>,
    pixels: Vec<u8>,
}

impl Corner {
    /// Looks at the frame's middle again, until a tag's focus is next read: for a new watcher.
    pub fn look_afresh(&mut self) {
        self.looked_at = None;
    }

    /// The magnified part of `image` as a JPEG, looking at `tag_corners`' first corner (the top-left
    /// of the printed tag's black square) when a tag was measured. None for a frame smaller than
    /// the part.
    pub fn make(
        &mut self,
        image: &GrayImage,
        tag_corners: Option<&[f32; 8]>,
    ) -> Option<Result<Vec<u8>, String>> {
        if let Some(corners) = tag_corners {
            self.looked_at = Some((corners[0], corners[1]));
        }
        if image.width < PART_PX || image.height < PART_PX {
            return None;
        }
        let from = self.from(image);
        let side = PART_PX * MAGNIFIED;
        self.pixels.resize((side * side) as usize, 0);
        for y in 0..side {
            let row = (from.1 + y / MAGNIFIED) as usize * image.width as usize;
            let out = (y * side) as usize;
            for x in 0..side {
                self.pixels[out + x as usize] =
                    image.pixels[row + (from.0 + x / MAGNIFIED) as usize];
            }
        }
        let mut jpeg = Vec::new();
        Some(
            Encoder::new(&mut jpeg, QUALITY)
                .encode(&self.pixels, side as u16, side as u16, ColorType::Luma)
                .map(|()| jpeg)
                .map_err(|e| format!("the corner: {e}")),
        )
    }

    /// The part's top-left in the frame: centred on where it looks, held inside the frame.
    fn from(&self, image: &GrayImage) -> (u32, u32) {
        let from_centre = |centre: f32, size: u32| {
            let from = (f64::from(centre) - f64::from(PART_PX) / 2.0).round();
            from.clamp(0.0, f64::from(size - PART_PX)) as u32
        };
        match self.looked_at {
            Some((x, y)) => (from_centre(x, image.width), from_centre(y, image.height)),
            None => ((image.width - PART_PX) / 2, (image.height - PART_PX) / 2),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_corner_is_the_frame_s_own_pixels_round_the_tag_held_inside_the_frame() {
        // A frame with a sharp step down x = 800: black left of it, white from it.
        let mut image = GrayImage::new(1600, 1304);
        for y in 0..1304 {
            for x in 0..1600 {
                image.pixels[y * 1600 + x] = if x < 800 { 20 } else { 220 };
            }
        }
        let tag_at = |x: f32, y: f32| [x, y, x + 300.0, y, x + 300.0, y + 300.0, x, y + 300.0];
        let mut corner = Corner::default();
        // Each row: the tag's first corner (none: no tag measured), and the part's top-left.
        let rows = [
            (None, (764, 616)),
            (Some(tag_at(800.0, 300.0)), (764, 264)),
            // The tag lost: still where it was.
            (None, (764, 264)),
            (Some(tag_at(1590.0, 5.0)), (1528, 0)),
        ];
        for (tag, from) in rows {
            let jpeg = corner.make(&image, tag.as_ref()).unwrap().unwrap();
            assert_eq!(&jpeg[..2], [0xff, 0xd8], "a JPEG");
            assert_eq!(corner.from(&image), from, "{tag:?}");
        }
        // Drawn pixel for pixel: looking at the step, the part's middle column is its edge, no ramp.
        corner
            .make(&image, Some(&tag_at(800.0, 300.0)))
            .unwrap()
            .unwrap();
        let side = (PART_PX * MAGNIFIED) as usize;
        let at = |x: usize| corner.pixels[100 * side + x];
        assert_eq!((at(105), at(107), at(108), at(110)), (20, 20, 220, 220));
        // A new watcher looks at the middle again; a frame smaller than the part has none.
        corner.look_afresh();
        corner.make(&image, None).unwrap().unwrap();
        assert_eq!(corner.from(&image), (764, 616));
        assert!(corner.make(&GrayImage::new(40, 40), None).is_none());
    }
}
