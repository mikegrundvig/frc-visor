//! How bright a frame is, from a sample of its pixels: what every frame tells the robot, so a log
//! shows whether the exposure and gain suit the light, and what Visor warns of when they don't.

use crate::image::GrayImage;

/// Every this many pixels across and down are sampled: 1 in 16, about 130,000 of a 1600 x 1304
/// frame, a fraction of a millisecond.
const SAMPLE_EVERY: usize = 4;

/// A pixel this bright or brighter counts as clipped: at or near full white, where a brighter part
/// of the scene reads the same and a tag's white loses its edge.
pub const CLIPPED_AT: u8 = 250;

/// A frame whose brightest 1% is below this is dark: a tag's black-to-white step is then a few
/// dozen levels at most, which the sensor's noise eats into at range. A quarter of full scale.
pub const DARK_BELOW: u8 = 64;

/// A frame with more than this share of its pixels clipped, in hundredths of a percent (1%), has
/// whites clipping: more than a near tag's white, or a bright light in view.
pub const CLIPPED_ABOVE: u16 = 100;

/// A frame's brightness, 0 (black) to 255 (white).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Brightness {
    /// The middle pixel's.
    pub median: u8,
    /// The 99th percentile's: about as bright as a tag's white in view, the brightest of most
    /// scenes.
    pub high: u8,
    /// The share of pixels at [`CLIPPED_AT`] or brighter, in hundredths of a percent (0 to 10,000).
    pub clipped: u16,
}

impl Brightness {
    /// A frame's, from a sample of its pixels.
    pub fn of(image: &GrayImage) -> Brightness {
        let width = image.width as usize;
        let mut counts = [0u32; 256];
        let mut sampled = 0u32;
        for row in image
            .pixels
            .chunks_exact(width.max(1))
            .step_by(SAMPLE_EVERY)
        {
            for &pixel in row.iter().step_by(SAMPLE_EVERY) {
                counts[pixel as usize] += 1;
                sampled += 1;
            }
        }
        if sampled == 0 {
            return Brightness::default();
        }
        let clipped: u32 = counts[CLIPPED_AT as usize..].iter().sum();
        Brightness {
            median: percentile(&counts, sampled, 50),
            high: percentile(&counts, sampled, 99),
            clipped: (u64::from(clipped) * 10_000 / u64::from(sampled)) as u16,
        }
    }

    /// Whether it's too dark for tags to be read far ([`DARK_BELOW`]).
    pub fn is_dark(self) -> bool {
        self.high < DARK_BELOW
    }

    /// Whether its whites clip ([`CLIPPED_ABOVE`]).
    pub fn clips(self) -> bool {
        self.clipped > CLIPPED_ABOVE
    }
}

/// The brightness `percent` of the way up the sampled pixels.
fn percentile(counts: &[u32; 256], sampled: u32, percent: u32) -> u8 {
    let wanted = (u64::from(sampled) * u64::from(percent))
        .div_ceil(100)
        .max(1);
    let mut seen = 0u64;
    for (level, &count) in counts.iter().enumerate() {
        seen += u64::from(count);
        if seen >= wanted {
            return level as u8;
        }
    }
    255
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame of one brightness, with a share of its rows another.
    fn frame(background: u8, rows_of: Option<(u8, u32)>) -> GrayImage {
        let mut image = GrayImage::new(160, 100);
        image.pixels.fill(background);
        if let Some((level, rows)) = rows_of {
            let start = (100 - rows) as usize * 160;
            image.pixels[start..].fill(level);
        }
        image
    }

    #[test]
    fn a_frame_s_brightness_is_its_median_high_end_and_clipped_share() {
        // What each frame is, then its brightness, and whether it's dark or clips.
        let cases = [
            (
                "a dim room",
                frame(20, Some((40, 8))),
                (20, 40, 0),
                (true, false),
            ),
            (
                "lit, a tag's white in view",
                frame(60, Some((180, 8))),
                (60, 180, 0),
                (false, false),
            ),
            (
                "over-exposed: a fifth at full white",
                frame(120, Some((255, 20))),
                (120, 255, 2000),
                (false, true),
            ),
            (
                "a light in a dark frame",
                frame(10, Some((255, 4))),
                (10, 255, 400),
                (false, true),
            ),
            ("black", frame(0, None), (0, 0, 0), (true, false)),
        ];
        for (name, image, (median, high, clipped), (dark, clips)) in cases {
            let measured = Brightness::of(&image);
            assert_eq!(
                (measured.median, measured.high, measured.clipped),
                (median, high, clipped),
                "{name}"
            );
            assert_eq!(
                (measured.is_dark(), measured.clips()),
                (dark, clips),
                "{name}"
            );
        }
    }

    #[test]
    fn an_empty_frame_is_black() {
        assert_eq!(Brightness::of(&GrayImage::new(0, 0)), Brightness::default());
    }
}
