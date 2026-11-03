//! 8-bit grayscale images: what NAPA reads, what cameras are converted to, and what `visor record`
//! saves as PGM files for lens calibration.

use std::fs;
use std::io;
use std::path::Path;

/// A grayscale image, its rows packed with no padding.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GrayImage {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

impl GrayImage {
    pub fn new(width: u32, height: u32) -> GrayImage {
        GrayImage {
            width,
            height,
            pixels: vec![0; width as usize * height as usize],
        }
    }

    /// Copies an 8-bit mono (V4L2 `GREY`) frame whose rows are `stride` bytes apart.
    pub fn copy_grey(&mut self, frame: &[u8], stride: usize) -> io::Result<()> {
        let width = self.width as usize;
        check_size(frame, stride, width, self.height as usize)?;
        for (row, out) in self.pixels.chunks_exact_mut(width).enumerate() {
            out.copy_from_slice(&frame[row * stride..row * stride + width]);
        }
        Ok(())
    }

    /// Takes the luma from a `YUYV` frame (Y0 U Y1 V for each two pixels), rows `stride` bytes apart.
    pub fn copy_yuyv_luma(&mut self, frame: &[u8], stride: usize) -> io::Result<()> {
        let width = self.width as usize;
        check_size(frame, stride, 2 * width, self.height as usize)?;
        for (row, out) in self.pixels.chunks_exact_mut(width).enumerate() {
            let line = &frame[row * stride..row * stride + 2 * width];
            let (pairs, _) = line.as_chunks::<2>();
            for (pixel, [luma, _]) in out.iter_mut().zip(pairs) {
                *pixel = *luma;
            }
        }
        Ok(())
    }

    /// Reads a binary PGM (`P5`, 8-bit).
    pub fn read_pgm(path: &Path) -> io::Result<GrayImage> {
        let bytes = fs::read(path)?;
        parse_pgm(&bytes).map_err(|why| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("{}: {why}", path.display()),
            )
        })
    }

    /// Writes a binary PGM (`P5`, 8-bit).
    pub fn write_pgm(&self, path: &Path) -> io::Result<()> {
        let mut bytes = format!("P5\n{} {}\n255\n", self.width, self.height).into_bytes();
        bytes.extend_from_slice(&self.pixels);
        fs::write(path, bytes)
    }
}

fn check_size(frame: &[u8], stride: usize, row_bytes: usize, height: usize) -> io::Result<()> {
    if stride < row_bytes || frame.len() < stride * (height - 1) + row_bytes {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "a {} byte frame is too small for {height} rows of {row_bytes} bytes, {stride} apart",
                frame.len()
            ),
        ));
    }
    Ok(())
}

fn parse_pgm(bytes: &[u8]) -> Result<GrayImage, String> {
    // The header is "P5", width, height and maxval, separated by whitespace, with # comments, then
    // one whitespace byte, then the pixels.
    let mut fields = Vec::new();
    let mut at = 0;
    while fields.len() < 4 {
        while at < bytes.len() && (bytes[at].is_ascii_whitespace() || bytes[at] == b'#') {
            if bytes[at] == b'#' {
                while at < bytes.len() && bytes[at] != b'\n' {
                    at += 1;
                }
            } else {
                at += 1;
            }
        }
        let start = at;
        while at < bytes.len() && !bytes[at].is_ascii_whitespace() {
            at += 1;
        }
        if start == at {
            return Err("the header is cut short".into());
        }
        fields.push(String::from_utf8_lossy(&bytes[start..at]).into_owned());
    }
    if fields[0] != "P5" {
        return Err(format!("not a binary PGM (starts {:?})", fields[0]));
    }
    let number = |text: &str| {
        text.parse::<u32>()
            .map_err(|_| format!("not a number: {text:?}"))
    };
    let (width, height, maxval) = (
        number(&fields[1])?,
        number(&fields[2])?,
        number(&fields[3])?,
    );
    if maxval != 255 {
        return Err(format!("only 8-bit PGMs are read (maxval {maxval})"));
    }
    let pixels = &bytes[(at + 1).min(bytes.len())..];
    let size = width as usize * height as usize;
    if pixels.len() != size {
        return Err(format!(
            "{width}x{height} needs {size} pixels, the file has {}",
            pixels.len()
        ));
    }
    Ok(GrayImage {
        width,
        height,
        pixels: pixels.to_vec(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames_copy_with_their_row_padding_dropped() {
        // A 3x2 image in rows 4 bytes apart (one byte of padding), as GREY and as YUYV.
        let grey = [1, 2, 3, 0, 4, 5, 6, 0];
        let yuyv = [1, 9, 2, 9, 3, 9, 0, 0, 4, 9, 5, 9, 6, 9, 0, 0];
        let mut image = GrayImage::new(3, 2);
        image.copy_grey(&grey, 4).unwrap();
        assert_eq!(image.pixels, [1, 2, 3, 4, 5, 6]);
        let mut image = GrayImage::new(3, 2);
        image.copy_yuyv_luma(&yuyv, 8).unwrap();
        assert_eq!(image.pixels, [1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn a_short_frame_is_refused() {
        let mut image = GrayImage::new(3, 2);
        assert!(image.copy_grey(&[0; 6], 4).is_err());
        assert!(image.copy_grey(&[0; 5], 3).is_err());
        assert!(image.copy_yuyv_luma(&[0; 11], 6).is_err());
    }

    #[test]
    fn pgm_files_round_trip_and_bad_ones_are_named() {
        let dir = std::env::temp_dir().join(format!("visor-pgm-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("a.pgm");
        let image = GrayImage {
            width: 3,
            height: 2,
            pixels: vec![0, 10, 20, 30, 40, 255],
        };
        image.write_pgm(&path).unwrap();
        assert_eq!(GrayImage::read_pgm(&path).unwrap(), image);

        let cases: [(&[u8], &str); 4] = [
            (b"P2\n3 2\n255\n", "not a binary PGM"),
            (b"P5\n3 2\n65535\n", "only 8-bit"),
            (b"P5\n3 2\n255\n\x01\x02", "needs 6 pixels"),
            (b"P5\n# a comment\n3", "cut short"),
        ];
        for (bytes, why) in cases {
            let error = parse_pgm(bytes).unwrap_err();
            assert!(error.contains(why), "{error}");
        }
        // A comment in the header is skipped.
        let commented = parse_pgm(b"P5\n# made by a camera\n1 1\n255\n\x07").unwrap();
        assert_eq!(commented.pixels, [7]);
        fs::remove_file(&path).unwrap();
        fs::remove_dir(&dir).unwrap();
    }
}
