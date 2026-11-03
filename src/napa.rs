//! NAPA, the AprilTag detector, through its C API (vendor/napa/napa_core.h). NAPA is by Zachary
//! Wilson, under GPL-3.0-only; see vendor/napa/NOTICE.

use std::ffi::{CStr, c_int, c_void};
use std::fmt;
use std::ptr::NonNull;

use crate::image::GrayImage;

/// The tag family FRC uses.
const FAMILY: &CStr = c"tag36h11";
/// Room for more tags than a frame will ever hold.
const MAX_DETECTIONS: usize = 64;

/// One tag NAPA found.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Detection {
    pub id: c_int,
    /// Payload bits corrected.
    pub hamming: c_int,
    /// The lesser of the white and the black payload bits' mean distance from the threshold.
    pub decision_margin: f32,
    /// x0, y0 … x3, y3 in OpenCV's corner order, in pixel-centre coordinates.
    pub corners: [f64; 8],
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct Params {
    min_tag_px: c_int,
    max_hamming: c_int,
    min_contrast: c_int,
    min_border_cells: c_int,
    sharpening_x100: c_int,
}

#[repr(C)]
struct FamilyLayout {
    ncodes: c_int,
    nbits: c_int,
    width: c_int,
    total_width: c_int,
    reversed: c_int,
    min_distance: c_int,
    max_hamming: c_int,
    recommended_hamming: c_int,
    x: [i8; 64],
    y: [i8; 64],
}

unsafe extern "C" {
    fn napa_create_family(family: *const std::ffi::c_char) -> *mut c_void;
    fn napa_destroy(detector: *mut c_void);
    fn napa_family_info(family: *const std::ffi::c_char, info: *mut FamilyLayout) -> c_int;
    fn napa_detect(
        detector: *mut c_void,
        image: *const u8,
        width: c_int,
        height: c_int,
        stride: c_int,
        params: *const Params,
        out: *mut Detection,
        max_out: c_int,
    ) -> c_int;
}

/// Why NAPA couldn't run.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NapaError {
    MinTagPx(i32),
    OutOfMemory,
    InvalidInput,
}

impl fmt::Display for NapaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NapaError::MinTagPx(px) => write!(f, "min_tag_px is from 8 to 30000, not {px}"),
            NapaError::OutOfMemory => write!(f, "NAPA ran out of memory"),
            NapaError::InvalidInput => write!(f, "NAPA refused the image or its settings"),
        }
    }
}

impl std::error::Error for NapaError {}

/// A tag36h11 detector. Not thread-safe, as NAPA's handle isn't: one per camera.
pub struct Detector {
    handle: NonNull<c_void>,
    params: Params,
    found: Vec<Detection>,
}

// The handle is only ever used through `&mut self`, so moving it to another thread is safe.
unsafe impl Send for Detector {}

impl Detector {
    /// A detector that finds tags whose black square is at least `min_tag_px` pixels across. The
    /// other settings are NAPA's defaults.
    pub fn new(min_tag_px: i32) -> Result<Detector, NapaError> {
        if !(8..=30_000).contains(&min_tag_px) {
            return Err(NapaError::MinTagPx(min_tag_px));
        }
        let mut layout: FamilyLayout = unsafe { std::mem::zeroed() };
        if unsafe { napa_family_info(FAMILY.as_ptr(), &mut layout) } != 0 {
            unreachable!("NAPA always has tag36h11");
        }
        let handle = NonNull::new(unsafe { napa_create_family(FAMILY.as_ptr()) })
            .ok_or(NapaError::OutOfMemory)?;
        Ok(Detector {
            handle,
            params: Params {
                min_tag_px,
                max_hamming: layout.recommended_hamming,
                min_contrast: 20,
                min_border_cells: 22,
                sharpening_x100: 25,
            },
            found: vec![Detection::default(); MAX_DETECTIONS],
        })
    }

    /// The tags in an image. The slice is reused by the next call.
    pub fn detect(&mut self, image: &GrayImage) -> Result<&[Detection], NapaError> {
        let count = unsafe {
            napa_detect(
                self.handle.as_ptr(),
                image.pixels.as_ptr(),
                image.width as c_int,
                image.height as c_int,
                image.width as c_int,
                &self.params,
                self.found.as_mut_ptr(),
                MAX_DETECTIONS as c_int,
            )
        };
        match count {
            -2 => Err(NapaError::OutOfMemory),
            n if n < 0 => Err(NapaError::InvalidInput),
            n => Ok(&self.found[..n as usize]),
        }
    }
}

impl Drop for Detector {
    fn drop(&mut self) {
        unsafe { napa_destroy(self.handle.as_ptr()) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_c_structs_match_napa_s() {
        // From napa_core.h: three ints and a float, then eight doubles; and five ints.
        assert_eq!(std::mem::size_of::<Detection>(), 80);
        assert_eq!(std::mem::size_of::<Params>(), 20);
        assert_eq!(std::mem::size_of::<FamilyLayout>(), 8 * 4 + 128);
    }

    #[test]
    fn settings_out_of_range_are_refused() {
        for px in [0, 7, 30_001] {
            assert_eq!(Detector::new(px).err(), Some(NapaError::MinTagPx(px)));
        }
        assert!(Detector::new(16).is_ok());
    }

    #[test]
    fn a_blank_image_has_no_tags() {
        let mut detector = Detector::new(16).unwrap();
        let image = GrayImage::new(640, 480);
        assert!(detector.detect(&image).unwrap().is_empty());
    }
}
