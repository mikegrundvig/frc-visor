//! The PGM file `visor record` saves, as the lens calibration tool reads it: Visor's writer must
//! still make testdata/visor-written.pgm byte for byte, and tools/test_calibrate.py checks that
//! tools/calibrate.py reads that file with these same pixels (VISOR_WRITE_GOLDEN=1 writes it
//! afresh).

use std::path::Path;

use visor::image::GrayImage;

/// An odd width, so a writer that pads its rows shows; every value from 0 to 255; and the first
/// pixel a newline byte, so a reader that skips more than the one whitespace byte after the header
/// shows too.
fn pattern() -> GrayImage {
    let (width, height) = (37u32, 23u32);
    let mut image = GrayImage::new(width, height);
    for y in 0..height {
        for x in 0..width {
            let value = (y * width + x + 10) % 256;
            image.pixels[(y * width + x) as usize] = value as u8;
        }
    }
    image
}

#[test]
fn the_written_file_matches_the_golden_file() {
    let image = pattern();
    let written = Path::new(env!("CARGO_TARGET_TMPDIR")).join("visor-written.pgm");
    image.write_pgm(&written).unwrap();
    let bytes = std::fs::read(&written).unwrap();

    let golden = Path::new(env!("CARGO_MANIFEST_DIR")).join("testdata/visor-written.pgm");
    if std::env::var_os("VISOR_WRITE_GOLDEN").is_some() {
        std::fs::write(&golden, &bytes).unwrap();
    }
    let file = std::fs::read(&golden).expect("testdata/visor-written.pgm");
    assert_eq!(
        bytes, file,
        "the PGM Visor writes changed: check tools/calibrate.py still reads it"
    );
    assert_eq!(GrayImage::read_pgm(&golden).unwrap(), image);
}
