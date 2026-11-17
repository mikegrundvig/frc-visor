//! The live view's focus number on rendered tags: the width of a tag's edge follows the lens's
//! blur, and little else. The frames are made by the detection sweep's renderer, from fixed seeds,
//! and the tags are found by NAPA, so the width is read from the corners Visor would have.
//! Each row's readings are printed (`cargo test --test focus -- --nocapture`).

// The renderer has more than this test uses.
#[allow(dead_code)]
#[path = "detection_limits/render.rs"]
mod render;

use nalgebra::{Matrix3, Rotation3, Vector3};
use render::{Camera, Optics, Payload, Placed};
use visor::napa::Detector;
use visor::stream::focus;

const CAMERA: Camera = Camera {
    width: 640,
    height: 512,
};

/// One row: how the tag is seen.
#[derive(Debug, Clone, Copy)]
struct View {
    distance_m: f64,
    yaw_deg: f64,
    /// A Gaussian blur's standard deviation, in pixels.
    blur_px: f64,
    exposure: f64,
}

/// (ID, seed, offset of the tag's centre from the frame's, in pixels.)
const TRIALS: [(i32, u64, (f64, f64)); 4] = [
    (1, 1, (-10.0, -8.0)),
    (7, 2, (6.3, 3.2)),
    (300, 3, (-3.6, 7.5)),
    (586, 4, (9.9, -4.7)),
];

fn readings(view: View) -> Vec<Option<f64>> {
    let mut detector = Detector::new(16).unwrap();
    TRIALS
        .iter()
        .map(|&(id, seed, offset)| {
            let turned = Rotation3::new(Vector3::new(0.05, view.yaw_deg.to_radians(), 0.1));
            let face_out = Rotation3::from_matrix_unchecked(Matrix3::from_diagonal(&Vector3::new(
                1.0, -1.0, -1.0,
            )));
            let (cx, cy) = (
                (f64::from(CAMERA.width) - 1.0) / 2.0,
                (f64::from(CAMERA.height) - 1.0) / 2.0,
            );
            let tag = Placed {
                payload: Payload::Bits(render::tag36h11_code(id)),
                rotation: turned * face_out,
                translation_m: CAMERA.point_at(cx + offset.0, cy + offset.1, view.distance_m),
                black: 25.0,
                // Below the white a reading is trusted in (light::TOO_BRIGHT_ABOVE), noise and all.
                white: 160.0,
            };
            let optics = Optics {
                blur_px: view.blur_px,
                exposure: view.exposure,
            };
            let image = CAMERA.render(&[tag], optics, seed);
            let found = detector.detect(&image).unwrap();
            let detection = found.iter().find(|d| d.id == id)?;
            focus::edge_px(&image, &detection.corners.map(|c| c as f32))
        })
        .collect()
}

#[test]
fn a_tag_s_edge_width_follows_the_blur_and_little_else() {
    const SHARP: View = View {
        distance_m: 2.0,
        yaw_deg: 0.0,
        blur_px: 0.0,
        exposure: 1.0,
    };
    // Each row: the view, then the least and most any trial's tag may read, in pixels. The bounds
    // are what today's measure reads (in each comment, the trials' range) with a margin.
    let rows = [
        // Blur, at 2 m, where the square is 100 px across. In focus an edge reads a pixel and a
        // half: the sensor's own pixels. Each pixel of blur adds about two and a half.
        (SHARP, 1.35, 1.75), // 1.54 to 1.56
        (
            View {
                blur_px: 0.5,
                ..SHARP
            },
            1.6,
            2.05,
        ), // 1.79 to 1.83
        (
            View {
                blur_px: 1.0,
                ..SHARP
            },
            2.65,
            3.1,
        ), // 2.85 to 2.91
        (
            View {
                blur_px: 2.0,
                ..SHARP
            },
            4.6,
            5.4,
        ), // 4.96 to 5.05
        // The same blur reads the same nearer, further, at an angle and at half the exposure.
        (
            View {
                blur_px: 1.0,
                distance_m: 1.0,
                ..SHARP
            },
            2.65,
            3.1,
        ), // 2.92 to 2.93
        (
            View {
                blur_px: 1.0,
                distance_m: 4.0,
                ..SHARP
            },
            2.5,
            3.1,
        ), // 2.71 to 2.83
        (
            View {
                blur_px: 1.0,
                yaw_deg: 40.0,
                ..SHARP
            },
            2.65,
            3.1,
        ), // 2.84 to 2.87
        (
            View {
                blur_px: 1.0,
                exposure: 0.5,
                ..SHARP
            },
            2.65,
            3.1,
        ), // 2.83 to 2.93
    ];
    for (view, least, most) in rows {
        let read = readings(view);
        println!("{view:?}: {read:?}");
        for reading in read {
            let reading = reading.unwrap_or_else(|| panic!("{view:?}: a tag wasn't measured"));
            assert!((least..most).contains(&reading), "{view:?}: {reading:.3}");
        }
    }
    // A tag too small in the frame to measure says so rather than guessing: at 4 m and 60 degrees
    // its square is under 32 px across.
    let small = View {
        distance_m: 4.0,
        yaw_deg: 60.0,
        blur_px: 1.0,
        ..SHARP
    };
    assert_eq!(readings(small), vec![None; TRIALS.len()]);
}

/// A camera's response, as tests/dial.rs has it: straight to 172, then a slope of 0.2 to full
/// white. A ThriftyCam read a sharp tag's edge true (within 3%) to a white of about 170, then
/// falsely sharper, 10% by 175 and 30% by 200 (1.5 px reading 1.37 at 175, 1.04 at 201). A sharp
/// tag rendered through this response reads true to 170, then 9% sharp by 180 and 17% by 188:
/// from about where the camera did, if not as far.
fn tone(linear: f64) -> u8 {
    const KNEE: f64 = 172.0;
    const SQUEEZE: f64 = 0.2;
    let level = if linear <= KNEE {
        linear
    } else {
        KNEE + SQUEEZE * (linear - KNEE)
    };
    level.round().clamp(0.0, 255.0) as u8
}

/// What the view's focus goes by for one tag rendered at `blur_px` and shown `brighter` times as
/// bright through the camera's response, and that tag's white; and the width its edge would read if
/// nothing were left out for being too bright: the same frame scaled down until its white is 130,
/// which keeps the edge's shape (a width is a step over a slope) but brings it under the ceiling.
fn through_the_camera(blur_px: f64, brighter: f64) -> (focus::Reading, Option<u8>, Option<f64>) {
    let (id, seed, offset) = TRIALS[1];
    let (cx, cy) = (
        (f64::from(CAMERA.width) - 1.0) / 2.0,
        (f64::from(CAMERA.height) - 1.0) / 2.0,
    );
    let face_out =
        Rotation3::from_matrix_unchecked(Matrix3::from_diagonal(&Vector3::new(1.0, -1.0, -1.0)));
    let tag = Placed {
        payload: Payload::Bits(render::tag36h11_code(id)),
        rotation: Rotation3::new(Vector3::new(0.05, 0.0, 0.1)) * face_out,
        translation_m: CAMERA.point_at(cx + offset.0, cy + offset.1, 1.5),
        black: 25.0,
        white: 210.0,
    };
    let optics = Optics {
        blur_px,
        exposure: 1.0,
    };
    let mut image = CAMERA.render(&[tag], optics, seed);
    for pixel in &mut image.pixels {
        *pixel = tone(f64::from(*pixel) * brighter);
    }
    let mut detector = Detector::new(16).unwrap();
    let found = detector.detect(&image).unwrap();
    let Some(detection) = found.iter().find(|d| d.id == id) else {
        return (focus::Reading::NoTag, None, None);
    };
    let tags = [visor::wire::Tag {
        id: detection.id as u16,
        hamming: 0,
        decision_margin: 50.0,
        corners: detection.corners.map(|c| c as f32),
        poses: None,
    }];
    let measured = focus::measure(&image, &tags);
    let white = measured.light().map(|l| l.white);
    let shape = white.and_then(|white| {
        let down = 130.0 / f64::from(white.max(1));
        let mut scaled = image.clone();
        for pixel in &mut scaled.pixels {
            *pixel = (f64::from(*pixel) * down).round() as u8;
        }
        focus::edge_px(&scaled, &tags[0].corners)
    });
    (measured.reading(), white, shape)
}

#[test]
fn an_overexposed_tag_never_reads_sharper_than_the_same_tag_well_exposed() {
    use visor::stream::light::TOO_BRIGHT_ABOVE;
    // At each blur: the tag well exposed (its white about 130, the dial's target), then brighter
    // and brighter through the camera's squeezed, clipping response.
    for blur_px in [0.0, 0.5, 1.0] {
        let (well, white, _) = through_the_camera(blur_px, 0.62);
        let well = well
            .edge_px()
            .unwrap_or_else(|| panic!("blur {blur_px}: well exposed, white {white:?}: {well:?}"));
        println!("blur {blur_px}: well exposed {well:.3} px, white {white:?}");
        for brighter in [0.7, 0.8, 0.85, 0.9, 1.0, 1.2, 1.5, 2.0, 3.0] {
            let (reading, white, shape) = through_the_camera(blur_px, brighter);
            println!("  x{brighter}: {reading:?}, white {white:?}, its shape reads {shape:?}");
            let white = white.expect("the tag is found");
            let shape = shape.expect("its shape is measured");
            // The response itself, as the camera's: true to a white of about 170, falsely sharp
            // past it, as the camera read.
            if white <= TOO_BRIGHT_ABOVE {
                assert!(
                    shape >= well * 0.97,
                    "x{brighter}: shape {shape:.3}, {well:.3}"
                );
            }
            if (185..250).contains(&white) {
                assert!(
                    shape < well * 0.95,
                    "x{brighter}: shape {shape:.3}, {well:.3}"
                );
            }
            // Past the trusted white, a reading doesn't count at all; below it, one that counts
            // is within a few percent of the well exposed one.
            if let Some(width) = reading.edge_px() {
                assert!(
                    (width - well).abs() <= well * 0.03,
                    "blur {blur_px}, x{brighter}: {width:.3} against {well:.3}, white {white}"
                );
            }
            if white > TOO_BRIGHT_ABOVE {
                assert_eq!(reading, focus::Reading::TooBright, "x{brighter}");
            }
        }
    }
}
