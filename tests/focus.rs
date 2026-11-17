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
                white: 210.0,
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
