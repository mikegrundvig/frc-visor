//! Where NAPA, through Visor's wrapper, stops finding tags: a table of rendered views over distance,
//! angle, blur, exposure and the print's black and white levels, each row asserting how many of
//! its tags are found and how near their corners are. Rows past a limit are written as misses, so
//! the limits are written down. The bounds are measurements of today's detector with a margin, not
//! targets: a change to the detector that moves them should move the table with it.
//!
//! The frames are made by `render`, from fixed seeds, so every machine sees the same ones. Each
//! row's measurement is printed (`cargo test --test detection_limits -- --nocapture`).

#[path = "../common/mod.rs"]
mod common;
mod render;

use nalgebra::{Matrix3, Rotation3, Vector3};
use render::{Camera, Optics, Payload, Placed};
use visor::napa::Detector;

/// The detector as a camera is configured by default (deploy/visor.example.toml).
const MIN_TAG_PX: i32 = 16;

/// The sweep's frame: smaller than a ThriftyCam's, with its lens, so a row renders quickly. A tag
/// at 1 m, its quiet zone included, still fits.
const CAMERA: Camera = Camera {
    width: 640,
    height: 512,
};

/// How a tag is seen in one row.
#[derive(Debug, Clone, Copy)]
struct View {
    distance_m: f64,
    /// About the tag's vertical axis, as the renderer's scenes turn it (with a small tilt and roll
    /// besides, so no edge lines up with the pixel grid).
    yaw_deg: f64,
    /// A Gaussian blur's standard deviation, in pixels.
    blur_px: f64,
    /// The exposure, as a scale on the scene's light.
    exposure: f64,
    /// The print's black and white grey levels at an exposure of 1.
    black: f64,
    white: f64,
}

/// A tag at 2 m, face on, in focus, at the exposure the renderer's scenes have, printed as a tag
/// under real light is: its black isn't 0 and its white isn't 255.
const AT_2_M: View = View {
    distance_m: 2.0,
    yaw_deg: 0.0,
    blur_px: 0.0,
    exposure: 1.0,
    black: 25.0,
    white: 210.0,
};

const AT_6_M: View = AT_2_M.distance(6.0);
const AT_8_M: View = AT_2_M.distance(8.0);

/// The same view with one thing changed.
impl View {
    const fn distance(self, distance_m: f64) -> View {
        View { distance_m, ..self }
    }

    const fn yaw(self, yaw_deg: f64) -> View {
        View { yaw_deg, ..self }
    }

    const fn blur(self, blur_px: f64) -> View {
        View { blur_px, ..self }
    }

    const fn exposure(self, exposure: f64) -> View {
        View { exposure, ..self }
    }

    const fn levels(self, black: f64, white: f64) -> View {
        View {
            black,
            white,
            ..self
        }
    }
}

/// What the detector does with a row's tags.
#[derive(Debug, Clone, Copy)]
enum Expect {
    /// Every trial's tag is found, each corner within this many pixels of where it truly is.
    Found { corner_px: f64 },
    /// No trial's tag is found: the view is past the detector's limit.
    Missed,
}

struct Row {
    view: View,
    expect: Expect,
}

const fn found(view: View, corner_px: f64) -> Row {
    Row {
        view,
        expect: Expect::Found { corner_px },
    }
}

const fn missed(view: View) -> Row {
    Row {
        view,
        expect: Expect::Missed,
    }
}

/// The table. Each bound is the worst corner measured over the row's trials (in its comment),
/// about doubled. Where recall falls off was measured with six trials a row, between the rows
/// here: the table keeps to rows where every tag is found or none is, so it doesn't rest on a
/// tag at the edge.
const ROWS: [Row; 33] = [
    // Distance, face on. Every tag is found across the field and beyond: the first are lost at
    // 20 m, where the square is 10 px across.
    found(AT_2_M.distance(1.0), 0.06),  // 0.028
    found(AT_2_M, 0.06),                // 0.028
    found(AT_2_M.distance(4.0), 0.06),  // 0.027
    found(AT_6_M, 0.08),                // 0.036
    found(AT_8_M, 0.2),                 // 0.090
    found(AT_2_M.distance(12.0), 0.35), // 0.163
    found(AT_2_M.distance(16.0), 1.1),  // 0.516
    // Yaw. At 2 m tags are found to 83 degrees and lost by 87; at 6 m found to 75 and lost by
    // 80; at 8 m found to 65, and some lost at 70.
    found(AT_2_M.yaw(30.0), 0.05), // 0.024
    found(AT_2_M.yaw(50.0), 0.05), // 0.020
    found(AT_2_M.yaw(70.0), 0.1),  // 0.043
    found(AT_2_M.yaw(80.0), 0.2),  // 0.093
    missed(AT_2_M.yaw(87.0)),
    found(AT_6_M.yaw(50.0), 0.1),  // 0.045
    found(AT_6_M.yaw(70.0), 0.35), // 0.155
    missed(AT_6_M.yaw(80.0)),
    found(AT_8_M.yaw(60.0), 0.45), // 0.224
    // Blur, as a Gaussian's sigma. Near tags take all of it: at 2 m they're found to 5 px, the
    // corners pulled 1.5 px. Far tags' cells are a few pixels wide: at 6 m found to 2 px, some
    // lost at 2.5 and all by 3; at 8 m found to 1.5, half lost at 2 and all by 2.5.
    found(AT_2_M.blur(1.0), 0.07), // 0.031
    found(AT_2_M.blur(2.0), 0.07), // 0.030
    found(AT_2_M.blur(3.0), 0.2),  // 0.093
    found(AT_6_M.blur(1.0), 0.12), // 0.060
    found(AT_6_M.blur(2.0), 1.0),  // 0.493
    missed(AT_6_M.blur(3.0)),
    found(AT_8_M.blur(1.5), 1.0), // 0.484
    missed(AT_8_M.blur(2.5)),
    // Exposure. Found to 0.12 and lost at 0.1, where the print's black and white are under 20
    // grey levels apart, NAPA's least contrast. Over, found to 5 at 2 m and 3 at 6 m, clipped
    // whites pulling the corners.
    missed(AT_2_M.exposure(0.1)),
    found(AT_2_M.exposure(0.3), 0.08), // 0.039
    found(AT_2_M.exposure(1.5), 0.11), // 0.054
    missed(AT_6_M.exposure(0.1)),
    found(AT_6_M.exposure(0.3), 0.13), // 0.063
    found(AT_6_M.exposure(1.5), 0.35), // 0.163
    // The print's black and white levels. A print 120 levels apart is found, and a washed-out
    // one; at 60 apart some tags among brighter or darker surroundings are lost, at 40 about
    // half, and at 10 all. A dark print, 5 and 40, loses half. Near 20 apart, NAPA's least
    // contrast, a tag that is found can have a corner pulled several pixels (8 px, at 6 m).
    found(AT_2_M.levels(60.0, 180.0), 0.07),  // 0.031
    found(AT_2_M.levels(200.0, 250.0), 0.08), // 0.040
    missed(AT_6_M.levels(115.0, 125.0)),
];

/// Each row is seen as four tags, one frame each: different IDs, backgrounds and noise (the
/// seed), and centres off the frame's middle by different fractions of a pixel. (ID, seed, offset
/// in pixels.)
const TRIALS: [(i32, u64, (f64, f64)); 4] = [
    (1, 1, (-10.0, -8.0)),
    (7, 2, (6.3, 3.2)),
    (300, 3, (-3.6, 7.5)),
    (586, 4, (9.9, -4.7)),
];

/// Visor's tag frame is the renderer's scenes' turned half a turn about x: y up and z out of the
/// face rather than y down and z into it (tools/render_frames.py).
fn printed_face_out() -> Rotation3<f64> {
    Rotation3::from_matrix_unchecked(Matrix3::from_diagonal(&Vector3::new(1.0, -1.0, -1.0)))
}

/// The tag a trial puts in front of the camera.
fn place(view: &View, id: i32, offset: (f64, f64)) -> Placed {
    let turned = Rotation3::new(Vector3::new(0.05, view.yaw_deg.to_radians(), 0.1));
    let (cx, cy) = (
        (f64::from(CAMERA.width) - 1.0) / 2.0,
        (f64::from(CAMERA.height) - 1.0) / 2.0,
    );
    Placed {
        payload: Payload::Bits(render::tag36h11_code(id)),
        rotation: turned * printed_face_out(),
        translation_m: CAMERA.point_at(cx + offset.0, cy + offset.1, view.distance_m),
        black: view.black,
        white: view.white,
    }
}

/// What a row's trials found: how many of the tags, the worst corner among them, and anything
/// found that isn't the tag.
struct Measured {
    found: usize,
    worst_corner_px: f64,
    others: Vec<i32>,
}

fn measure(detector: &mut Detector, view: &View) -> Measured {
    let mut measured = Measured {
        found: 0,
        worst_corner_px: 0.0,
        others: Vec::new(),
    };
    for (id, seed, offset) in TRIALS {
        let tag = place(view, id, offset);
        let optics = Optics {
            blur_px: view.blur_px,
            exposure: view.exposure,
        };
        let frame = CAMERA.render(&[tag], optics, seed);
        let truth = CAMERA.corners(&tag);
        for detection in detector.detect(&frame).unwrap() {
            if detection.id != id {
                measured.others.push(detection.id);
                continue;
            }
            measured.found += 1;
            let error = common::worst_corner(&detection.corners, &truth);
            measured.worst_corner_px = measured.worst_corner_px.max(error);
        }
    }
    measured
}

#[test]
fn recall_and_corners_across_the_table() {
    let mut detector = Detector::new(MIN_TAG_PX).unwrap();
    let mut failures = Vec::new();
    for row in &ROWS {
        let view = &row.view;
        let measured = measure(&mut detector, view);
        println!(
            "{view:?}: {} of {} found, worst corner {:.3} px",
            measured.found,
            TRIALS.len(),
            measured.worst_corner_px
        );
        if !measured.others.is_empty() {
            failures.push(format!(
                "{view:?}: found tags not there, {:?}",
                measured.others
            ));
        }
        match row.expect {
            Expect::Found { corner_px } => {
                if measured.found != TRIALS.len() {
                    failures.push(format!(
                        "{view:?}: {} of {} found, all expected",
                        measured.found,
                        TRIALS.len()
                    ));
                } else if measured.worst_corner_px > corner_px {
                    failures.push(format!(
                        "{view:?}: a corner {:.3} px off, {corner_px} allowed",
                        measured.worst_corner_px
                    ));
                }
            }
            Expect::Missed => {
                if measured.found != 0 {
                    failures.push(format!(
                        "{view:?}: {} found, a miss expected",
                        measured.found
                    ));
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Patterns that look like tags and aren't: a black border around cells that are no tag36h11
/// tag, within the three bits NAPA corrects.
fn lookalikes() -> [Payload; 7] {
    // Four cells of a real tag changed: tag36h11's tags differ in at least 11 cells however they
    // are turned, so this is four from its own tag and seven or more from any other.
    let four_changed = |id: i32| render::tag36h11_code(id) ^ (1 << 0 | 1 << 9 | 1 << 22 | 1 << 35);
    // A checkerboard of the cells: white where the row and column add to an even number.
    let checkerboard = (0..36)
        .filter(|k| (k / 6 + k % 6) % 2 == 0)
        .fold(0u64, |bits, k| bits | 1 << (35 - k));
    [
        Payload::Bits(four_changed(7)),
        Payload::Bits(four_changed(21)),
        Payload::Mirrored(render::tag36h11_code(7)),
        Payload::Mirrored(render::tag36h11_code(300)),
        Payload::Bits(checkerboard),
        // A plain black square, and a black frame around white.
        Payload::Bits(0),
        Payload::Bits((1 << 36) - 1),
    ]
}

#[test]
fn lookalikes_are_not_tags() {
    let camera = Camera {
        width: 1280,
        height: 640,
    };
    let tags: Vec<Placed> = lookalikes()
        .into_iter()
        .enumerate()
        .map(|(i, payload)| {
            // Four across and two down, at 2 m, each turned a little differently.
            let (u, v) = (
                160.0 + 320.0 * (i % 4) as f64,
                160.0 + 320.0 * (i / 4) as f64,
            );
            let turned = Rotation3::new(Vector3::new(0.05, (10.0 * i as f64).to_radians(), 0.1));
            Placed {
                payload,
                rotation: turned * printed_face_out(),
                translation_m: camera.point_at(u, v, 2.0),
                black: AT_2_M.black,
                white: AT_2_M.white,
            }
        })
        .collect();
    let mut detector = Detector::new(MIN_TAG_PX).unwrap();
    let views = [AT_2_M, AT_2_M.blur(1.5), AT_2_M.exposure(0.3)];
    for (seed, view) in views.iter().enumerate() {
        let optics = Optics {
            blur_px: view.blur_px,
            exposure: view.exposure,
        };
        let frame = camera.render(&tags, optics, seed as u64 + 1);
        let found: Vec<i32> = detector
            .detect(&frame)
            .unwrap()
            .iter()
            .map(|d| d.id)
            .collect();
        assert!(found.is_empty(), "{view:?}: lookalikes found as {found:?}");
    }
}

/// The sweep's renderer against OpenCV's: tools/render_frames.py's one-tag scene, drawn again at
/// its true pose, is found with the corners OpenCV's drawing has. So the sweep's tags are drawn
/// the right way round, and its corners mean what the rendered scenes' do.
#[test]
fn the_renderer_draws_tags_as_opencv_does() {
    let scene = common::scene("one-tag");
    let known = &scene.tags[0];
    let camera = Camera {
        width: scene.image.width,
        height: scene.image.height,
    };
    let tag = Placed {
        payload: Payload::Bits(render::tag36h11_code(known.id)),
        rotation: known.pose.rotation,
        translation_m: known.pose.translation_m,
        black: AT_2_M.black,
        white: AT_2_M.white,
    };
    assert!(common::worst_corner(&camera.corners(&tag), &known.corners) < 0.001);
    let frame = camera.render(
        &[tag],
        Optics {
            blur_px: 0.0,
            exposure: 1.0,
        },
        1,
    );
    let mut detector = Detector::new(MIN_TAG_PX).unwrap();
    let found = detector.detect(&frame).unwrap();
    assert_eq!(found.len(), 1);
    assert_eq!(found[0].id, known.id);
    let error = common::worst_corner(&found[0].corners, &known.corners);
    assert!(error < 0.1, "corners {error:.3} px from OpenCV's");
}
