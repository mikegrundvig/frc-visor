//! A tag's pose from its four corners: IPPE, Collins and Bartoli's "Infinitesimal Plane-based Pose
//! Estimation" (IJCV 2014), as OpenCV's `solvePnPGeneric` with `SOLVEPNP_IPPE_SQUARE` does it.
//!
//! A flat square seen by one camera fits two poses, mirror images about the line of sight; IPPE
//! gives both, best first, each with its reprojection error. The robot chooses between them (with
//! its gyro, or other tags). Poses are the tag in the camera's frame, OpenCV's axes: x right, y
//! down, z out of the lens; the tag's own frame, centred on it, has x right and y up as the tag is
//! printed, and z out of its face, toward a camera that sees it.

use nalgebra::{Matrix3, Rotation3, SMatrix, SVector, Vector3};

use crate::lens::Lens;

/// One of a tag's two poses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TagPose {
    /// The tag's rotation, as an axis scaled by its angle in radians (OpenCV's rvec).
    pub rvec: Vector3<f64>,
    /// The tag's centre, in metres (OpenCV's tvec).
    pub tvec: Vector3<f64>,
    /// How far the corners land from where this pose puts them, root-mean-square per coordinate,
    /// in pixels, as OpenCV reports it.
    pub error_px: f64,
}

/// Both of a tag's poses, best first, from its corners (x0, y0 … x3, y3 in pixels, in OpenCV's
/// corner order: the printed tag's top-left, top-right, bottom-right, bottom-left). `None` when the
/// corners don't make a square that can be solved (a degenerate or edge-on view).
pub fn solve_square(corners: &[f64; 8], lens: &Lens, tag_size_m: f64) -> Option<[TagPose; 2]> {
    let seen: [(f64, f64); 4] =
        std::array::from_fn(|i| lens.undistort(corners[2 * i], corners[2 * i + 1]));
    solve_seen(&seen, corners, lens, tag_size_m)
}

/// The solve, from the corners already undistorted (`seen`); `corners` and `lens` only measure the
/// reprojection errors.
fn solve_seen(
    seen: &[(f64, f64); 4],
    corners: &[f64; 8],
    lens: &Lens,
    tag_size_m: f64,
) -> Option<[TagPose; 2]> {
    let half = tag_size_m / 2.0;
    let model = [(-half, half), (half, half), (half, -half), (-half, -half)];
    let h = homography(&model, seen)?;
    // The homography's value and slope at the tag's centre: where the centre is seen, and how the
    // view stretches around it. IPPE solves the rotation from those alone.
    let (p, q) = (h[(0, 2)], h[(1, 2)]);
    let jacobian = [
        [h[(0, 0)] - h[(2, 0)] * p, h[(0, 1)] - h[(2, 1)] * p],
        [h[(1, 0)] - h[(2, 0)] * q, h[(1, 1)] - h[(2, 1)] * q],
    ];
    let rotations = rotations(jacobian, p, q)?;

    let mut solutions = rotations.map(|r| {
        let t = translation(&model, seen, &r);
        (r, t, squared_error(&model, seen, &r, &t))
    });
    // Best first, by the error in the undistorted view, as OpenCV orders them.
    if solutions[1].2 < solutions[0].2 {
        solutions.swap(0, 1);
    }
    Some(solutions.map(|(r, t, _)| TagPose {
        rvec: Rotation3::from_matrix(&r).scaled_axis(),
        tvec: t,
        error_px: pixel_error(&model, corners, lens, &r, &t),
    }))
}

/// The homography taking the tag's plane (its x, y) to the undistorted view, with h22 = 1: the
/// unique one through four points.
fn homography(model: &[(f64, f64); 4], seen: &[(f64, f64); 4]) -> Option<Matrix3<f64>> {
    let mut a = SMatrix::<f64, 8, 8>::zeros();
    let mut b = SVector::<f64, 8>::zeros();
    for (i, (&(x, y), &(u, v))) in model.iter().zip(seen).enumerate() {
        let (r, s) = (2 * i, 2 * i + 1);
        a.row_mut(r)
            .copy_from_slice(&[x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y]);
        a.row_mut(s)
            .copy_from_slice(&[0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y]);
        b[r] = u;
        b[s] = v;
    }
    let h = a.lu().solve(&b)?;
    Some(Matrix3::new(
        h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1.0,
    ))
}

/// IPPE's two rotations from the homography's slope (`j`) and value (`p`, `q`) at the tag's centre
/// (OpenCV's `IPPE::PoseSolver::computeRotations`).
fn rotations(j: [[f64; 2]; 2], p: f64, q: f64) -> Option<[Matrix3<f64>; 2]> {
    // A rotation taking the camera's z axis onto the line of sight to the tag's centre.
    let rv = rotation_onto_z(&Vector3::new(p, q, 1.0)).transpose();
    let b = [
        [rv[(0, 0)] - p * rv[(2, 0)], rv[(0, 1)] - p * rv[(2, 1)]],
        [rv[(1, 0)] - q * rv[(2, 0)], rv[(1, 1)] - q * rv[(2, 1)]],
    ];
    let det = b[0][0] * b[1][1] - b[0][1] * b[1][0];
    if det.abs() < 1e-300 {
        return None;
    }
    let inv = [
        [b[1][1] / det, -b[0][1] / det],
        [-b[1][0] / det, b[0][0] / det],
    ];
    let a = [
        [
            inv[0][0] * j[0][0] + inv[0][1] * j[1][0],
            inv[0][0] * j[0][1] + inv[0][1] * j[1][1],
        ],
        [
            inv[1][0] * j[0][0] + inv[1][1] * j[1][0],
            inv[1][0] * j[0][1] + inv[1][1] * j[1][1],
        ],
    ];
    // The largest singular value of a, from a's own normal matrix.
    let ata00 = a[0][0] * a[0][0] + a[0][1] * a[0][1];
    let ata01 = a[0][0] * a[1][0] + a[0][1] * a[1][1];
    let ata11 = a[1][0] * a[1][0] + a[1][1] * a[1][1];
    let gamma2 = 0.5 * (ata00 + ata11 + ((ata00 - ata11).powi(2) + 4.0 * ata01 * ata01).sqrt());
    let gamma = gamma2.sqrt();
    if !gamma.is_finite() || gamma < f64::from(f32::EPSILON) {
        return None;
    }
    let r = [
        [a[0][0] / gamma, a[0][1] / gamma],
        [a[1][0] / gamma, a[1][1] / gamma],
    ];
    // The two ways to complete the 2x2 block into a rotation: the third row's two values have
    // fixed sizes and either sign together. Rounding can push a size a hair below zero.
    let b0 = (1.0 - r[0][0] * r[0][0] - r[1][0] * r[1][0])
        .max(0.0)
        .sqrt();
    let mut b1 = (1.0 - r[0][1] * r[0][1] - r[1][1] * r[1][1])
        .max(0.0)
        .sqrt();
    if -(r[0][0] * r[0][1] + r[1][0] * r[1][1]) < 0.0 {
        b1 = -b1;
    }
    let complete = |b0: f64, b1: f64| {
        let first = Vector3::new(r[0][0], r[1][0], b0);
        let second = Vector3::new(r[0][1], r[1][1], b1);
        rv * Matrix3::from_columns(&[first, second, first.cross(&second)])
    };
    Some([complete(b0, b1), complete(-b0, -b1)])
}

/// A rotation taking `a` onto the z axis (OpenCV's `rotateVec2ZAxis`).
fn rotation_onto_z(a: &Vector3<f64>) -> Matrix3<f64> {
    let a = a.normalize();
    let (ax, ay, az) = (a.x, a.y, a.z);
    if (1.0 + az).abs() < 1e-12 {
        // a points straight down -z: turn half round the x axis.
        return Matrix3::new(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
    }
    let d = 1.0 / (1.0 + az);
    Matrix3::new(
        1.0 - ax * ax * d,
        -ax * ay * d,
        -ax,
        -ax * ay * d,
        1.0 - ay * ay * d,
        -ay,
        ax,
        ay,
        1.0 - (ax * ax + ay * ay) * d,
    )
}

/// The translation that best fits a rotation to the view: least squares over the four corners, in
/// the undistorted view (OpenCV's `IPPE::PoseSolver::computeTranslation`).
fn translation(model: &[(f64, f64); 4], seen: &[(f64, f64); 4], r: &Matrix3<f64>) -> Vector3<f64> {
    // Each corner gives two equations linear in t: u (r3·X + tz) = r1·X + tx, and the same in v.
    let mut normal = Matrix3::<f64>::zeros();
    let mut rhs = Vector3::<f64>::zeros();
    for (&(x, y), &(u, v)) in model.iter().zip(seen) {
        let point = Vector3::new(x, y, 0.0);
        let depth = r.row(2).dot(&point.transpose());
        for (coefficients, target) in [
            (
                Vector3::new(1.0, 0.0, -u),
                u * depth - r.row(0).dot(&point.transpose()),
            ),
            (
                Vector3::new(0.0, 1.0, -v),
                v * depth - r.row(1).dot(&point.transpose()),
            ),
        ] {
            normal += coefficients * coefficients.transpose();
            rhs += coefficients * target;
        }
    }
    normal.lu().solve(&rhs).unwrap_or_else(Vector3::zeros)
}

/// The sum of squared distances, in the undistorted view, between the corners and where a pose
/// puts them.
fn squared_error(
    model: &[(f64, f64); 4],
    seen: &[(f64, f64); 4],
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
) -> f64 {
    model
        .iter()
        .zip(seen)
        .map(|(&(x, y), &(u, v))| {
            let c = r * Vector3::new(x, y, 0.0) + t;
            (c.x / c.z - u).powi(2) + (c.y / c.z - v).powi(2)
        })
        .sum()
}

/// The root-mean-square error per coordinate, in pixels, through the lens: OpenCV's
/// reprojection error.
fn pixel_error(
    model: &[(f64, f64); 4],
    corners: &[f64; 8],
    lens: &Lens,
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
) -> f64 {
    let sum: f64 = model
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| {
            let c = r * Vector3::new(x, y, 0.0) + t;
            let (u, v) = lens.project(c.x / c.z, c.y / c.z);
            (u - corners[2 * i]).powi(2) + (v - corners[2 * i + 1]).powi(2)
        })
        .sum();
    (sum / 8.0).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    struct Case {
        lens: Lens,
        corners: [f64; 8],
        undistorted: [f64; 8],
        poses: Vec<(Vector3<f64>, Vector3<f64>, f64)>,
    }

    fn cases() -> (f64, Vec<Case>) {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/testdata/pose-cases.json");
        let json: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let numbers = |v: &serde_json::Value| -> Vec<f64> {
            v.as_array()
                .unwrap()
                .iter()
                .map(|n| n.as_f64().unwrap())
                .collect()
        };
        let vector = |v: &serde_json::Value| {
            let n = numbers(v);
            Vector3::new(n[0], n[1], n[2])
        };
        let cases = json["cases"]
            .as_array()
            .unwrap()
            .iter()
            .map(|c| Case {
                lens: Lens::new(
                    c["fx"].as_f64().unwrap(),
                    c["fy"].as_f64().unwrap(),
                    c["cx"].as_f64().unwrap(),
                    c["cy"].as_f64().unwrap(),
                    &numbers(&c["distortion"]),
                )
                .unwrap(),
                corners: numbers(&c["corners"]).try_into().unwrap(),
                undistorted: numbers(&c["undistorted"]).try_into().unwrap(),
                poses: c["poses"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|p| {
                        (
                            vector(&p["rvec"]),
                            vector(&p["tvec"]),
                            p["error"].as_f64().unwrap(),
                        )
                    })
                    .collect(),
            })
            .collect();
        (json["tag_size_m"].as_f64().unwrap(), cases)
    }

    /// The angle between two rotations, in radians. From the quaternion between them with atan2,
    /// which stays exact near zero, where acos of a rounded cosine can come out NaN.
    fn angle_between(a: &Vector3<f64>, b: &Vector3<f64>) -> f64 {
        let between =
            UnitQuaternion::from_scaled_axis(*a).inverse() * UnitQuaternion::from_scaled_axis(*b);
        2.0 * between.vector().norm().atan2(between.scalar().abs())
    }

    #[test]
    fn undistortion_is_exact_and_close_to_opencv() {
        let (_, cases) = cases();
        let (mut ours, mut opencv, mut apart): (f64, f64, f64) = (0.0, 0.0, 0.0);
        for case in &cases {
            let lens = &case.lens;
            for i in 0..4 {
                let (u, v) = (case.corners[2 * i], case.corners[2 * i + 1]);
                let (x, y) = lens.undistort(u, v);
                let (cv_x, cv_y) = (case.undistorted[2 * i], case.undistorted[2 * i + 1]);
                // Back through the lens, each should land on the corner it came from.
                let (back_u, back_v) = lens.project(x, y);
                ours = ours.max((back_u - u).hypot(back_v - v));
                let (cv_u, cv_v) = lens.project(cv_x, cv_y);
                opencv = opencv.max((cv_u - u).hypot(cv_v - v));
                apart = apart.max((x - cv_x).abs().max((y - cv_y).abs()));
            }
        }
        eprintln!("round trip, worst: Visor {ours:e} px, OpenCV {opencv:e} px; apart {apart:e}");
        assert!(ours < 1e-6, "Visor's undistortion is off by {ours} px");
        // OpenCV stops after five refinements; on strongly distorted corners that leaves it
        // hundredths of a pixel short, which is all that separates the two.
        assert!(apart < 1e-4, "{apart}");
    }

    /// The worst rotation angle, distance (relative) and reprojection error between Visor's poses
    /// and OpenCV's, over every case.
    fn worst_against_opencv(solve: impl Fn(&Case, f64) -> [TagPose; 2]) -> (f64, f64, f64) {
        let (size, cases) = cases();
        let (mut angle, mut distance, mut error): (f64, f64, f64) = (0.0, 0.0, 0.0);
        for case in &cases {
            let poses = solve(case, size);
            // When the two are almost equally good, rounding can order them either way.
            let close_call =
                (case.poses[0].2 - case.poses[1].2).abs() < 1e-3 * case.poses[1].2.max(1e-9);
            let swapped = close_call
                && angle_between(&poses[0].rvec, &case.poses[0].0)
                    > angle_between(&poses[0].rvec, &case.poses[1].0);
            let order = if swapped { [1, 0] } else { [0, 1] };
            for (pose, k) in poses.iter().zip(order) {
                let (rvec, tvec, opencv_error) = &case.poses[k];
                angle = angle.max(angle_between(&pose.rvec, rvec));
                distance = distance.max((pose.tvec - tvec).norm() / tvec.norm());
                error = error.max((pose.error_px - opencv_error).abs());
            }
        }
        (angle, distance, error)
    }

    #[test]
    fn the_solve_is_opencv_s_from_the_same_undistorted_corners() {
        let (angle, distance, error) = worst_against_opencv(|case, size| {
            let seen =
                std::array::from_fn(|i| (case.undistorted[2 * i], case.undistorted[2 * i + 1]));
            solve_seen(&seen, &case.corners, &case.lens, size).expect("a solution")
        });
        eprintln!("same input, worst: {angle:e} rad, {distance:e} of the distance, {error:e} px");
        assert!(
            angle < 1e-8 && distance < 1e-8 && error < 1e-6,
            "{angle} {distance} {error}"
        );
    }

    #[test]
    fn from_pixels_the_poses_differ_from_opencv_s_only_by_its_undistortion() {
        let (angle, distance, error) = worst_against_opencv(|case, size| {
            solve_square(&case.corners, &case.lens, size).expect("a solution")
        });
        eprintln!("from pixels, worst: {angle:e} rad, {distance:e} of the distance, {error:e} px");
        // The solve is OpenCV's exactly (the test above), so this gap is OpenCV's undistortion,
        // which stops five refinements in and is up to 0.07 px short on these deliberately strong
        // lenses (the undistortion test). It moves their poses by up to about 0.2 degrees and a
        // tenth of a percent of the distance. This bounds it, with room.
        assert!(
            angle < 1e-2 && distance < 5e-3 && error < 0.1,
            "{angle} {distance} {error}"
        );
    }

    #[test]
    fn a_degenerate_square_has_no_pose() {
        let lens = Lens::new(1000.0, 1000.0, 800.0, 650.0, &[]).unwrap();
        // All four corners on one point, and on one line.
        for corners in [
            [800.0; 8],
            [700.0, 650.0, 750.0, 650.0, 800.0, 650.0, 850.0, 650.0],
        ] {
            assert!(
                solve_square(&corners, &lens, 0.1651).is_none(),
                "{corners:?}"
            );
        }
    }
}
