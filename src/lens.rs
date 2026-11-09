//! A camera's lens calibration in OpenCV's model: focal lengths and optical centre in pixels, and
//! up to eight distortion coefficients in OpenCV's order (k1, k2, p1, p2, k3, k4, k5, k6), as
//! OpenCV's calibration gives them. It's for the resolution it was measured at.

use std::fmt;

/// How many times undistortion refines its guess at most; it stops sooner once the guess settles.
/// (OpenCV's own `undistortPoints` stops at five.)
const UNDISTORT_ITERATIONS: usize = 20;

#[derive(Debug, Clone, PartialEq)]
pub struct Lens {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// k1, k2, p1, p2, k3, k4, k5, k6: those not given are zero.
    pub distortion: [f64; 8],
}

/// Why a calibration is refused.
#[derive(Debug, Clone, PartialEq)]
pub struct LensError(pub String);

impl fmt::Display for LensError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for LensError {}

impl Lens {
    /// A calibration with 0, 4, 5 or 8 distortion coefficients.
    pub fn new(
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
        coefficients: &[f64],
    ) -> Result<Lens, LensError> {
        if ![fx, fy, cx, cy].iter().all(|v| v.is_finite()) || fx <= 0.0 || fy <= 0.0 {
            return Err(LensError(format!(
                "focal lengths are positive and the centre is finite: fx {fx}, fy {fy}, cx {cx}, cy {cy}"
            )));
        }
        if ![0, 4, 5, 8].contains(&coefficients.len()) {
            return Err(LensError(format!(
                "OpenCV's distortion has 0, 4, 5 or 8 coefficients, not {}",
                coefficients.len()
            )));
        }
        if !coefficients.iter().all(|v| v.is_finite()) {
            return Err(LensError("a distortion coefficient isn't a number".into()));
        }
        let mut distortion = [0.0; 8];
        distortion[..coefficients.len()].copy_from_slice(coefficients);
        Ok(Lens {
            fx,
            fy,
            cx,
            cy,
            distortion,
        })
    }

    /// Where a point in the image was looking: from its pixel, the point (x, y) with z = 1 in the
    /// camera's frame (OpenCV's: x right, y down, z out of the lens), with the distortion undone.
    pub fn undistort(&self, u: f64, v: f64) -> (f64, f64) {
        let (x0, y0) = ((u - self.cx) / self.fx, (v - self.cy) / self.fy);
        // OpenCV's fixed-point iteration: undo the distortion at the current guess, repeatedly.
        let (mut x, mut y) = (x0, y0);
        for _ in 0..UNDISTORT_ITERATIONS {
            let (radial, dx, dy) = self.distortion_at(x, y);
            let (next_x, next_y) = ((x0 - dx) / radial, (y0 - dy) / radial);
            let settled = (next_x - x).abs() < 1e-15 && (next_y - y).abs() < 1e-15;
            (x, y) = (next_x, next_y);
            if settled {
                break;
            }
        }
        (x, y)
    }

    /// The pixel a point (x, y) with z = 1 in the camera's frame lands on, through the lens.
    pub fn project(&self, x: f64, y: f64) -> (f64, f64) {
        let (radial, dx, dy) = self.distortion_at(x, y);
        (
            self.fx * (x * radial + dx) + self.cx,
            self.fy * (y * radial + dy) + self.cy,
        )
    }

    /// The radial scale and the tangential shift at a point, OpenCV's rational model.
    fn distortion_at(&self, x: f64, y: f64) -> (f64, f64, f64) {
        let [k1, k2, p1, p2, k3, k4, k5, k6] = self.distortion;
        let r2 = x * x + y * y;
        let radial =
            (1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))) / (1.0 + r2 * (k4 + r2 * (k5 + r2 * k6)));
        let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        (radial, dx, dy)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn strong() -> Lens {
        Lens::new(
            1000.0,
            1010.0,
            800.0,
            650.0,
            &[0.25, -0.15, 0.002, -0.003, 0.04, 0.2, -0.1, 0.03],
        )
        .unwrap()
    }

    #[test]
    fn undistorting_undoes_projecting() {
        let lens = strong();
        for (x, y) in [
            (0.0, 0.0),
            (0.3, -0.2),
            (-0.6, 0.5),
            (0.7, 0.55),
            (-0.75, -0.6),
        ] {
            let (u, v) = lens.project(x, y);
            let (back_x, back_y) = lens.undistort(u, v);
            assert!(
                (back_x - x).abs() < 1e-10 && (back_y - y).abs() < 1e-10,
                "({x}, {y})"
            );
        }
    }

    #[test]
    fn a_lens_without_distortion_is_a_pinhole() {
        let lens = Lens::new(1000.0, 1000.0, 800.0, 650.0, &[]).unwrap();
        assert_eq!(lens.project(0.1, -0.2), (900.0, 450.0));
        assert_eq!(lens.undistort(900.0, 450.0), (0.1, -0.2));
    }

    #[test]
    fn bad_calibrations_are_named() {
        let cases: [(f64, &[f64], &str); 3] = [
            (0.0, &[], "focal lengths are positive"),
            (1000.0, &[0.1, 0.2, 0.0], "0, 4, 5 or 8 coefficients, not 3"),
            (1000.0, &[f64::NAN, 0.0, 0.0, 0.0], "isn't a number"),
        ];
        for (fx, coefficients, why) in cases {
            let error = Lens::new(fx, 1000.0, 800.0, 650.0, coefficients)
                .unwrap_err()
                .0;
            assert!(error.contains(why), "{error}");
        }
    }
}
