//! Visor's solve of the corners PhotonVision's simulation saw (testdata/photonvision-cases.json).
//!
//! Visor's answers are kept in testdata/photonvision-visor.json, as the wire carries them: OpenCV's
//! rvec and tvec for each pose, best first, and each one's error. The robot client's tests convert
//! them to WPILib's axes and check they're PhotonVision's own transforms. This test checks Visor
//! still gives those answers (VISOR_WRITE_GOLDEN=1 writes them afresh).

use serde_json::{Value, json};
use visor::lens::Lens;
use visor::pose::solve_square;

fn testdata(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("testdata")
        .join(name)
}

fn numbers(value: &Value) -> Vec<f64> {
    value
        .as_array()
        .unwrap()
        .iter()
        .map(|n| n.as_f64().unwrap())
        .collect()
}

/// Visor's answer for every case: [best, other], each {rvec, tvec, error}, as f32 on the wire.
fn answers() -> Value {
    let cases: Value = serde_json::from_str(
        &std::fs::read_to_string(testdata("photonvision-cases.json")).unwrap(),
    )
    .unwrap();
    let f = |key: &str| cases[key].as_f64().unwrap();
    let lens = Lens::new(f("fx"), f("fy"), f("cx"), f("cy"), &[]).unwrap();
    let size = f("tag_size_m");
    let solved: Vec<Value> = cases["cases"]
        .as_array()
        .unwrap()
        .iter()
        .map(|case| {
            let corners: [f64; 8] = numbers(&case["corners"]).try_into().unwrap();
            let poses = solve_square(&corners, &lens, size).expect("PhotonVision solved it");
            let wire = |v: nalgebra::Vector3<f64>| [v.x as f32, v.y as f32, v.z as f32];
            Value::Array(
                poses
                    .iter()
                    .map(|p| json!({"rvec": wire(p.rvec), "tvec": wire(p.tvec), "error": p.error_px as f32}))
                    .collect(),
            )
        })
        .collect();
    Value::Array(solved)
}

#[test]
fn visor_still_solves_photonvision_s_corners_the_same() {
    let fresh = answers();
    let path = testdata("photonvision-visor.json");
    if std::env::var_os("VISOR_WRITE_GOLDEN").is_some() {
        std::fs::write(&path, serde_json::to_string(&fresh).unwrap() + "\n").unwrap();
    }
    let kept: Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    let (fresh, kept) = (fresh.as_array().unwrap(), kept.as_array().unwrap());
    assert_eq!(fresh.len(), kept.len());
    // To a few float steps: x86 and Arm may round the last bit differently.
    let mut worst: f64 = 0.0;
    for (a, b) in fresh.iter().zip(kept) {
        for (pa, pb) in a.as_array().unwrap().iter().zip(b.as_array().unwrap()) {
            for key in ["rvec", "tvec"] {
                for (x, y) in numbers(&pa[key]).iter().zip(numbers(&pb[key])) {
                    worst = worst.max((x - y).abs() / y.abs().max(1.0));
                }
            }
        }
    }
    assert!(worst < 1e-5, "Visor's answers moved by {worst}");
}
