//! `visor check`, as the robot's deploy runs it before a configuration replaces the old one: the
//! built program on a file, its exit status and what it says.

use std::path::{Path, PathBuf};
use std::process::Command;

/// A configuration with one mistake in it.
const BROKEN: &str = r#"
    team = 1234
    [[camera]]
    name = "front-left"
    index = 0
    device = "/dev/v4l/by-id/usb-cam-video-index0"
    width = 1600
    height = 1304
    fps = 0
"#;

fn broken_file() -> PathBuf {
    let dir = std::env::temp_dir().join(format!("visor-check-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("visor.toml");
    std::fs::write(&path, BROKEN).unwrap();
    path
}

/// The configuration the robot writes for a board with none of its cameras.
fn empty_file() -> PathBuf {
    let dir = std::env::temp_dir().join(format!("visor-check-empty-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("visor.toml");
    std::fs::write(&path, "team = 1234\n").unwrap();
    path
}

#[test]
fn check_passes_a_good_file_and_names_what_is_wrong_with_a_bad_one() {
    let example = Path::new(env!("CARGO_MANIFEST_DIR")).join("deploy/visor.example.toml");
    // (the file, whether it passes, part of what's printed: stdout on a pass, stderr on a refusal)
    let cases = [
        (
            example.clone(),
            true,
            "camera 0 \"front-left\": 1600x1304 at 60 fps",
        ),
        (example, true, "tag36h11 tags 0.1651 m"),
        (
            empty_file(),
            true,
            "no camera: Visor opens none, and sits idle",
        ),
        (
            broken_file(),
            false,
            "camera \"front-left\": fps 0 isn't a frame rate",
        ),
        (
            PathBuf::from("/no/such/visor.toml"),
            false,
            "/no/such/visor.toml",
        ),
    ];
    for (path, passes, said) in &cases {
        let output = Command::new(env!("CARGO_BIN_EXE_visor"))
            .arg("check")
            .arg("--config")
            .arg(path)
            .output()
            .unwrap();
        let printed = if *passes {
            &output.stdout
        } else {
            &output.stderr
        };
        let printed = String::from_utf8_lossy(printed);
        assert_eq!(
            output.status.success(),
            *passes,
            "{}: {printed}",
            path.display()
        );
        assert!(printed.contains(said), "{}: {printed}", path.display());
    }
    for (path, _, _) in &cases[2..4] {
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }
}
