//! tools/package.sh makes the release archive a board's image installs Visor from: the program, its
//! notices (NAPA's, and the crates it's built from with their licences), and under `os/` everything the image needs for Visor (its account, its folder, its unit, and what
//! Spotter's agent needs to run Visor's pack), so no image keeps a copy of its own.
//!
//! The case packs a stand-in program and checks every member of the archive, by type, mode and
//! name, and that each of `os/`'s files is its source in `deploy/`.

use std::path::{Path, PathBuf};
use std::process::Command;

/// Each member of the archive: its type and mode as `tar -tv` lists them, its name, and for a file
/// of `os/`, its source in the repository.
const MEMBERS: &[(&str, &str, Option<&str>)] = &[
    ("drwxr-xr-x", "./", None),
    ("-rw-r--r--", "./LICENSE", None),
    ("-rw-r--r--", "./NOTICE", None),
    ("-rw-r--r--", "./THIRD-PARTY-LICENSES", None),
    ("-rw-r--r--", "./chrony.conf", None),
    ("drwxr-xr-x", "./os/", None),
    ("drwxr-xr-x", "./os/usr/", None),
    ("drwxr-xr-x", "./os/usr/lib/", None),
    ("drwxr-xr-x", "./os/usr/lib/systemd/", None),
    ("drwxr-xr-x", "./os/usr/lib/systemd/system/", None),
    (
        "drwxr-xr-x",
        "./os/usr/lib/systemd/system/frc-spotter.service.d/",
        None,
    ),
    (
        "-rw-r--r--",
        "./os/usr/lib/systemd/system/frc-spotter.service.d/visor.conf",
        Some("deploy/frc-spotter-visor.conf"),
    ),
    (
        "-rw-r--r--",
        "./os/usr/lib/systemd/system/visor.service",
        Some("deploy/visor.service"),
    ),
    ("drwxr-xr-x", "./os/usr/lib/sysusers.d/", None),
    (
        "-rw-r--r--",
        "./os/usr/lib/sysusers.d/visor.conf",
        Some("deploy/visor.sysusers"),
    ),
    ("drwxr-xr-x", "./os/usr/lib/tmpfiles.d/", None),
    (
        "-rw-r--r--",
        "./os/usr/lib/tmpfiles.d/visor.conf",
        Some("deploy/visor.tmpfiles"),
    ),
    ("drwxr-xr-x", "./os/usr/share/", None),
    ("drwxr-xr-x", "./os/usr/share/polkit-1/", None),
    ("drwxr-xr-x", "./os/usr/share/polkit-1/rules.d/", None),
    (
        "-rw-r--r--",
        "./os/usr/share/polkit-1/rules.d/50-frc-spotter-visor.rules",
        Some("deploy/50-frc-spotter-visor.rules"),
    ),
    ("drwxr-xr-x", "./spotter/", None),
    ("drwxr-xr-x", "./spotter/visor/", None),
    ("-rw-r--r--", "./spotter/visor/README.md", None),
    ("-rwxr-xr-x", "./spotter/visor/configure", None),
    ("-rwxr-xr-x", "./spotter/visor/install", None),
    ("-rwxr-xr-x", "./spotter/visor/journal", None),
    ("-rw-r--r--", "./spotter/visor/pack.yaml", None),
    ("-rwxr-xr-x", "./spotter/visor/status", None),
    ("drwxr-xr-x", "./vendor/", None),
    ("drwxr-xr-x", "./vendor/napa/", None),
    (
        "-rw-r--r--",
        "./vendor/napa/NOTICE",
        Some("vendor/napa/NOTICE"),
    ),
    (
        "-rw-r--r--",
        "./vendor/napa/THIRD_PARTY_NOTICES",
        Some("vendor/napa/THIRD_PARTY_NOTICES"),
    ),
    ("-rwxr-xr-x", "./visor", None),
];

fn repo() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

/// Packs a stand-in program into a folder of its own; returns the archive and the folder.
fn pack() -> (PathBuf, PathBuf) {
    let dir =
        Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("package-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let program = dir.join("visor");
    std::fs::write(&program, "#!/bin/sh\necho visor 2026.1.2\n").unwrap();
    let output = Command::new("bash")
        .arg(repo().join("tools/package.sh"))
        .arg(&program)
        .arg(dir.join("out"))
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "tools/package.sh: {stderr}");
    let archive = PathBuf::from(String::from_utf8(output.stdout).unwrap().trim());
    (archive, dir)
}

#[test]
fn the_release_archive_carries_the_program_its_pack_and_the_image_s_part() {
    let (archive, dir) = pack();
    let version = env!("CARGO_PKG_VERSION");
    assert_eq!(
        archive.file_name().unwrap().to_string_lossy(),
        format!("visor-{version}-linux-arm64.tar.gz")
    );

    let listed = Command::new("tar")
        .arg("-tvzf")
        .arg(&archive)
        .output()
        .unwrap();
    assert!(listed.status.success());
    let listed: Vec<(String, String)> = String::from_utf8(listed.stdout)
        .unwrap()
        .lines()
        .map(|line| {
            let fields: Vec<&str> = line.split_whitespace().collect();
            (fields[0].to_string(), fields[fields.len() - 1].to_string())
        })
        .collect();
    let wanted: Vec<(String, String)> = MEMBERS
        .iter()
        .map(|(mode, name, _)| (mode.to_string(), name.to_string()))
        .collect();
    assert_eq!(listed, wanted);

    let unpacked = dir.join("unpacked");
    std::fs::create_dir_all(&unpacked).unwrap();
    let status = Command::new("tar")
        .arg("-xzf")
        .arg(&archive)
        .arg("-C")
        .arg(&unpacked)
        .status()
        .unwrap();
    assert!(status.success());
    for (_, name, source) in MEMBERS {
        let Some(source) = source else { continue };
        assert_eq!(
            std::fs::read(unpacked.join(name)).unwrap(),
            std::fs::read(repo().join(source)).unwrap(),
            "{name} is {source}"
        );
    }
    // The crates the program is built from, each with its licences; not the tests' own, nor Visor.
    let crates = std::fs::read_to_string(unpacked.join("THIRD-PARTY-LICENSES")).unwrap();
    // (part of a line, whether it's listed)
    let rows = [
        ("\naxum@0.8.9 (1): MIT\n", true),
        ("\njpeg-encoder@", true),
        ("\nv4l@", true),
        ("\nureq@", false),
        ("\nimage@", false),
        ("\nvisor@", false),
    ];
    for (line, listed) in rows {
        assert_eq!(crates.contains(line), listed, "{line:?} in\n{crates}");
    }
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn visor_s_unit_names_no_board_s_cores() {
    // Which cores are fast is the board's, and its image's drop-in says so (CPUAffinity=).
    let unit = std::fs::read_to_string(repo().join("deploy/visor.service")).unwrap();
    let pinned: Vec<&str> = unit
        .lines()
        .filter(|line| line.starts_with("CPUAffinity=") || line.starts_with("AllowedCPUs="))
        .collect();
    assert!(pinned.is_empty(), "deploy/visor.service: {pinned:?}");
}
