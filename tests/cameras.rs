//! The cameras plugged in, as udev and the kernel describe them, in folders of the test's: each
//! camera in a USB port, with its capture and metadata devices, its by-id and by-path names as
//! udev makes them, and its USB device's serial, if any, where sysfs has it. And the names Visor's
//! Spotter pack reports for them, which are `visor cameras --names`'s.

use std::os::unix::fs::symlink;
use std::path::{Path, PathBuf};
use std::process::Command;

use visor::cameras::{Plugged, plugged};

fn folder(name: &str) -> PathBuf {
    let path = Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&path);
    std::fs::create_dir_all(&path).unwrap();
    path
}

/// A camera: the port it's in, its maker and model as udev writes them, and its serial (`None`:
/// its USB device has no `serial` at all).
struct Camera {
    port: &'static str,
    model: &'static str,
    serial: Option<&'static str>,
}

const THRIFTY: &str = "DCX-240524--XH_SPCA2630_PC_Camera";

fn no_serial(port: &'static str) -> Camera {
    Camera {
        port,
        model: THRIFTY,
        serial: None,
    }
}

fn with_serial(port: &'static str, serial: &'static str) -> Camera {
    Camera {
        port,
        model: "Thrifty_Cam",
        serial: Some(serial),
    }
}

/// The cameras plugged in, in order: camera `n` is `video{2n}` (capture) and `video{2n+1}`
/// (metadata). udev names each by maker, model and serial, which cameras without one, or with the
/// same one, share: the camera enumerated last keeps the name.
fn plug(root: &Path, cameras: &[Camera]) -> Vec<Plugged> {
    let [dev, by_id, by_path, sys, usb] =
        ["dev", "by-id", "by-path", "sys", "usb"].map(|name| root.join(name));
    for each in [&dev, &by_id, &by_path, &sys, &usb] {
        std::fs::create_dir_all(each).unwrap();
    }
    for (n, camera) in cameras.iter().enumerate() {
        let device = usb.join(camera.port);
        let interface = device.join(format!("{}:1.0", camera.port));
        std::fs::create_dir_all(&interface).unwrap();
        if let Some(serial) = camera.serial {
            std::fs::write(device.join("serial"), format!("{serial}\n")).unwrap();
        }
        let id = match camera.serial {
            Some(serial) if !serial.is_empty() => format!("usb-{}_{serial}", camera.model),
            _ => format!("usb-{}", camera.model),
        };
        for index in 0..2 {
            let video = format!("video{}", 2 * n + index);
            std::fs::write(dev.join(&video), "").unwrap();
            std::fs::create_dir_all(sys.join(&video)).unwrap();
            symlink(&interface, sys.join(&video).join("device")).unwrap();
            std::fs::write(sys.join(&video).join("index"), format!("{index}\n")).unwrap();
            let id_link = by_id.join(format!("{id}-video-index{index}"));
            let _ = std::fs::remove_file(&id_link);
            symlink(dev.join(&video), id_link).unwrap();
            let port = format!("platform-xhci-usb-0:{}:1.0-video-index{index}", camera.port);
            symlink(dev.join(&video), by_path.join(port)).unwrap();
        }
    }
    plugged(&by_id, &by_path, &sys).unwrap()
}

#[test]
fn a_camera_is_named_by_id_only_with_a_serial_of_its_own() {
    for (n, (why, cameras, expected)) in naming_cases().into_iter().enumerate() {
        let root = folder(&format!("cameras-{n}"));
        let found = plug(&root, &cameras);
        let names: Vec<String> = found
            .iter()
            .map(|p| p.name().file_name().unwrap().to_str().unwrap().to_owned())
            .collect();
        let wanted: Vec<String> = expected
            .iter()
            .flat_map(|stem| (0..2).map(move |index| format!("{stem}-video-index{index}")))
            .collect();
        assert_eq!(names, wanted, "{why}");
        for (each, camera) in found.iter().zip(cameras.iter().flat_map(|c| [c, c])) {
            let serial = camera.serial.filter(|s| !s.is_empty());
            assert_eq!(each.serial.as_deref(), serial, "{why}");
            assert_eq!(each.by_path.len(), 1, "{why}");
        }
    }
}

#[test]
fn the_pack_reports_each_camera_by_the_name_visor_gives_it() {
    // The capture device alone (index 0, not the metadata device), by the name the rule above
    // gives it: as `visor cameras --names` prints them, and as the pack's status reports them,
    // through the installed program, so the rule has one owner.
    for (n, (why, cameras, expected)) in naming_cases().into_iter().enumerate() {
        let root = folder(&format!("pack-cameras-{n}"));
        plug(&root, &cameras);
        let wanted: Vec<String> = expected
            .iter()
            .map(|stem| format!("{stem}-video-index0"))
            .collect();
        let printed = names_printed(&root);
        assert_eq!(printed, wanted, "visor cameras --names: {why}");
        install_visor(&root);
        let reported = status_cameras(&root);
        assert_eq!(reported, wanted.join(","), "the pack's status: {why}");
    }
}

#[test]
fn the_pack_reports_no_cameras_when_none_are_plugged_in_or_visor_is_missing() {
    let root = folder("pack-no-cameras");
    for each in ["by-id", "by-path", "sys"] {
        std::fs::create_dir_all(root.join(each)).unwrap();
    }
    assert!(names_printed(&root).is_empty());
    install_visor(&root);
    assert_eq!(status_cameras(&root), "");
    // No program installed: no names, rather than a status that fails.
    let without = folder("pack-no-visor");
    plug(&without, &[with_serial("1.1", "0001")]);
    assert_eq!(status_cameras(&without), "");
}

/// What `visor cameras --names` prints for the cameras plugged in under `root`.
fn names_printed(root: &Path) -> Vec<String> {
    let out = Command::new(env!("CARGO_BIN_EXE_visor"))
        .args(["cameras", "--names", "--by-id"])
        .arg(root.join("by-id"))
        .arg("--by-path")
        .arg(root.join("by-path"))
        .arg("--sysfs")
        .arg(root.join("sys"))
        .output()
        .unwrap();
    assert!(out.status.success(), "{out:?}");
    String::from_utf8(out.stdout)
        .unwrap()
        .lines()
        .map(str::to_owned)
        .collect()
}

/// The built program, installed as the pack finds it, in `root`'s `opt-visor`.
fn install_visor(root: &Path) {
    let installed = root.join("opt-visor");
    std::fs::create_dir_all(&installed).unwrap();
    symlink(env!("CARGO_BIN_EXE_visor"), installed.join("visor")).unwrap();
}

/// The `cameras` value of the pack's status, run as Spotter's agent runs it, with Visor installed
/// in `root`'s `opt-visor`, if it is.
fn status_cameras(root: &Path) -> String {
    let installed = root.join("opt-visor");
    let out = Command::new(concat!(env!("CARGO_MANIFEST_DIR"), "/spotter/visor/status"))
        .env("VISOR_DIR", &installed)
        .env("VISOR_BY_ID", root.join("by-id"))
        .env("VISOR_BY_PATH", root.join("by-path"))
        .env("VISOR_SYSFS", root.join("sys"))
        .env("VISOR_RUN", root.join("run"))
        .env("VISOR_MAIN_PID", "/bin/false")
        .output()
        .unwrap();
    assert!(out.status.success(), "{out:?}");
    let report: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    report["cameras"].as_str().unwrap().to_owned()
}

/// Cameras plugged in, and the names they're given, each without its `-video-indexN`.
fn naming_cases() -> [(&'static str, Vec<Camera>, Vec<String>); 5] {
    let port = |p: &str| format!("platform-xhci-usb-0:{p}:1.0");
    [
        (
            "no serial, alone: by its port, though the by-id name is its alone for now",
            vec![no_serial("1.1")],
            vec![port("1.1")],
        ),
        (
            "two with no serial: both by port, though udev points the by-id name at the second",
            vec![no_serial("1.1"), no_serial("1.2")],
            vec![port("1.1"), port("1.2")],
        ),
        (
            "a serial: by id",
            vec![with_serial("1.1", "0001")],
            vec!["usb-Thrifty_Cam_0001".into()],
        ),
        (
            "an empty serial: by its port",
            vec![with_serial("1.1", "")],
            vec![port("1.1")],
        ),
        (
            "a serial another camera has: udev gave the shared by-id name to the third, so the \
             first is by its port",
            vec![
                with_serial("1.1", "0001"),
                with_serial("1.2", "0002"),
                with_serial("1.3", "0001"),
            ],
            vec![
                port("1.1"),
                "usb-Thrifty_Cam_0002".into(),
                "usb-Thrifty_Cam_0001".into(),
            ],
        ),
    ]
}

#[test]
fn nothing_plugged_in_is_no_cameras() {
    let root = folder("no-cameras");
    let found: Vec<Plugged> = plugged(
        &root.join("by-id"),
        &root.join("by-path"),
        &root.join("sys"),
    )
    .unwrap();
    assert!(found.is_empty());
}
