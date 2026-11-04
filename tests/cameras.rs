//! The cameras plugged in, as udev names them, in folders of the test's: two cameras with serials
//! of their own, each with its capture and metadata devices, and a third whose serial is the
//! first's, so udev's one by-id name for that serial leads to the first alone.

use std::os::unix::fs::symlink;
use std::path::{Path, PathBuf};

use visor::cameras::{Plugged, plugged};

fn folder(name: &str) -> PathBuf {
    let path = Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&path);
    std::fs::create_dir_all(&path).unwrap();
    path
}

#[test]
fn each_device_with_every_name_that_leads_to_it() {
    let root = folder("cameras");
    let dev = root.join("dev");
    let by_id = root.join("by-id");
    let by_path = root.join("by-path");
    for each in [&dev, &by_id, &by_path] {
        std::fs::create_dir_all(each).unwrap();
    }
    for n in 0..6 {
        std::fs::write(dev.join(format!("video{n}")), "").unwrap();
    }
    let link = |device: u32, folder: &Path, name: &str| {
        symlink(dev.join(format!("video{device}")), folder.join(name)).unwrap();
    };
    link(0, &by_id, "usb-Thrifty_Cam_0001-video-index0");
    link(1, &by_id, "usb-Thrifty_Cam_0001-video-index1");
    link(2, &by_id, "usb-Thrifty_Cam_0002-video-index0");
    link(3, &by_id, "usb-Thrifty_Cam_0002-video-index1");
    for (device, port) in [
        (0, "1.1"),
        (1, "1.1"),
        (2, "1.2"),
        (3, "1.2"),
        (4, "1.3"),
        (5, "1.3"),
    ] {
        let index = device % 2;
        link(
            device,
            &by_path,
            &format!("platform-xhci-usb-0:{port}:1.0-video-index{index}"),
        );
    }

    let found = plugged(&by_id, &by_path).unwrap();
    let names: Vec<(&str, &str)> = found
        .iter()
        .map(|p| {
            (
                p.device.file_name().unwrap().to_str().unwrap(),
                p.name().file_name().unwrap().to_str().unwrap(),
            )
        })
        .collect();
    assert_eq!(
        names,
        [
            ("video0", "usb-Thrifty_Cam_0001-video-index0"),
            ("video1", "usb-Thrifty_Cam_0001-video-index1"),
            ("video2", "usb-Thrifty_Cam_0002-video-index0"),
            ("video3", "usb-Thrifty_Cam_0002-video-index1"),
            // The third camera has no by-id name of its own: its port's is the one to give Visor.
            ("video4", "platform-xhci-usb-0:1.3:1.0-video-index0"),
            ("video5", "platform-xhci-usb-0:1.3:1.0-video-index1"),
        ]
    );
    let first = &found[0];
    assert_eq!(first.by_id.len(), 1);
    assert_eq!(first.by_path.len(), 1);
    assert!(found[4].by_id.is_empty());
}

#[test]
fn nothing_plugged_in_is_no_cameras() {
    let root = folder("no-cameras");
    let found: Vec<Plugged> = plugged(&root.join("by-id"), &root.join("by-path")).unwrap();
    assert!(found.is_empty());
}
