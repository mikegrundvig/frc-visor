//! The cameras plugged in, by the names udev gives them: `/dev/v4l/by-id/`, from the camera's
//! maker, model and serial, which follows the camera to any port; and `/dev/v4l/by-path/`, from
//! the USB port, which follows the port.
//!
//! A by-id name is the camera's own only when its USB device reports a serial. Without one (the
//! ThriftyCam has none), every camera of that model gets the same by-id name, and udev points it at
//! whichever was plugged in last: a name that works with one camera breaks when a second is plugged
//! in. So a camera with no serial is always named by its USB port, even alone. Two cameras with the
//! same serial share one by-id name too, and the one udev didn't give it is named by its port.

use std::collections::BTreeMap;
use std::io;
use std::path::{Path, PathBuf};

/// Where udev puts the names.
pub const BY_ID: &str = "/dev/v4l/by-id";
pub const BY_PATH: &str = "/dev/v4l/by-path";
/// Where the kernel describes each video device, `video0` and on: its `device/..` is the camera's
/// USB device, with its `serial` when it has one.
pub const SYSFS: &str = "/sys/class/video4linux";

/// One video device, with every name that leads to it.
#[derive(Debug, PartialEq, Eq)]
pub struct Plugged {
    /// The device itself, like `/dev/video0`: its number can change from boot to boot.
    pub device: PathBuf,
    /// The serial its USB device reports; none when it reports none, or an empty one.
    pub serial: Option<String>,
    pub by_id: Vec<PathBuf>,
    pub by_path: Vec<PathBuf>,
}

impl Plugged {
    /// The name to give Visor: its by-id name when it has a serial and udev gave it the name;
    /// otherwise its port's; and the device itself, unstable, only when udev named it neither way.
    pub fn name(&self) -> &Path {
        let own_id = if self.serial.is_some() {
            self.by_id.first()
        } else {
            None
        };
        own_id
            .or(self.by_path.first())
            .map_or(self.device.as_path(), PathBuf::as_path)
    }
}

/// Every video device either folder names, in the devices' order, each with its serial from the
/// kernel's description of it under `sysfs`. A folder that isn't there (nothing plugged in) names
/// none.
pub fn plugged(by_id: &Path, by_path: &Path, sysfs: &Path) -> io::Result<Vec<Plugged>> {
    let mut devices: BTreeMap<PathBuf, Plugged> = BTreeMap::new();
    for (folder, is_id) in [(by_id, true), (by_path, false)] {
        for name in names(folder)? {
            let device = std::fs::canonicalize(&name)?;
            let entry = devices.entry(device.clone()).or_insert_with(|| Plugged {
                device,
                serial: None,
                by_id: Vec::new(),
                by_path: Vec::new(),
            });
            if is_id {
                entry.by_id.push(name);
            } else {
                entry.by_path.push(name);
            }
        }
    }
    let mut found: Vec<Plugged> = devices.into_values().collect();
    for each in &mut found {
        each.serial = serial(sysfs, &each.device)?;
    }
    Ok(found)
}

/// The serial of the USB device a video device belongs to, as the kernel has it (with a newline);
/// none when there's no `serial` to read, or it's blank.
fn serial(sysfs: &Path, device: &Path) -> io::Result<Option<String>> {
    let Some(video) = device.file_name() else {
        return Ok(None);
    };
    let file = sysfs.join(video).join("device/../serial");
    match std::fs::read_to_string(file) {
        Ok(text) if text.trim().is_empty() => Ok(None),
        Ok(text) => Ok(Some(text.trim().to_string())),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(None),
        Err(error) => Err(error),
    }
}

/// The entries in a folder, sorted; none if it isn't there.
fn names(folder: &Path) -> io::Result<Vec<PathBuf>> {
    let entries = match std::fs::read_dir(folder) {
        Ok(entries) => entries,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(error) => return Err(error),
    };
    let mut names = entries
        .map(|entry| entry.map(|e| e.path()))
        .collect::<io::Result<Vec<_>>>()?;
    names.sort();
    Ok(names)
}
