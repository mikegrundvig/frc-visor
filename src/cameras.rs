//! The cameras plugged in, by the names udev gives them: `/dev/v4l/by-id/`, from the camera's
//! maker, model and serial, which follows the camera to any port; and `/dev/v4l/by-path/`, from
//! the USB port, which follows the port. Two cameras with the same serial, or none, share one by-id
//! name (udev keeps one of them), so a camera with no by-id name of its own has only its port's.

use std::collections::BTreeMap;
use std::io;
use std::path::{Path, PathBuf};

/// Where udev puts the names.
pub const BY_ID: &str = "/dev/v4l/by-id";
pub const BY_PATH: &str = "/dev/v4l/by-path";

/// One video device, with every name that leads to it.
#[derive(Debug, PartialEq, Eq)]
pub struct Plugged {
    /// The device itself, like `/dev/video0`: its number can change from boot to boot.
    pub device: PathBuf,
    pub by_id: Vec<PathBuf>,
    pub by_path: Vec<PathBuf>,
}

impl Plugged {
    /// The name to give Visor: its by-id name, or, without one, its port's.
    pub fn name(&self) -> &Path {
        self.by_id
            .first()
            .or(self.by_path.first())
            .map_or(self.device.as_path(), PathBuf::as_path)
    }
}

/// Every video device either folder names, in the devices' order. A folder that isn't there
/// (nothing plugged in) names none.
pub fn plugged(by_id: &Path, by_path: &Path) -> io::Result<Vec<Plugged>> {
    let mut devices: BTreeMap<PathBuf, Plugged> = BTreeMap::new();
    for (folder, is_id) in [(by_id, true), (by_path, false)] {
        for name in names(folder)? {
            let device = std::fs::canonicalize(&name)?;
            let entry = devices.entry(device.clone()).or_insert_with(|| Plugged {
                device,
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
    Ok(devices.into_values().collect())
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
