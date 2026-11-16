//! Finding a camera's USB device by what it is, when it has no video device to find it by.
//!
//! A USB 3 camera that falls back past USB 2 to USB 1's full speed (12 Mb/s, on a port with an
//! OHCI controller behind it) is on USB, but uvcvideo finds no streaming interface it can use
//! there: no video device, so no `/dev/v4l` name. It's found instead among every USB device the
//! kernel lists, by its vendor, product and serial, and only when exactly one device matches.

use std::fs;
use std::io;
use std::path::Path;

use super::{Findable, Identity, Link, link_at};

/// The USB devices with an identity.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Found {
    /// None: or the identity has no serial to tell cameras of one model apart.
    Nothing,
    /// Exactly one, on this link.
    One(Link),
    /// More than one: which is the camera can't be told.
    Many,
}

/// The USB devices under `devices` (`/sys/bus/usb/devices`: one entry per device and per
/// interface) whose vendor, product and serial are `identity`'s.
pub fn find_devices(devices: &Path, identity: &Identity) -> io::Result<Found> {
    if identity.findable() == Findable::No {
        return Ok(Found::Nothing);
    }
    let entries = match fs::read_dir(devices) {
        Ok(entries) => entries,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(Found::Nothing),
        Err(error) => return Err(error),
    };
    let mut matching = Vec::new();
    for entry in entries {
        let entry = entry?;
        // An interface's entry, like `6-1:1.0`, describes no device of its own.
        if entry.file_name().to_string_lossy().contains(':') {
            continue;
        }
        let Ok(folder) = fs::canonicalize(entry.path()) else {
            // Gone while being listed.
            continue;
        };
        if let Some(link) = link_at(&folder)?
            && link.identity == *identity
        {
            matching.push(link);
        }
    }
    let found = match matching.len() {
        0 => Found::Nothing,
        1 => Found::One(matching.remove(0)),
        _ => Found::Many,
    };
    Ok(found)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    /// A `/sys/bus/usb/devices` of the test's: each device's folder with its fields, and an
    /// interface entry for each.
    fn devices(root: &Path, each: &[(&str, &[(&str, &str)])]) -> PathBuf {
        let _ = fs::remove_dir_all(root);
        let listed = root.join("devices");
        fs::create_dir_all(&listed).unwrap();
        for (name, fields) in each {
            let folder = root.join("platform/fc880000.usb").join(name);
            fs::create_dir_all(folder.join(format!("{name}:1.0"))).unwrap();
            for (field, value) in *fields {
                fs::write(folder.join(field), format!("{value}\n")).unwrap();
            }
            std::os::unix::fs::symlink(&folder, listed.join(name)).unwrap();
            std::os::unix::fs::symlink(
                folder.join(format!("{name}:1.0")),
                listed.join(format!("{name}:1.0")),
            )
            .unwrap();
        }
        listed
    }

    fn camera(serial: &str, busnum: &str) -> Vec<(&'static str, String)> {
        vec![
            ("speed", "12".into()),
            ("version", " 2.10".into()),
            ("busnum", busnum.into()),
            ("devnum", "3".into()),
            ("idVendor", "05c8".into()),
            ("idProduct", "0a00".into()),
            ("serial", serial.into()),
        ]
    }

    /// A row: the devices listed with their fields, the identity looked for, and what's found:
    /// the bus of the one, or what else.
    type FindCase<'a> = (
        Vec<(&'a str, &'a [(&'a str, String)])>,
        &'a Identity,
        Result<u32, Found>,
    );

    #[test]
    fn a_camera_is_found_by_its_serial_only_when_exactly_one_device_has_it() {
        let root = std::env::temp_dir().join(format!("usb-find-{}", std::process::id()));
        let wanted = Identity {
            vendor: Some(0x05c8),
            product: Some(0x0a00),
            serial: Some("TC1".into()),
        };
        let no_serial = Identity {
            serial: None,
            ..wanted.clone()
        };
        let (one, other, hub) = (camera("TC1", "6"), camera("TC2", "4"), {
            let mut hub = camera("fc880000.usb", "6");
            hub[4] = ("idVendor", "1d6b".into());
            hub
        });
        let twin = camera("TC1", "4");
        // (the devices listed, the identity looked for, what's found: the bus of the one)
        let cases: [FindCase; 5] = [
            (
                vec![("6-1", &one), ("4-1", &other), ("usb6", &hub)],
                &wanted,
                Ok(6),
            ),
            (vec![("4-1", &other)], &wanted, Err(Found::Nothing)),
            (
                vec![("6-1", &one), ("4-1", &twin)],
                &wanted,
                Err(Found::Many),
            ),
            (vec![("6-1", &one)], &no_serial, Err(Found::Nothing)),
            (vec![], &wanted, Err(Found::Nothing)),
        ];
        for (listed, identity, expected) in cases {
            let owned: Vec<(&str, Vec<(&str, &str)>)> = listed
                .iter()
                .map(|(name, fields)| {
                    (
                        *name,
                        fields.iter().map(|(f, v)| (*f, v.as_str())).collect(),
                    )
                })
                .collect();
            let borrowed: Vec<(&str, &[(&str, &str)])> = owned
                .iter()
                .map(|(name, fields)| (*name, fields.as_slice()))
                .collect();
            let folder = devices(&root, &borrowed);
            let found = find_devices(&folder, identity).unwrap();
            match (found, expected) {
                (Found::One(link), Ok(bus)) => {
                    assert_eq!(link.bus, bus);
                    assert_eq!(link.mbps, 12);
                    assert_eq!(link.controller.as_deref(), Some("fc880000.usb"));
                }
                (found, Err(expected)) => assert_eq!(found, expected, "{listed:?}"),
                (found, expected) => panic!("{listed:?}: {found:?}, expected {expected:?}"),
            }
        }
        // No device list at all.
        assert_eq!(
            find_devices(&root.join("none"), &wanted).unwrap(),
            Found::Nothing
        );
        fs::remove_dir_all(&root).unwrap();
    }
}
