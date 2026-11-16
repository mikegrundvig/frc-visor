//! Powering a camera off and on through its USB Type-C port: the software equivalent of pulling
//! the plug, and the strongest recovery there is. A camera that stopped answering (the kernel
//! saying `device not accepting address` for half a minute) came back at SuperSpeed within 5 s of
//! its port's 5 V being cut and restored. It doesn't mend USB 3 contacts that aren't touching.
//!
//! The Type-C class gives each port a `port_type`: `[dual] source sink`, the bracketed one current.
//! Writing `sink` makes the port stop supplying 5 V, so the camera powers off and the controller's
//! buses go; writing the original back powers it again, and the camera enumerates 1 to 2 s later.
//!
//! Which port powers which camera: Linux 6.1 has no link in sysfs from a USB port to its Type-C
//! connector. So the rule is: a camera's USB controller (the `<address>.usb` part of its sysfs
//! path, like `fc000000.usb`) has a role switch, `/sys/class/usb_role/<address>.usb-role-switch`,
//! which only a dual-role (Type-C) controller has, and the system has exactly one Type-C port;
//! then that port powers it. In any other case the port's power can't be switched: a camera on a
//! Type-A port, fed from an always-on rail, is the ordinary case.
//!
//! `port_type` is root's to write; the release's udev rule gives the `video` group write access to
//! it (deploy/70-visor-usb-cameras.rules).

use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::usb::{Back, PowerPort, Seen, await_return};

/// Where the kernel puts the Type-C ports and the USB role switches.
pub const TYPEC: &str = "/sys/class/typec";
pub const ROLES: &str = "/sys/class/usb_role";
/// How long a port stays off: long enough for the camera to lose power and the controller to
/// notice; 3 s brought a hung camera back both times it was tried.
pub const OFF_FOR: Duration = Duration::from_secs(3);
/// How long a powered camera is looked for at most: it enumerates 1 to 2 s after its power comes
/// back.
pub const POWER_RETURN: Duration = Duration::from_secs(5);
/// What a port is set to while it's off: it takes power, and gives none.
const OFF: &str = "sink";

/// The port that powers a camera on the controller `controller`, or why none can be switched.
pub fn power_port(typec: &Path, roles: &Path, controller: Option<&str>) -> io::Result<PowerPort> {
    let fixed = |why: &str| Ok(PowerPort::Fixed(why.to_string()));
    let Some(controller) = controller else {
        return fixed("its USB controller can't be told from its sysfs path");
    };
    if !roles.join(format!("{controller}-role-switch")).exists() {
        return fixed("its USB controller has no role switch, so no Type-C port feeds it");
    }
    let ports = ports(typec)?;
    let [port] = ports.as_slice() else {
        return match ports.len() {
            0 => fixed("the system has no Type-C port"),
            n => Ok(PowerPort::Fixed(format!(
                "the system has {n} Type-C ports, and which one feeds it can't be told"
            ))),
        };
    };
    let now = fs::read_to_string(port.join("port_type"))?;
    match current(&now) {
        Some(OFF) => fixed("its Type-C port is set to take power, not give it"),
        Some(_) => Ok(PowerPort::Switchable(port.clone())),
        None => Ok(PowerPort::Fixed(format!(
            "its Type-C port's port_type reads {:?}",
            now.trim()
        ))),
    }
}

/// The Type-C ports under `typec`: `port0` and on, not their partners, cables or plugs.
fn ports(typec: &Path) -> io::Result<Vec<PathBuf>> {
    let entries = match fs::read_dir(typec) {
        Ok(entries) => entries,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(error) => return Err(error),
    };
    let mut ports = Vec::new();
    for entry in entries {
        let entry = entry?;
        let name = entry.file_name();
        let is_port = name
            .to_str()
            .and_then(|n| n.strip_prefix("port"))
            .is_some_and(|n| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()));
        if is_port {
            ports.push(entry.path());
        }
    }
    ports.sort();
    Ok(ports)
}

/// The current choice in a sysfs list like `[dual] source sink`: the bracketed one.
fn current(text: &str) -> Option<&str> {
    text.split_whitespace()
        .find_map(|word| word.strip_prefix('[')?.strip_suffix(']'))
}

/// Powers `port` off for [`OFF_FOR`] and back on to what it was, then waits up to
/// [`POWER_RETURN`] for the camera to come back (`look` and `pause` as
/// [`crate::usb::await_return`]'s). What the port was set to is written back whatever happens
/// after it was switched off: its power always comes back.
pub fn power_cycle(
    port: &Path,
    look: impl FnMut() -> Seen,
    mut pause: impl FnMut(Duration),
) -> io::Result<Back> {
    let file = port.join("port_type");
    let was = fs::read_to_string(&file)?;
    let Some(was) = current(&was).map(str::to_string) else {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("{} reads {:?}", file.display(), was.trim()),
        ));
    };
    fs::write(&file, OFF)?;
    pause(OFF_FOR);
    fs::write(&file, &was)?;
    Ok(await_return(None, POWER_RETURN, look, pause))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    /// A sysfs of the test's: these Type-C ports, each with its `port_type`, and role switches
    /// for these controllers.
    fn sysfs(root: &Path, ports: &[(&str, &str)], switches: &[&str]) -> (PathBuf, PathBuf) {
        let _ = fs::remove_dir_all(root);
        let (typec, roles) = (root.join("typec"), root.join("usb_role"));
        fs::create_dir_all(&typec).unwrap();
        fs::create_dir_all(&roles).unwrap();
        for (port, port_type) in ports {
            fs::create_dir_all(typec.join(port)).unwrap();
            fs::write(typec.join(port).join("port_type"), format!("{port_type}\n")).unwrap();
            // A port's partner sits beside it, and isn't a port.
            fs::create_dir_all(typec.join(format!("{port}-partner"))).unwrap();
        }
        for controller in switches {
            fs::create_dir_all(roles.join(format!("{controller}-role-switch"))).unwrap();
        }
        (typec, roles)
    }

    /// The camera, back at SuperSpeed.
    fn powered_up() -> crate::usb::Link {
        crate::usb::Link {
            name: "8-1".into(),
            mbps: 5000,
            version: crate::usb::Version {
                major: 3,
                minor: 10,
            },
            bus: 8,
            address: 2,
            controller: Some("fc000000.usb".into()),
        }
    }

    fn root(name: &str) -> PathBuf {
        std::env::temp_dir().join(format!("typec-{name}-{}", std::process::id()))
    }

    /// A row: the ports and their `port_type`, the role switches, the camera's controller, and
    /// the port or part of why there's none.
    type PortCase<'a> = (
        &'a [(&'a str, &'a str)],
        &'a [&'a str],
        Option<&'a str>,
        Result<&'a str, &'a str>,
    );

    #[test]
    fn a_camera_s_port_is_switchable_only_by_the_one_type_c_port_rule() {
        let root = root("rule");
        let dual = [("port0", "[dual] source sink")];
        let two = [("port0", "[dual] source sink"), ("port1", "[source] sink")];
        let sink = [("port0", "dual source [sink]")];
        let unreadable = [("port0", "dual source sink")];
        let switch = ["fc000000.usb"];
        // (the ports, the role switches, the camera's controller, the port or part of why not)
        let cases: [PortCase; 7] = [
            (&dual, &switch, Some("fc000000.usb"), Ok("port0")),
            (&dual, &switch, Some("fc400000.usb"), Err("no role switch")),
            (
                &dual,
                &switch,
                None,
                Err("can't be told from its sysfs path"),
            ),
            (
                &two,
                &switch,
                Some("fc000000.usb"),
                Err("has 2 Type-C ports"),
            ),
            (
                &[],
                &switch,
                Some("fc000000.usb"),
                Err("has no Type-C port"),
            ),
            (
                &sink,
                &switch,
                Some("fc000000.usb"),
                Err("set to take power"),
            ),
            (
                &unreadable,
                &switch,
                Some("fc000000.usb"),
                Err("reads \"dual source sink\""),
            ),
        ];
        for (ports, switches, controller, expected) in cases {
            let (typec, roles) = sysfs(&root, ports, switches);
            let found = power_port(&typec, &roles, controller).unwrap();
            match (&found, expected) {
                (PowerPort::Switchable(port), Ok(name)) => assert_eq!(port, &typec.join(name)),
                (PowerPort::Fixed(why), Err(part)) => assert!(why.contains(part), "{why}"),
                (found, expected) => panic!("{ports:?}: {found:?}, expected {expected:?}"),
            }
        }
        // No Type-C class at all.
        let gone = root.join("none");
        let (_, roles) = sysfs(&root, &dual, &switch);
        assert!(matches!(
            power_port(&gone, &roles, Some("fc000000.usb")).unwrap(),
            PowerPort::Fixed(_)
        ));
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_power_cycle_switches_the_port_off_then_back_to_what_it_was_whether_or_not_the_camera_returns()
     {
        let root = root("cycle");
        // (the port as it was, what the camera does once powered, whether it's back, after how
        // long)
        let cases = [
            (
                "[dual] source sink",
                Some(3),
                Back::Returned,
                OFF_FOR + Duration::from_millis(60),
            ),
            (
                "[source] sink",
                Some(1),
                Back::Returned,
                OFF_FOR + Duration::from_millis(20),
            ),
            (
                "[dual] source sink",
                None,
                Back::NotYet,
                OFF_FOR + POWER_RETURN,
            ),
        ];
        for (was, back_after, expected, took) in cases {
            let (typec, _) = sysfs(&root, &[("port0", was)], &[]);
            let port = typec.join("port0");
            let file = port.join("port_type");
            // What the port was set to at each pause, and how long it all took.
            let written = RefCell::new(Vec::new());
            let mut waited = Duration::ZERO;
            let mut looks = 0;
            let back = power_cycle(
                &port,
                || {
                    looks += 1;
                    match back_after {
                        Some(after) if looks >= after => Seen::Present(powered_up()),
                        _ => Seen::Absent,
                    }
                },
                |pause| {
                    written
                        .borrow_mut()
                        .push(fs::read_to_string(&file).unwrap());
                    waited += pause;
                },
            )
            .unwrap();
            assert_eq!(back, expected, "{was}");
            assert_eq!(waited, took, "{was}");
            let written = written.into_inner();
            assert_eq!(written[0], "sink", "{was}: off first");
            let restored = current(was).unwrap();
            assert!(
                written[1..].iter().all(|w| w == restored),
                "{was}: {written:?}"
            );
            assert_eq!(
                fs::read_to_string(&file).unwrap(),
                restored,
                "{was}: left on"
            );
        }
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_port_visor_may_not_write_is_refused_and_left_as_it_was() {
        use std::os::unix::fs::PermissionsExt;
        // Root writes anything: the refusal can't be staged.
        if unsafe { libc::geteuid() } == 0 {
            return;
        }
        let root = root("refused");
        let (typec, _) = sysfs(&root, &[("port0", "[dual] source sink")], &[]);
        let file = typec.join("port0/port_type");
        fs::set_permissions(&file, fs::Permissions::from_mode(0o444)).unwrap();
        let error = power_cycle(&typec.join("port0"), || Seen::Absent, |_| {}).unwrap_err();
        assert_eq!(error.kind(), io::ErrorKind::PermissionDenied);
        assert_eq!(fs::read_to_string(&file).unwrap(), "[dual] source sink\n");
        fs::set_permissions(&file, fs::Permissions::from_mode(0o644)).unwrap();
        fs::remove_dir_all(&root).unwrap();
    }
}
