//! Recovery, scenario by scenario: a camera's USB side scripted open by open (the link it's on,
//! what a reset does, what the camera does once its port is powered again), with the Type-C ports
//! and role switches in a sysfs of the test's, so the port rule and the power cycle are the real
//! ones.

use super::*;
use std::collections::VecDeque;
use std::fs;
use std::io;
use std::os::unix::fs::PermissionsExt;
use std::path::PathBuf;

use crate::typec;
use crate::usb::{Identity, Seen, Version};

/// The controller behind the Type-C port, and one behind a Type-A port.
const TYPE_C: &str = "fc000000.usb";
const TYPE_A: &str = "fc400000.usb";
const PORT_TYPE: &str = "[dual] source sink";

fn link(mbps: u32, bus: u32, address: u32, controller: &str) -> Link {
    Link {
        name: format!("{bus}-1"),
        mbps,
        version: Version {
            major: 3,
            minor: 10,
        },
        bus,
        address,
        controller: Some(controller.into()),
        identity: Identity {
            vendor: Some(0x05c8),
            product: Some(0x0a00),
            serial: Some("TC1".into()),
        },
    }
}

/// The camera fallen to USB 1's full speed, where it has no video device, behind its Type-C
/// port's controller.
fn full(address: u32) -> Link {
    link(12, 6, address, TYPE_C)
}

/// The same on a Type-A port.
fn full_on_a(address: u32) -> Link {
    link(12, 2, address, TYPE_A)
}

/// A camera that reports no serial.
fn unserialled(link: Link) -> Link {
    Link {
        identity: Identity {
            serial: None,
            ..link.identity
        },
        ..link
    }
}

/// The camera on its Type-C port, at USB 2 and at SuperSpeed; each address another enumeration.
fn slow(address: u32) -> Link {
    link(480, 7, address, TYPE_C)
}

fn fast(address: u32) -> Link {
    link(5000, 8, address, TYPE_C)
}

/// The camera on a Type-A port, at USB 2.
fn slow_on_a(address: u32) -> Link {
    link(480, 3, address, TYPE_A)
}

/// The system's Type-C ports.
#[derive(Clone, Copy)]
enum Ports {
    /// One, feeding the Type-C controller (it has a role switch).
    One,
    /// Two: which one feeds the camera can't be told.
    Two,
    /// One, whose `port_type` Visor may not write.
    ReadOnly,
}

/// What a reset or a power cycle does to the camera. Each reset goes through the real wait, with
/// the call ending as the kernel's does.
#[derive(Clone)]
enum Does {
    /// It leaves (a reset's call ends `ENODEV`) and comes back on this link with its video device.
    To(Link),
    /// The same: named for the rows written for the `ENODEV` fix.
    LeftTo(Link),
    /// It leaves and never comes back.
    LeftNever,
    /// The same.
    Never,
    /// It leaves and comes back on this link with no video device.
    Bare(Link),
    /// The reset's call returns cleanly: the kernel reset it where it is, its video device there
    /// again at once.
    InPlace,
    /// The reset's call returns cleanly, and it's where it was, still with no video device.
    InPlaceNoVideo,
    /// Visor may not reset it.
    Refused,
}

/// Where a reset or power cycle leaves the camera.
enum After {
    Video(Link),
    Bare(Link),
    Gone,
}

/// The camera at each open.
#[derive(Clone)]
enum On {
    /// On this link.
    At(Link),
    /// On USB on this link, with no video device.
    Bare(Link),
    /// Two devices on USB that report its identity, neither with a video device.
    Twins(Link, Link),
    /// Where the last open left it.
    Kept,
    /// Not there.
    Gone,
}

/// The test's USB side: the camera's link, scripted resets and power cycles, a clock moved by
/// the test and by the waits, and the Type-C ports in folders of its own.
struct Bench {
    now: Duration,
    on: Option<Link>,
    /// Devices on USB with no video device.
    bare: Vec<Link>,
    resets: VecDeque<Does>,
    cycles: VecDeque<Does>,
    typec: PathBuf,
    roles: PathBuf,
    reset_calls: u32,
    cycles_sent: u32,
}

impl Bench {
    /// Leaves the camera where `after` says.
    fn put(&mut self, after: After) {
        (self.on, self.bare) = match after {
            After::Video(link) => (Some(link), Vec::new()),
            After::Bare(link) => (None, vec![link]),
            After::Gone => (None, Vec::new()),
        };
    }
}

impl Usb for Bench {
    fn link(&mut self, _device: &Path) -> io::Result<Option<Link>> {
        Ok(self.on.clone())
    }

    fn reset(&mut self, _device: &Path, link: &Link) -> io::Result<Back> {
        self.reset_calls += 1;
        let does = self
            .resets
            .pop_front()
            .expect("a reset the scenario expects");
        let left = || Err(io::Error::from_raw_os_error(libc::ENODEV));
        let (sent, after) = match does {
            Does::To(back) | Does::LeftTo(back) => (left(), After::Video(back)),
            Does::LeftNever | Does::Never => (left(), After::Gone),
            Does::Bare(back) => (left(), After::Bare(back)),
            Does::InPlace => (Ok(()), After::Video(link.clone())),
            Does::InPlaceNoVideo => (Ok(()), After::Bare(link.clone())),
            Does::Refused => return Err(io::Error::from_raw_os_error(libc::EACCES)),
        };
        // A camera that left is gone at the first look; its video device, if it has one, is
        // there from the next. One reset in place has it at once.
        let in_place = sent.is_ok();
        let video = match &after {
            After::Video(link) => Some(link.clone()),
            After::Bare(_) | After::Gone => None,
        };
        let mut now = self.now;
        let mut looks = 0;
        let look = || {
            looks += 1;
            match (&video, in_place || looks >= 2) {
                (Some(link), true) => Seen::Present(link.clone()),
                _ => Seen::Absent,
            }
        };
        let back = crate::usb::reset_and_wait(sent, link, look, |pause| now += pause);
        self.now = now;
        self.put(after);
        back
    }

    fn find(&mut self, identity: &Identity) -> io::Result<crate::usb::Found> {
        let mut matching: Vec<Link> = self
            .bare
            .iter()
            .chain(&self.on)
            .filter(|link| link.identity == *identity)
            .cloned()
            .collect();
        let found = match (identity.findable(), matching.len()) {
            (crate::usb::Findable::No, _) | (_, 0) => crate::usb::Found::Nothing,
            (_, 1) => crate::usb::Found::One(matching.remove(0)),
            _ => crate::usb::Found::Many,
        };
        Ok(found)
    }

    fn power_port(&mut self, link: &Link) -> io::Result<PowerPort> {
        typec::power_port(&self.typec, &self.roles, link.controller.as_deref())
    }

    fn power_cycle(&mut self, _device: &Path, port: &Path) -> io::Result<Back> {
        // Looked at only once the port has been switched: a refused one sends nothing.
        let after = match self.cycles.front() {
            Some(Does::To(link) | Does::LeftTo(link)) => After::Video(link.clone()),
            Some(Does::Bare(link)) => After::Bare(link.clone()),
            Some(_) | None => After::Gone,
        };
        let back_on = match &after {
            After::Video(link) => Some(link.clone()),
            After::Bare(_) | After::Gone => None,
        };
        let mut now = self.now;
        let mut looks = 0;
        let look = || {
            looks += 1;
            // Off for the whole of the first look after power comes back, then on its link.
            match (&back_on, looks) {
                (Some(link), 2..) => crate::usb::Seen::Present(link.clone()),
                _ => crate::usb::Seen::Absent,
            }
        };
        let back = typec::power_cycle(port, look, |pause| now += pause)?;
        // The port was switched back to what it was, even when the camera didn't come back; and,
        // as the kernel does, it lists its choices again with that one bracketed.
        let file = port.join("port_type");
        assert_eq!(
            fs::read_to_string(&file)?,
            "dual",
            "the port switched back on"
        );
        fs::write(&file, PORT_TYPE)?;
        let scripted = self.cycles.pop_front();
        assert!(
            scripted.is_some(),
            "a power cycle the scenario didn't expect"
        );
        self.cycles_sent += 1;
        self.now = now;
        self.put(after);
        Ok(back)
    }

    fn now(&mut self) -> Duration {
        self.now
    }
}

/// One open: how long after the last it comes, where the camera is, what a reset and a power
/// cycle would do; then how many resets were asked for and power cycles sent, what came of them,
/// and whether the link opened on says a recovery left it slow.
struct Open {
    after_s: u64,
    on: On,
    reset: Option<Does>,
    cycle: Option<Does>,
    resets: u32,
    cycles: u32,
    outcome: Option<Outcome>,
    recovered: Recovered,
}

/// An open where nothing is sent.
fn quiet(after_s: u64, on: On, recovered: Recovered) -> Open {
    Open {
        after_s,
        on,
        reset: None,
        cycle: None,
        resets: 0,
        cycles: 0,
        outcome: None,
        recovered,
    }
}

/// An open where a recovery is sent: its reset and power cycle as scripted (each expected to be
/// asked for exactly when it's scripted), and what came of it.
fn recovers(
    after_s: u64,
    on: On,
    reset: Option<Does>,
    cycle: Option<Does>,
    outcome: Option<Outcome>,
    recovered: Recovered,
) -> Open {
    Open {
        after_s,
        on,
        resets: u32::from(reset.is_some()),
        cycles: u32::from(cycle.is_some()),
        reset,
        cycle,
        outcome,
        recovered,
    }
}

fn fast_by(on: Link, by: Tried) -> Option<Outcome> {
    Some(Outcome::Fast { on, by })
}

fn slow_by(on: Link, by: Tried) -> Option<Outcome> {
    Some(Outcome::Slow { on, by })
}

/// A sysfs of the test's with these Type-C ports, and a role switch for the Type-C controller.
fn sysfs(root: &Path, ports: Ports) -> (PathBuf, PathBuf) {
    let _ = fs::remove_dir_all(root);
    let (typec, roles) = (root.join("typec"), root.join("usb_role"));
    let count = match ports {
        Ports::One | Ports::ReadOnly => 1,
        Ports::Two => 2,
    };
    for n in 0..count {
        let port = typec.join(format!("port{n}"));
        fs::create_dir_all(&port).unwrap();
        fs::write(port.join("port_type"), PORT_TYPE).unwrap();
        if let Ports::ReadOnly = ports {
            fs::set_permissions(port.join("port_type"), fs::Permissions::from_mode(0o444)).unwrap();
        }
    }
    fs::create_dir_all(roles.join(format!("{TYPE_C}-role-switch"))).unwrap();
    (typec, roles)
}

#[test]
fn a_camera_is_reset_then_power_cycled_once_and_never_while_it_cant_be() {
    use Recovered::{NotTried, TriedStillSlow};
    use Tried::{PowerCycle, Reset, ResetAndPowerCycle};
    // Root writes a read-only file: the refused power cycle can't be staged.
    let root_user = unsafe { libc::geteuid() } == 0;
    let cases: Vec<(&str, Ports, Vec<Open>)> = vec![
        (
            "at SuperSpeed: nothing sent",
            Ports::One,
            vec![
                quiet(0, On::At(fast(2)), NotTried),
                quiet(1, On::Kept, NotTried),
            ],
        ),
        (
            "the reset brings SuperSpeed: no power cycle",
            Ports::One,
            vec![recovers(
                0,
                On::At(slow(2)),
                Some(Does::To(fast(3))),
                None,
                fast_by(fast(3), Reset),
                NotTried,
            )],
        ),
        (
            "the reset leaves it slow, its port switchable: the power cycle brings SuperSpeed",
            Ports::One,
            vec![recovers(
                0,
                On::At(slow(2)),
                Some(Does::To(slow(3))),
                Some(Does::To(fast(4))),
                fast_by(fast(4), ResetAndPowerCycle),
                NotTried,
            )],
        ),
        (
            "the reset ends with the device gone and the camera back at SuperSpeed: that's the \
             reset working, no power cycle",
            Ports::One,
            vec![recovers(
                0,
                On::At(slow(2)),
                Some(Does::LeftTo(fast(3))),
                None,
                fast_by(fast(3), Reset),
                NotTried,
            )],
        ),
        (
            "the reset ends with the device gone and the camera back slow: then one power cycle",
            Ports::One,
            vec![recovers(
                0,
                On::At(slow(2)),
                Some(Does::LeftTo(slow(3))),
                Some(Does::To(fast(4))),
                fast_by(fast(4), ResetAndPowerCycle),
                NotTried,
            )],
        ),
        (
            "the reset ends with the device gone and the camera never back: one power cycle, \
             which doesn't bring it either, then nothing more",
            Ports::One,
            vec![
                recovers(
                    0,
                    On::At(slow(2)),
                    Some(Does::LeftNever),
                    Some(Does::Never),
                    None,
                    NotTried,
                ),
                quiet(5, On::Gone, NotTried),
                quiet(30, On::Gone, NotTried),
            ],
        ),
        (
            "not back from the reset in time: the power cycle brings SuperSpeed",
            Ports::One,
            vec![recovers(
                0,
                On::At(slow(2)),
                Some(Does::Never),
                Some(Does::To(fast(4))),
                fast_by(fast(4), ResetAndPowerCycle),
                NotTried,
            )],
        ),
        (
            "the power cycle leaves it slow too: nothing further while it stays",
            Ports::One,
            vec![
                recovers(
                    0,
                    On::At(slow(2)),
                    Some(Does::To(slow(3))),
                    Some(Does::To(slow(4))),
                    slow_by(slow(4), ResetAndPowerCycle),
                    TriedStillSlow,
                ),
                quiet(1, On::Kept, TriedStillSlow),
                quiet(60, On::Kept, TriedStillSlow),
            ],
        ),
        (
            "on a Type-A port: the reset alone, quietly",
            Ports::One,
            vec![
                recovers(
                    0,
                    On::At(slow_on_a(2)),
                    Some(Does::To(slow_on_a(3))),
                    None,
                    slow_by(slow_on_a(3), Reset),
                    TriedStillSlow,
                ),
                quiet(1, On::Kept, TriedStillSlow),
            ],
        ),
        (
            "two Type-C ports: which feeds it can't be told, so the reset alone",
            Ports::Two,
            vec![recovers(
                0,
                On::At(slow(2)),
                Some(Does::To(slow(3))),
                None,
                slow_by(slow(3), Reset),
                TriedStillSlow,
            )],
        ),
        (
            "neither Visor's to send: asked once, never again, the camera opened as it is",
            Ports::ReadOnly,
            vec![
                recovers(
                    0,
                    On::At(slow(2)),
                    Some(Does::Refused),
                    None,
                    None,
                    NotTried,
                ),
                quiet(1, On::Kept, NotTried),
                quiet(1, On::At(slow(5)), NotTried),
            ],
        ),
        (
            "slow, replugged by hand: tried again on the new enumeration",
            Ports::One,
            vec![
                recovers(
                    0,
                    On::At(slow(2)),
                    Some(Does::To(slow(3))),
                    Some(Does::To(slow(4))),
                    slow_by(slow(4), ResetAndPowerCycle),
                    TriedStillSlow,
                ),
                quiet(1, On::Gone, NotTried),
                recovers(
                    2,
                    On::At(slow(9)),
                    Some(Does::To(fast(10))),
                    None,
                    fast_by(fast(10), Reset),
                    NotTried,
                ),
            ],
        ),
        (
            "promoted, then fallen to USB 2 again by itself: tried again",
            Ports::One,
            vec![
                recovers(
                    0,
                    On::At(slow(2)),
                    Some(Does::To(fast(3))),
                    None,
                    fast_by(fast(3), Reset),
                    NotTried,
                ),
                quiet(1, On::Kept, NotTried),
                recovers(
                    60,
                    On::At(slow(6)),
                    Some(Does::To(fast(7))),
                    None,
                    fast_by(fast(7), Reset),
                    NotTried,
                ),
            ],
        ),
        (
            "seen, then gone past the bound: exactly one power cycle, which brings it back",
            Ports::One,
            vec![
                quiet(0, On::At(fast(2)), NotTried),
                quiet(1, On::Gone, NotTried),
                quiet(3, On::Gone, NotTried),
                recovers(
                    2,
                    On::Gone,
                    None,
                    Some(Does::To(fast(7))),
                    fast_by(fast(7), PowerCycle),
                    NotTried,
                ),
                quiet(1, On::Kept, NotTried),
            ],
        ),
        (
            "seen, then gone, and not back after its power cycle: once, then it's found by \
             itself a minute later and is free again",
            Ports::One,
            vec![
                quiet(0, On::At(fast(2)), NotTried),
                quiet(1, On::Gone, NotTried),
                recovers(5, On::Gone, None, Some(Does::Never), None, NotTried),
                quiet(5, On::Gone, NotTried),
                quiet(30, On::Gone, NotTried),
                recovers(
                    30,
                    On::At(slow(9)),
                    Some(Does::To(slow(10))),
                    Some(Does::To(fast(11))),
                    fast_by(fast(11), ResetAndPowerCycle),
                    NotTried,
                ),
            ],
        ),
        (
            "not back from a recovery in time, then there soon after: the recovery's, not \
             tried again",
            Ports::One,
            vec![
                recovers(
                    0,
                    On::At(slow(2)),
                    Some(Does::Never),
                    Some(Does::Never),
                    None,
                    NotTried,
                ),
                quiet(2, On::At(slow(8)), TriedStillSlow),
                quiet(1, On::Kept, TriedStillSlow),
            ],
        ),
        (
            "a camera gone past the bound on a Type-A port: nothing to switch, said once",
            Ports::One,
            vec![
                quiet(0, On::At(link(5000, 4, 2, TYPE_A)), NotTried),
                quiet(0, On::Gone, NotTried),
                quiet(6, On::Gone, NotTried),
                quiet(6, On::Gone, NotTried),
            ],
        ),
        (
            "reset in place on a fixed port, still slow: still on USB 2 after the reset at once, \
             never a wait and never \"not back\"",
            Ports::One,
            vec![
                recovers(
                    0,
                    On::At(slow_on_a(2)),
                    Some(Does::InPlace),
                    None,
                    slow_by(slow_on_a(2), Reset),
                    TriedStillSlow,
                ),
                quiet(1, On::Kept, TriedStillSlow),
            ],
        ),
        (
            "reset in place on a switchable port, still slow: then one power cycle",
            Ports::One,
            vec![recovers(
                0,
                On::At(slow(2)),
                Some(Does::InPlace),
                Some(Does::To(fast(4))),
                fast_by(fast(4), ResetAndPowerCycle),
                NotTried,
            )],
        ),
        (
            "seen, then on USB at 12 Mb/s with no video device: reset at once, back at \
             SuperSpeed",
            Ports::One,
            vec![
                quiet(0, On::At(fast(2)), NotTried),
                recovers(
                    1,
                    On::Bare(full(3)),
                    Some(Does::LeftTo(fast(4))),
                    None,
                    fast_by(fast(4), Reset),
                    NotTried,
                ),
            ],
        ),
        (
            "seen, then at 12 Mb/s with no video on a fixed port, and the reset leaves it there: \
             said once, nothing more, however long",
            Ports::One,
            vec![
                quiet(0, On::At(link(5000, 4, 2, TYPE_A)), NotTried),
                recovers(
                    1,
                    On::Bare(full_on_a(3)),
                    Some(Does::InPlaceNoVideo),
                    None,
                    None,
                    NotTried,
                ),
                quiet(1, On::Kept, NotTried),
                quiet(10, On::Kept, NotTried),
            ],
        ),
        (
            "seen, then at 12 Mb/s with no video on a switchable port, the reset leaving it \
             there: one power cycle, which brings SuperSpeed",
            Ports::One,
            vec![
                quiet(0, On::At(fast(2)), NotTried),
                recovers(
                    1,
                    On::Bare(full(3)),
                    Some(Does::InPlaceNoVideo),
                    Some(Does::To(fast(5))),
                    fast_by(fast(5), ResetAndPowerCycle),
                    NotTried,
                ),
            ],
        ),
        (
            "at 12 Mb/s with no video after its reset and its power cycle: nothing more",
            Ports::One,
            vec![
                quiet(0, On::At(fast(2)), NotTried),
                recovers(
                    1,
                    On::Bare(full(3)),
                    Some(Does::InPlaceNoVideo),
                    Some(Does::Bare(full(6))),
                    None,
                    NotTried,
                ),
                quiet(1, On::Kept, NotTried),
                quiet(30, On::Kept, NotTried),
            ],
        ),
        (
            "at 12 Mb/s with no video, but no serial was recorded: it can't be told from another \
             camera, so nothing is sent",
            Ports::One,
            vec![
                quiet(0, On::At(unserialled(link(5000, 4, 2, TYPE_A))), NotTried),
                quiet(1, On::Bare(unserialled(full_on_a(3))), NotTried),
                quiet(1, On::Kept, NotTried),
            ],
        ),
        (
            "at 12 Mb/s with no video, and two devices report its serial: nothing is sent",
            Ports::One,
            vec![
                quiet(0, On::At(fast(2)), NotTried),
                quiet(1, On::Twins(full(3), full(4)), NotTried),
                quiet(1, On::Kept, NotTried),
            ],
        ),
        (
            "a camera never seen in this run: no power cycle, however long it's gone",
            Ports::One,
            vec![
                quiet(0, On::Gone, NotTried),
                quiet(6, On::Gone, NotTried),
                quiet(60, On::Gone, NotTried),
            ],
        ),
    ];
    let root = std::env::temp_dir().join(format!("recovery-{}", std::process::id()));
    for (name, ports, opens) in cases {
        if root_user && matches!(ports, Ports::ReadOnly) {
            continue;
        }
        let (typec, roles) = sysfs(&root, ports);
        let mut usb = Bench {
            now: Duration::ZERO,
            on: None,
            bare: Vec::new(),
            resets: VecDeque::new(),
            cycles: VecDeque::new(),
            typec: typec.clone(),
            roles,
            reset_calls: 0,
            cycles_sent: 0,
        };
        let mut recovery = Recovery::default();
        for (at, open) in opens.into_iter().enumerate() {
            usb.now += Duration::from_secs(open.after_s);
            match open.on {
                On::At(link) => usb.put(After::Video(link)),
                On::Bare(link) => usb.put(After::Bare(link)),
                On::Twins(one, other) => {
                    usb.on = None;
                    usb.bare = vec![one, other];
                }
                On::Gone => usb.put(After::Gone),
                On::Kept => {}
            }
            usb.resets.extend(open.reset);
            usb.cycles.extend(open.cycle);
            let (resets, cycles) = (usb.reset_calls, usb.cycles_sent);
            let opening = recovery.before_open(&mut usb, "camera", Path::new("/dev/camera"));
            let row = format!("{name}, open {at}");
            assert_eq!(usb.reset_calls - resets, open.resets, "{row}: resets");
            assert_eq!(usb.cycles_sent - cycles, open.cycles, "{row}: power cycles");
            assert!(
                usb.resets.is_empty() && usb.cycles.is_empty(),
                "{row}: all used"
            );
            assert_eq!(opening.outcome, open.outcome, "{row}");
            assert_eq!(opening.link, usb.on, "{row}: the link it's opened on");
            let recovered = opening
                .link
                .as_ref()
                .map_or(Recovered::NotTried, |link| recovery.recovered(link));
            assert_eq!(recovered, open.recovered, "{row}");
        }
        // Whatever happened, every port is as it was, never left switched off.
        for n in 0..2 {
            let file = typec.join(format!("port{n}/port_type"));
            if let Ok(text) = fs::read_to_string(&file) {
                assert_eq!(text, PORT_TYPE, "{name}: port{n}");
            }
        }
    }
    let _ = fs::remove_dir_all(&root);
}
