//! The USB link a camera is on, read from sysfs, and the one thing Visor does to it: a reset.
//!
//! A USB 3 camera whose USB 3 contacts lose touch for a moment (a loose plug, a jostled board)
//! falls back to USB 2 and stays there: it never tries USB 3 again by itself, and the host has no
//! way to make it. A USB reset does: with the wiring whole, the camera comes back at SuperSpeed a
//! fraction of a second later, on another bus, as a new video device behind the same udev name.
//! With its USB 3 contacts apart, it comes back at USB 2 again. Nothing the host can read tells
//! the two apart beforehand, so the only test is the attempt ([`crate::recovery`] decides when).
//!
//! As observed on a board, a reset that works usually ends in `ENODEV`: the camera leaves the USB 2
//! bus for the USB 3 one while the reset runs, so the device it was sent to is gone before the call
//! returns, and the camera is at SuperSpeed 0.13 to 0.24 s later. So a reset that ends with the
//! device gone ([`left_during`]) was sent, and the camera is waited for just as after one that
//! ends cleanly.
//!
//! A camera's USB device is its video device's `device/..` in sysfs, with its `speed` (Mb/s) and
//! its `version` (the USB version it reports it can do: ` 3.10` for a USB 3 camera, whatever it's
//! linked at now). The reset is `USBDEVFS_RESET` on its usbfs node, `/dev/bus/usb/BBB/DDD`, which
//! needs write access to it: the release's udev rule gives the `video` group that for USB devices
//! with a video interface (deploy/70-visor-usb-cameras.rules). A camera that stops answering
//! altogether is powered off and on instead, through its Type-C port ([`crate::typec`]).

use std::fmt;
use std::fs;
use std::io;
use std::os::fd::AsRawFd;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// SuperSpeed, USB 3's first: the speed a USB 3 camera links at with its wiring whole.
pub const SUPER_SPEED_MBPS: u32 = 5000;
/// Where the kernel puts the usbfs nodes.
pub const USBFS: &str = "/dev/bus/usb";
/// `USBDEVFS_RESET`, `_IO('U', 20)`.
const USBDEVFS_RESET: u64 = 0x5514;
/// How often a reset or powered-up camera is looked for.
const LOOK_EVERY: Duration = Duration::from_millis(20);
/// How long a reset camera is looked for at most. It comes back in 0.13 to 0.24 s at SuperSpeed
/// and in about 0.4 s at USB 2; past 3 s something else is wrong, and the capture loop's own
/// retries take over.
pub const RESET_RETURN: Duration = Duration::from_secs(3);

/// A camera's USB device, as it's linked now.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Link {
    /// The kernel's name for it, its bus and port path, like `8-1`.
    pub name: String,
    /// The speed it's linked at, in Mb/s: 12, 480, 5000 and up.
    pub mbps: u32,
    /// The USB version it reports it can do.
    pub version: Version,
    /// Which enumeration this is: its bus and address. Both change when it goes away and comes
    /// back.
    pub bus: u32,
    pub address: u32,
    /// The USB controller it hangs from, as its sysfs path names it (the last part ending
    /// `.usb`, like `fc000000.usb`); none when its path has no such part.
    pub controller: Option<String>,
}

/// A USB version as a device reports it, `bcdUSB`: 2.00, 2.10, 3.10.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Version {
    pub major: u8,
    pub minor: u8,
}

/// Whether a camera's link could be faster than it is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outlook {
    /// It's at SuperSpeed or faster.
    Fast,
    /// It's below SuperSpeed and reports USB 2.10 or later: likely a USB 3 device on USB 2. A
    /// USB 2.1 device says the same, so this is a guess that costs one wasted reset if wrong.
    MayDoBetter,
    /// It's below SuperSpeed and reports an older USB 2: a USB 2 device, at its best.
    AtItsBest,
}

impl Link {
    /// Whether this link could be faster: below SuperSpeed while the device reports USB 2.10 or
    /// later. A USB 3 device reports 3.x; 2.10 is the version a USB 2 device reports when it has a
    /// BOS descriptor, as every USB 3 device does, and is taken as able to do better rather than
    /// reading that descriptor by hand.
    pub fn outlook(&self) -> Outlook {
        if self.mbps >= SUPER_SPEED_MBPS {
            return Outlook::Fast;
        }
        let usb_2_1 = Version {
            major: 2,
            minor: 10,
        };
        if self.version >= usb_2_1 {
            Outlook::MayDoBetter
        } else {
            Outlook::AtItsBest
        }
    }

    /// Whether this is the same enumeration as `other`: neither has gone away and come back
    /// between them.
    pub fn same_enumeration(&self, other: &Link) -> bool {
        (self.name.as_str(), self.bus, self.address)
            == (other.name.as_str(), other.bus, other.address)
    }

    /// Its usbfs node under `usbfs`: `BBB/DDD`.
    pub fn node(&self, usbfs: &Path) -> PathBuf {
        usbfs.join(format!("{:03}/{:03}", self.bus, self.address))
    }
}

impl fmt::Display for Link {
    /// As the log says it: `USB 480 Mb/s (a USB 3.10 device, 7-1)`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "USB {} Mb/s (a USB {} device, {})",
            self.mbps, self.version, self.name
        )
    }
}

impl fmt::Display for Version {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{:02}", self.major, self.minor)
    }
}

/// Reads a version as sysfs gives it, ` 3.10` with a newline.
fn parse_version(text: &str) -> Option<Version> {
    let (major, minor) = text.trim().split_once('.')?;
    Some(Version {
        major: major.parse().ok()?,
        minor: minor.parse().ok()?,
    })
}

/// The USB link of the camera at `device` (a udev name or the video device itself), from the
/// kernel's description of its video devices under `sysfs` (`/sys/class/video4linux`). None when
/// the device isn't there, or isn't on USB (no `speed` beside it).
pub fn link_of(sysfs: &Path, device: &Path) -> io::Result<Option<Link>> {
    let video = match fs::canonicalize(device) {
        Ok(video) => video,
        Err(error) if gone(&error) => return Ok(None),
        Err(error) => return Err(error),
    };
    let Some(video) = video.file_name() else {
        return Ok(None);
    };
    let usb = match fs::canonicalize(sysfs.join(video).join("device/..")) {
        Ok(usb) => usb,
        Err(error) if gone(&error) => return Ok(None),
        Err(error) => return Err(error),
    };
    let Some(speed) = read_field(&usb, "speed")? else {
        return Ok(None);
    };
    let read = |field: &str| -> io::Result<String> {
        read_field(&usb, field)?
            .ok_or_else(|| invalid(format!("{} has a speed but no {field}", usb.display())))
    };
    let number = |field: &str| -> io::Result<u32> {
        let text = read(field)?;
        text.parse()
            .map_err(|_| invalid(format!("{}/{field} is {text:?}", usb.display())))
    };
    let version = read("version")?;
    let controller = usb
        .iter()
        .filter_map(|part| part.to_str())
        .rfind(|part| part.ends_with(".usb"))
        .map(str::to_string);
    Ok(Some(Link {
        controller,
        name: usb
            .file_name()
            .map_or(String::new(), |n| n.to_string_lossy().into_owned()),
        mbps: speed
            .parse::<f64>()
            .map_err(|_| invalid(format!("{}/speed is {speed:?}", usb.display())))?
            as u32,
        version: parse_version(&version)
            .ok_or_else(|| invalid(format!("{}/version is {version:?}", usb.display())))?,
        bus: number("busnum")?,
        address: number("devnum")?,
    }))
}

/// A sysfs field, trimmed; none when it isn't there.
fn read_field(folder: &Path, field: &str) -> io::Result<Option<String>> {
    match fs::read_to_string(folder.join(field)) {
        Ok(text) => Ok(Some(text.trim().to_string())),
        Err(error) if gone(&error) => Ok(None),
        Err(error) => Err(error),
    }
}

/// Whether a sysfs read failed because the device is gone, or going: not there, not an error.
fn gone(error: &io::Error) -> bool {
    error.kind() == io::ErrorKind::NotFound || error.raw_os_error() == Some(libc::ENODEV)
}

fn invalid(why: String) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, why)
}

/// Where a camera's USB link is read, and its reset and power cycle sent: the system's, or a
/// test's. Its clock is the one recovery's bounds are measured on.
pub trait Usb {
    /// The USB link of the camera at `device`, now; none when it isn't there or isn't on USB.
    fn link(&mut self, device: &Path) -> io::Result<Option<Link>>;
    /// Resets the USB device behind `link`, then waits a bounded time for the camera at `device`
    /// to come back. An error when the reset can't be sent: [`io::ErrorKind::PermissionDenied`]
    /// when Visor may not.
    fn reset(&mut self, device: &Path, link: &Link) -> io::Result<Back>;
    /// The port whose power can be switched for a camera last on `link`, or why there is none.
    fn power_port(&mut self, link: &Link) -> io::Result<PowerPort>;
    /// Powers `port` off and on again ([`crate::typec::power_cycle`]), then waits a bounded time
    /// for the camera at `device` to come back. [`io::ErrorKind::PermissionDenied`] when Visor
    /// may not switch it.
    fn power_cycle(&mut self, device: &Path, port: &Path) -> io::Result<Back>;
    /// Time on a monotonic clock.
    fn now(&mut self) -> Duration;
}

/// Whether a camera's port can be powered off and on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PowerPort {
    /// Through this Type-C port's folder (`/sys/class/typec/portN`).
    Switchable(PathBuf),
    /// It can't, and why, as the log says it.
    Fixed(String),
}

/// Whether a reset camera came back within the wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Back {
    Returned,
    NotYet,
}

/// The system's USB devices: links from sysfs, resets through usbfs, power through the Type-C
/// class.
#[derive(Debug, Clone)]
pub struct SystemUsb {
    /// `/sys/class/video4linux`.
    pub sysfs: PathBuf,
    /// `/dev/bus/usb`.
    pub usbfs: PathBuf,
    /// `/sys/class/typec`.
    pub typec: PathBuf,
    /// `/sys/class/usb_role`.
    pub roles: PathBuf,
    started: Instant,
}

impl Default for SystemUsb {
    fn default() -> SystemUsb {
        SystemUsb {
            sysfs: PathBuf::from(crate::cameras::SYSFS),
            usbfs: PathBuf::from(USBFS),
            typec: PathBuf::from(crate::typec::TYPEC),
            roles: PathBuf::from(crate::typec::ROLES),
            started: Instant::now(),
        }
    }
}

impl SystemUsb {
    /// A look for the camera at `device`, as a wait for it makes one.
    fn look(&self, device: &Path) -> Seen {
        match link_of(&self.sysfs, device) {
            Ok(Some(now)) => Seen::Present(now),
            // Unreadable counts as not back yet: the wait is bounded.
            Ok(None) | Err(_) => Seen::Absent,
        }
    }
}

impl Usb for SystemUsb {
    fn link(&mut self, device: &Path) -> io::Result<Option<Link>> {
        link_of(&self.sysfs, device)
    }

    fn reset(&mut self, device: &Path, link: &Link) -> io::Result<Back> {
        let sent = send_reset(&link.node(&self.usbfs));
        let look = || self.look(device);
        reset_and_wait(sent, link, look, std::thread::sleep)
    }

    fn power_port(&mut self, link: &Link) -> io::Result<PowerPort> {
        crate::typec::power_port(&self.typec, &self.roles, link.controller.as_deref())
    }

    fn power_cycle(&mut self, device: &Path, port: &Path) -> io::Result<Back> {
        let look = || self.look(device);
        crate::typec::power_cycle(port, look, std::thread::sleep)
    }

    fn now(&mut self) -> Duration {
        self.started.elapsed()
    }
}

/// Sends `USBDEVFS_RESET` to the usbfs node `node`. The node must be opened for writing.
fn send_reset(node: &Path) -> io::Result<()> {
    let file = fs::OpenOptions::new().write(true).open(node)?;
    // SAFETY: USBDEVFS_RESET takes no argument, and the descriptor is open for the call.
    let result = unsafe { libc::ioctl(file.as_raw_fd(), USBDEVFS_RESET as _, 0) };
    if result < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

/// Whether an error from talking to a USB device means it went away while Visor talked to it: its
/// node gone (`ENOENT`), the device gone (`ENODEV`), or the transfer cut off by it going (`EIO`).
pub fn left_during(error: &io::Error) -> bool {
    matches!(
        error.raw_os_error(),
        Some(libc::ENODEV | libc::ENOENT | libc::EIO)
    )
}

/// After a reset whose sending came to `sent`, waits for the camera that was on `before` to come
/// back ([`await_return`], with `look` and `pause` as its). A reset that ended with the device
/// gone ([`left_during`]) is how one that works usually ends, so it's waited for the same as one
/// that ended cleanly; any other error is the reset's, and nothing is waited for.
pub fn reset_and_wait(
    sent: io::Result<()>,
    before: &Link,
    look: impl FnMut() -> Seen,
    pause: impl FnMut(Duration),
) -> io::Result<Back> {
    match sent {
        Ok(()) => {}
        Err(error) if left_during(&error) => {}
        Err(error) => return Err(error),
    }
    Ok(await_return(Some(before), RESET_RETURN, look, pause))
}

/// What a look for a reset camera finds.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Seen {
    /// Its name leads nowhere: gone, or not back yet.
    Absent,
    /// Its name leads to a camera on this link.
    Present(Link),
}

/// Waits for a camera that was on `before` (none: known to be gone, powered off) to come back:
/// `look` each [`LOOK_EVERY`] (after `pause`), for at most `limit`. It's back once it has been
/// seen gone and then present, or present as another enumeration (gone and back between two
/// looks). Time is counted in pauses, so a test's `pause` needn't sleep.
pub fn await_return(
    before: Option<&Link>,
    limit: Duration,
    mut look: impl FnMut() -> Seen,
    mut pause: impl FnMut(Duration),
) -> Back {
    let mut gone = before.is_none();
    let mut waited = Duration::ZERO;
    while waited < limit {
        pause(LOOK_EVERY);
        waited += LOOK_EVERY;
        match (look(), before) {
            (Seen::Absent, _) => gone = true,
            (Seen::Present(_), _) if gone => return Back::Returned,
            (Seen::Present(now), Some(before)) if !now.same_enumeration(before) => {
                return Back::Returned;
            }
            (Seen::Present(_), _) => {}
        }
    }
    Back::NotYet
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::fs::symlink;

    fn link(name: &str, mbps: u32, version: (u8, u8), bus: u32, address: u32) -> Link {
        Link {
            name: name.into(),
            mbps,
            version: Version {
                major: version.0,
                minor: version.1,
            },
            bus,
            address,
            controller: Some("fc000000.usb".into()),
        }
    }

    #[test]
    fn a_link_below_super_speed_may_do_better_when_its_device_reports_usb_2_1_or_later() {
        let cases = [
            ("USB 3 at SuperSpeed", 5000, (3, 10), Outlook::Fast),
            ("USB 3 at SuperSpeed+", 10000, (3, 20), Outlook::Fast),
            ("USB 3 fallen to USB 2", 480, (3, 10), Outlook::MayDoBetter),
            (
                "USB 3 fallen to full speed",
                12,
                (3, 10),
                Outlook::MayDoBetter,
            ),
            (
                "USB 3 reporting 2.10 at USB 2",
                480,
                (2, 10),
                Outlook::MayDoBetter,
            ),
            ("a USB 2.0 device", 480, (2, 0), Outlook::AtItsBest),
            ("a USB 2.01 device", 480, (2, 1), Outlook::AtItsBest),
            ("a USB 1.1 device", 12, (1, 10), Outlook::AtItsBest),
        ];
        for (name, mbps, version, expected) in cases {
            assert_eq!(
                link("7-1", mbps, version, 7, 2).outlook(),
                expected,
                "{name}"
            );
        }
    }

    #[test]
    fn a_link_is_said_with_its_speed_version_and_name_and_reset_through_its_node() {
        let fallen = link("7-1", 480, (3, 10), 7, 2);
        assert_eq!(fallen.to_string(), "USB 480 Mb/s (a USB 3.10 device, 7-1)");
        assert_eq!(
            fallen.node(Path::new("/dev/bus/usb")),
            Path::new("/dev/bus/usb/007/002")
        );
    }

    /// A sysfs of the test's: `video0` on a USB device with the given fields, and a name for it.
    fn sysfs(root: &Path, fields: &[(&str, &str)]) -> (PathBuf, PathBuf) {
        let _ = fs::remove_dir_all(root);
        let usb = root.join("platform/fc000000.usb/xhci-hcd.0.auto/usb8/8-1");
        let interface = usb.join("8-1:1.0");
        fs::create_dir_all(&interface).unwrap();
        for (field, value) in fields {
            fs::write(usb.join(field), format!("{value}\n")).unwrap();
        }
        let class = root.join("class");
        fs::create_dir_all(class.join("video0")).unwrap();
        symlink(&interface, class.join("video0/device")).unwrap();
        let dev = root.join("dev");
        fs::create_dir_all(&dev).unwrap();
        fs::write(dev.join("video0"), "").unwrap();
        let name = dev.join("by-id-camera-video-index0");
        symlink(dev.join("video0"), &name).unwrap();
        (class, name)
    }

    /// A row: a USB device's fields, and the link read or part of the error.
    type FieldsCase<'a> = (&'a [(&'a str, &'a str)], Result<Option<Link>, &'a str>);
    /// A row: its name, the link before, what each look sees, whether it's back, after how many
    /// looks.
    type WaitCase<'a> = (&'a str, Option<&'a Link>, Vec<Seen>, Back, u32);

    #[test]
    fn a_link_is_read_from_the_usb_device_behind_a_video_device() {
        let root = std::env::temp_dir().join(format!("usb-{}", std::process::id()));
        let usb_3 = [
            ("speed", "5000"),
            ("version", " 3.10"),
            ("busnum", "8"),
            ("devnum", "3"),
        ];
        let usb_2 = [
            ("speed", "480"),
            ("version", " 3.10"),
            ("busnum", "7"),
            ("devnum", "12"),
        ];
        let full_speed = [
            ("speed", "12"),
            ("version", " 2.00"),
            ("busnum", "6"),
            ("devnum", "2"),
        ];
        let no_speed = [("version", " 3.10")];
        let no_bus = [("speed", "480"), ("version", " 3.10")];
        let bad_version = [
            ("speed", "480"),
            ("version", "three"),
            ("busnum", "7"),
            ("devnum", "2"),
        ];
        // (the fields, the link read, or part of the error); its controller is the path's
        // fc000000.usb.
        let cases: [FieldsCase; 6] = [
            (&usb_3, Ok(Some(link("8-1", 5000, (3, 10), 8, 3)))),
            (&usb_2, Ok(Some(link("8-1", 480, (3, 10), 7, 12)))),
            (&full_speed, Ok(Some(link("8-1", 12, (2, 0), 6, 2)))),
            (&no_speed, Ok(None)),
            (&bad_version, Err("version is \"three\"")),
            (&no_bus, Err("has a speed but no busnum")),
        ];
        for (fields, expected) in cases {
            let (class, name) = sysfs(&root, fields);
            match (link_of(&class, &name), expected) {
                (Ok(read), Ok(link)) => assert_eq!(read, link, "{fields:?}"),
                (Err(error), Err(part)) => {
                    assert!(error.to_string().contains(part), "{error}")
                }
                (read, expected) => panic!("{fields:?}: {read:?}, expected {expected:?}"),
            }
        }
        // A name that leads nowhere: unplugged.
        let (class, _) = sysfs(&root, &usb_3);
        assert_eq!(link_of(&class, &root.join("dev/gone")).unwrap(), None);
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_reset_that_ends_with_the_device_gone_was_sent_and_its_camera_is_waited_for() {
        let before = link("7-1", 480, (3, 10), 7, 2);
        let back = || Seen::Present(link("8-1", 5000, (3, 10), 8, 4));
        // (how the reset's sending ended, whether it's waited for and back, or the error's kind)
        let cases: [(io::Error, Result<Back, io::ErrorKind>); 5] = [
            (
                io::Error::from_raw_os_error(libc::ENODEV),
                Ok(Back::Returned),
            ),
            (
                io::Error::from_raw_os_error(libc::ENOENT),
                Ok(Back::Returned),
            ),
            (io::Error::from_raw_os_error(libc::EIO), Ok(Back::Returned)),
            (
                io::Error::from_raw_os_error(libc::EACCES),
                Err(io::ErrorKind::PermissionDenied),
            ),
            (
                io::Error::from_raw_os_error(libc::EBUSY),
                Err(io::ErrorKind::ResourceBusy),
            ),
        ];
        for (error, expected) in cases {
            let name = error.to_string();
            let waited = reset_and_wait(Err(error), &before, back, |_| {});
            match (waited, expected) {
                (Ok(got), Ok(back)) => assert_eq!(got, back, "{name}"),
                (Err(got), Err(kind)) => assert_eq!(got.kind(), kind, "{name}"),
                (got, expected) => panic!("{name}: {got:?}, expected {expected:?}"),
            }
        }
        assert_eq!(
            reset_and_wait(Ok(()), &before, back, |_| {}).unwrap(),
            Back::Returned
        );
    }

    #[test]
    fn a_reset_camera_is_back_once_it_has_gone_and_returned_or_returned_as_another() {
        let before = link("7-1", 480, (3, 10), 7, 2);
        let before = &before;
        let super_speed = Seen::Present(link("8-1", 5000, (3, 10), 8, 4));
        let same_again = Seen::Present((*before).clone());
        let a = Seen::Absent;
        // (what it was on before (none: powered off), what each look sees, then the same for good;
        // whether it's back, after how many looks)
        let cases: [WaitCase; 7] = [
            (
                "gone, then back at SuperSpeed",
                Some(before),
                vec![
                    same_again.clone(),
                    a.clone(),
                    a.clone(),
                    super_speed.clone(),
                ],
                Back::Returned,
                4,
            ),
            (
                "gone, then back on the same bus at the same address",
                Some(before),
                vec![a.clone(), a.clone(), same_again.clone()],
                Back::Returned,
                3,
            ),
            (
                "back as another enumeration between two looks",
                Some(before),
                vec![super_speed.clone()],
                Back::Returned,
                1,
            ),
            (
                "never gone: waited out",
                Some(before),
                vec![same_again.clone()],
                Back::NotYet,
                150,
            ),
            (
                "gone for good: waited out",
                Some(before),
                vec![a.clone()],
                Back::NotYet,
                150,
            ),
            (
                "powered off, then back as it was: any camera after is back",
                None,
                vec![a.clone(), a.clone(), same_again.clone()],
                Back::Returned,
                3,
            ),
            (
                "powered off, never back",
                None,
                vec![a.clone()],
                Back::NotYet,
                150,
            ),
        ];
        for (name, before, script, expected, looks) in cases {
            let mut script = std::collections::VecDeque::from(script);
            let mut looked = 0;
            let mut paused = Duration::ZERO;
            let back = await_return(
                before,
                RESET_RETURN,
                || {
                    looked += 1;
                    if script.len() > 1 {
                        script.pop_front().unwrap()
                    } else {
                        script[0].clone()
                    }
                },
                |pause| paused += pause,
            );
            assert_eq!(back, expected, "{name}");
            assert_eq!(looked, looks, "{name}");
            assert_eq!(paused, LOOK_EVERY * looks, "{name}");
        }
    }
}
