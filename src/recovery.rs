//! Getting a camera back to its best link when it costs nothing: a USB 3 camera stuck at USB 2 or
//! at USB 1 speed, or one that stopped answering. Two means, each sent at most once
//! ([`crate::usb`] and [`crate::typec`] say why each works):
//!
//! - A camera linked below SuperSpeed that looks able to do better, on a port whose power Visor
//!   can switch, gets one USB reset. If it's still not at SuperSpeed after (still slow, or not
//!   back within the wait), one power cycle of that port. On any other port it's left as it is,
//!   running on the link it has: a reset left a camera not answering twice in about forty on a
//!   bench, and only cutting its power brought it back, so Visor doesn't risk a camera that
//!   works where it couldn't undo that.
//! - A camera seen in this run that is on USB below SuperSpeed with no video device (at USB 1's
//!   full speed, where uvcvideo finds nothing it can stream) gets the reset at once, whatever its
//!   port (it has no video to lose), then the power cycle where that can be sent: it's found by
//!   its vendor, product and serial ([`crate::usb::find_devices`]), only when it has a serial and
//!   exactly one device has it.
//! - A camera seen in this run that has been gone for [`GONE_FOR`] gets one power cycle of the port
//!   it was last on: the hung camera.
//!
//! Never while it streams: only in the capture loop's open path, as Visor starts and when it opens
//! again a camera that went away anyway. Once, not in a loop: after a recovery that left it below
//! SuperSpeed or gone, nothing more is tried until the camera comes back by itself (an enumeration
//! Visor didn't cause) or Visor restarts. A recovery Visor may not send (no permission, or a port
//! that can't be switched) is said once, and is never an error for the camera.

mod outcome;

use std::path::{Path, PathBuf};
use std::time::Duration;

use log::{info, warn};

pub use outcome::{Outcome, Recovered, Tried, say};

use crate::settings::Said;
use crate::usb::{Back, Found, Link, Outlook, PowerPort, SUPER_SPEED_MBPS, Usb};

/// How long a camera seen in this run must have been gone before its port is power-cycled. Longer
/// than every ordinary disappearance: a fall to USB 2 is gone 0.6 s, a reset 0.4 s, and a replug
/// by hand a few seconds; the hung camera was gone over 30 s.
pub const GONE_FOR: Duration = Duration::from_secs(5);
/// A camera not back by the end of a recovery's wait that appears within this of it is taken as
/// that recovery's, not as back by itself.
const LATE_RETURN: Duration = Duration::from_secs(10);

/// Where a camera stands with recovery, across the capture loop's reopens.
#[derive(Debug, Default)]
pub struct Recovery {
    standing: Standing,
    reset: Allowed,
    power: Allowed,
    /// The camera's link when it was last seen in this run.
    last_seen: Option<Link>,
    absence: Absence,
    said: Said,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
enum Standing {
    /// A recovery may be tried.
    #[default]
    Free,
    /// One was, at `at`, and the camera stayed below SuperSpeed (on `after`) or wasn't back (no
    /// `after` yet). Nothing more for this enumeration of it.
    Spent { after: Option<Link>, at: Duration },
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
enum Allowed {
    #[default]
    Yes,
    /// Visor may not: said once, never tried again.
    Refused,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
enum Absence {
    /// It was there at the last open, or hasn't been looked for.
    #[default]
    Present,
    /// It has been gone since then.
    GoneSince(Duration),
    /// It has been gone past [`GONE_FOR`], and that was dealt with.
    Handled,
}

/// Where a camera is, as a recovery finds it.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Where {
    /// With its video device, on this link.
    Video(Link),
    /// On USB on this link, with no video device.
    Bare(Link),
    /// Nowhere Visor can find it.
    Nowhere,
}

/// What [`Recovery::before_open`] leaves: the link to open the camera on (none when it can't be
/// read), and what came of a recovery, if one was sent and the camera is back.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Opening {
    pub link: Option<Link>,
    pub outcome: Option<Outcome>,
}

impl Opening {
    fn on(link: Option<Link>) -> Opening {
        Opening {
            link,
            outcome: None,
        }
    }
}

impl Recovery {
    /// Before the camera at `device` is opened: reads its link and, when a recovery is due and
    /// allowed, sends it and waits for the camera.
    pub fn before_open(&mut self, usb: &mut impl Usb, name: &str, device: &Path) -> Opening {
        let now = usb.now();
        match self.read(usb, name, device) {
            Some(link) => self.present(usb, name, device, link, now),
            None => self.absent(usb, name, device, now),
        }
    }

    /// Whether a recovery was tried and left the camera, now on `link`, below SuperSpeed.
    pub fn recovered(&self, link: &Link) -> Recovered {
        match &self.standing {
            Standing::Spent {
                after: Some(after), ..
            } if after.same_enumeration(link) && link.outlook() != Outlook::Fast => {
                Recovered::TriedStillSlow
            }
            _ => Recovered::NotTried,
        }
    }

    fn read(&mut self, usb: &mut impl Usb, name: &str, device: &Path) -> Option<Link> {
        match usb.link(device) {
            Ok(link) => link,
            Err(why) => {
                self.warn_once(&format!(
                    "{name}: its USB link can't be read ({why}), so it isn't checked for USB 3"
                ));
                None
            }
        }
    }

    fn present(
        &mut self,
        usb: &mut impl Usb,
        name: &str,
        device: &Path,
        link: Link,
        now: Duration,
    ) -> Opening {
        self.absence = Absence::Present;
        self.notice(&link, now);
        self.last_seen = Some(link.clone());
        if link.outlook() != Outlook::MayDoBetter || self.standing != Standing::Free {
            return Opening::on(Some(link));
        }
        self.recover(usb, name, device, Where::Video(link))
    }

    /// A camera a recovery didn't bring back in time is taken as that recovery's when it appears
    /// soon after; one that went away and came back by itself since is free again.
    fn notice(&mut self, link: &Link, now: Duration) {
        let Standing::Spent { after, at } = &self.standing else {
            return;
        };
        let at = *at;
        match after {
            None if now.saturating_sub(at) <= LATE_RETURN => {
                self.standing = Standing::Spent {
                    after: Some(link.clone()),
                    at,
                };
            }
            Some(after) if after.same_enumeration(link) => {}
            _ => self.standing = Standing::Free,
        }
    }

    fn absent(&mut self, usb: &mut impl Usb, name: &str, device: &Path, now: Duration) -> Opening {
        if let Some(bare) = self.bare(usb, name) {
            return self.present_without_video(usb, name, device, bare, now);
        }
        let since = match self.absence {
            Absence::Present => {
                self.absence = Absence::GoneSince(now);
                now
            }
            Absence::GoneSince(since) => since,
            Absence::Handled => return Opening::on(None),
        };
        if now.saturating_sub(since) < GONE_FOR {
            return Opening::on(None);
        }
        self.absence = Absence::Handled;
        // Never seen in this run, nothing says it's plugged in at all; and after a recovery, once
        // is all.
        let Some(last) = self.last_seen.clone() else {
            return Opening::on(None);
        };
        if self.standing != Standing::Free {
            return Opening::on(None);
        }
        let why = format!("gone {} s", GONE_FOR.as_secs());
        let gone = Where::Nowhere;
        match self.power_cycle(usb, name, device, &gone, &why) {
            Some(back) => {
                let found = self.locate(usb, name, device, back, &Where::Bare(last));
                self.settle(usb, name, found, Tried::PowerCycle)
            }
            None => Opening::on(None),
        }
    }

    /// The camera seen earlier in this run, on USB below SuperSpeed with no video device: found
    /// by what it reported when last seen, when that includes a serial and one device has it.
    fn bare(&mut self, usb: &mut impl Usb, name: &str) -> Option<Link> {
        let last = self.last_seen.clone()?;
        let found = self.find(usb, name, &last)?;
        (found.mbps < SUPER_SPEED_MBPS).then_some(found)
    }

    /// The one USB device with the identity `last` reported, if exactly one has it.
    fn find(&mut self, usb: &mut impl Usb, name: &str, last: &Link) -> Option<Link> {
        match usb.find(&last.identity) {
            Ok(Found::One(link)) => Some(link),
            Ok(Found::Nothing) => None,
            Ok(Found::Many) => {
                self.info_once(&format!(
                    "{name}: more than one USB device reports its serial, so which is it can't \
                     be told without its video device"
                ));
                None
            }
            Err(error) => {
                self.warn_once(&format!("{name}: the USB devices can't be listed: {error}"));
                None
            }
        }
    }

    fn present_without_video(
        &mut self,
        usb: &mut impl Usb,
        name: &str,
        device: &Path,
        bare: Link,
        now: Duration,
    ) -> Opening {
        self.absence = Absence::Present;
        self.notice(&bare, now);
        self.last_seen = Some(bare.clone());
        if self.standing != Standing::Free {
            return Opening::on(None);
        }
        self.recover(usb, name, device, Where::Bare(bare))
    }

    /// A reset, then, if it isn't at SuperSpeed with its video device after, a power cycle: each
    /// as allowed; then where that leaves it. `start` is where it is now, below SuperSpeed.
    ///
    /// A camera with its video device is left as it is unless Visor can cut its port's power: a
    /// reset can leave a camera not answering, and then only its power going off brings it back.
    fn recover(&mut self, usb: &mut impl Usb, name: &str, device: &Path, start: Where) -> Opening {
        if let Where::Video(link) = &start
            && self.port(usb, name, link).is_none()
        {
            self.warn_once(&outcome::left_as_it_is(name, link));
            return Opening::on(Some(link.clone()));
        }
        let mut tried = None;
        let mut found = start;
        if let Some(back) = self.reset(usb, name, device, &found) {
            tried = Some(Tried::Reset);
            found = self.locate(usb, name, device, back, &found);
        }
        if !at_super_speed(&found)
            && let Some(back) = self.power_cycle(usb, name, device, &found, &why_power(&found))
        {
            tried = Some(Tried::then_power_cycle(tried));
            found = self.locate(usb, name, device, back, &found);
        }
        match (tried, found) {
            (Some(tried), found) => self.settle(usb, name, found, tried),
            (None, Where::Video(link)) => Opening::on(Some(link)),
            (None, Where::Bare(link)) => {
                self.warn_once(&outcome::without_video(name, &link, None));
                Opening::on(None)
            }
            (None, Where::Nowhere) => Opening::on(None),
        }
    }

    /// One USB reset of the camera where it's `at`, when Visor may: how the wait for the camera
    /// ended. None when nothing was sent.
    fn reset(&mut self, usb: &mut impl Usb, name: &str, device: &Path, at: &Where) -> Option<Back> {
        if self.reset == Allowed::Refused {
            return None;
        }
        let (link, what) = match at {
            Where::Video(link) => (link, format!("below the {SUPER_SPEED_MBPS} Mb/s it can do")),
            Where::Bare(link) => (link, "with no video device".to_string()),
            Where::Nowhere => return None,
        };
        info!("{name}: on {link}, {what}: a USB reset, once, to let it link at USB 3");
        match usb.reset(device, link) {
            Ok(back) => Some(back),
            Err(why) if why.kind() == std::io::ErrorKind::PermissionDenied => {
                self.reset = Allowed::Refused;
                self.warn_once(&format!(
                    "{name}: Visor may not reset USB devices ({why}: the release's udev rule \
                     gives the video group that), so it doesn't try"
                ));
                None
            }
            Err(why) => {
                warn!("{name}: the USB reset failed: {why}");
                Some(Back::NotYet)
            }
        }
    }

    /// One power cycle of the port feeding the camera last on `link`, when it can be switched and
    /// Visor may: how the wait for the camera ended. None when nothing was sent.
    fn power_cycle(
        &mut self,
        usb: &mut impl Usb,
        name: &str,
        device: &Path,
        at: &Where,
        why: &str,
    ) -> Option<Back> {
        if self.power == Allowed::Refused {
            return None;
        }
        // A camera not found since is on the port it was last seen on.
        let last = self.last_seen.clone();
        let link = match (at, &last) {
            (Where::Video(link) | Where::Bare(link), _) => link,
            (Where::Nowhere, Some(last)) => last,
            (Where::Nowhere, None) => return None,
        };
        let port = self.port(usb, name, link)?;
        info!(
            "{name}: {why}: powering its USB port off for {} s and on again, once ({})",
            crate::typec::OFF_FOR.as_secs(),
            port.display()
        );
        match usb.power_cycle(device, &port) {
            Ok(back) => Some(back),
            Err(why) if why.kind() == std::io::ErrorKind::PermissionDenied => {
                self.power = Allowed::Refused;
                self.warn_once(&format!(
                    "{name}: Visor may not switch its port's power ({why}: the release's udev \
                     rule gives the video group that), so it doesn't try"
                ));
                None
            }
            Err(why) => {
                warn!("{name}: the power cycle of its port failed: {why}");
                Some(Back::NotYet)
            }
        }
    }

    /// The port Visor can power off and on that feeds the camera on `link`. When there's none,
    /// why is said once.
    fn port(&mut self, usb: &mut impl Usb, name: &str, link: &Link) -> Option<PathBuf> {
        if self.power == Allowed::Refused {
            return None;
        }
        match usb.power_port(link) {
            Ok(PowerPort::Switchable(port)) => Some(port),
            Ok(PowerPort::Fixed(fixed)) => {
                self.info_once(&format!(
                    "{name}: its port's power can't be switched: {fixed}"
                ));
                None
            }
            Ok(PowerPort::Refused(port)) => {
                self.power = Allowed::Refused;
                self.warn_once(&format!(
                    "{name}: Visor may not switch its port's power ({} isn't its to write: the \
                     release's udev rule gives the video group that), so it doesn't try",
                    port.join("port_type").display()
                ));
                None
            }
            Err(error) => {
                self.warn_once(&format!(
                    "{name}: whether its port's power can be switched can't be read: {error}"
                ));
                None
            }
        }
    }

    /// Where the camera is after a wait that ended `back`, having been `before`: with its video
    /// device; reset in place with its video device, so there still, its video device to come; or
    /// on USB without one, found by what it reported.
    fn locate(
        &mut self,
        usb: &mut impl Usb,
        name: &str,
        device: &Path,
        back: Back,
        before: &Where,
    ) -> Where {
        if let Ok(Some(link)) = usb.link(device) {
            return Where::Video(link);
        }
        if let (Back::InPlace, Where::Video(link)) = (back, before) {
            return Where::Video(link.clone());
        }
        let Some(last) = self.last_seen.clone() else {
            return Where::Nowhere;
        };
        match self.find(usb, name, &last) {
            Some(bare) => Where::Bare(bare),
            None => Where::Nowhere,
        }
    }

    /// Where recovery stands after `by` left the camera where it's `found`.
    fn settle(&mut self, usb: &mut impl Usb, name: &str, found: Where, by: Tried) -> Opening {
        let at = usb.now();
        let link = match found {
            Where::Video(link) => link,
            Where::Bare(link) => {
                self.last_seen = Some(link.clone());
                self.absence = Absence::Present;
                self.standing = Standing::Spent {
                    after: Some(link.clone()),
                    at,
                };
                self.warn_once(&outcome::without_video(name, &link, Some(by)));
                return Opening::on(None);
            }
            Where::Nowhere => {
                self.standing = Standing::Spent { after: None, at };
                warn!(
                    "{name}: not back after {}; Visor keeps looking for it, and tries nothing \
                     more until it comes back by itself or Visor restarts",
                    by.said()
                );
                return Opening::on(None);
            }
        };
        self.last_seen = Some(link.clone());
        self.absence = Absence::Present;
        let outcome = if link.outlook() == Outlook::Fast {
            self.standing = Standing::Free;
            Outcome::Fast {
                on: link.clone(),
                by,
            }
        } else {
            self.standing = Standing::Spent {
                after: Some(link.clone()),
                at,
            };
            Outcome::Slow {
                on: link.clone(),
                by,
            }
        };
        Opening {
            link: Some(link),
            outcome: Some(outcome),
        }
    }

    fn warn_once(&mut self, text: &str) {
        if self.said.first_time(text) {
            warn!("{text}");
        }
    }

    fn info_once(&mut self, text: &str) {
        if self.said.first_time(text) {
            info!("{text}");
        }
    }
}

/// Whether a camera found `at` is at SuperSpeed with its video device: nothing more to try.
fn at_super_speed(at: &Where) -> bool {
    matches!(at, Where::Video(link) if link.outlook() == Outlook::Fast)
}

/// Why a power cycle is due, as the log says it, for a camera found `at` after its reset.
fn why_power(at: &Where) -> String {
    match at {
        Where::Video(now) => format!("still on {now}"),
        Where::Bare(now) => format!("on {now} with no video device"),
        Where::Nowhere => "not back after its USB reset".to_string(),
    }
}

#[cfg(test)]
mod tests;
