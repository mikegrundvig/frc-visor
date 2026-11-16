//! Getting a camera back to its best link when it costs nothing: a USB 3 camera stuck at USB 2, or
//! one that stopped answering. Two means, each sent at most once ([`crate::usb`] and
//! [`crate::typec`] say why each works):
//!
//! - A camera linked below SuperSpeed that looks able to do better gets one USB reset. If it's
//!   still not at SuperSpeed after (still slow, or not back within the wait) and its port's power
//!   can be switched, one power cycle of that port.
//! - A camera seen in this run that has been gone for [`GONE_FOR`] gets one power cycle of the port
//!   it was last on: the hung camera.
//!
//! Never while it streams: only in the capture loop's open path, as Visor starts and when it opens
//! again a camera that went away anyway. Once, not in a loop: after a recovery that left it below
//! SuperSpeed or gone, nothing more is tried until the camera comes back by itself (an enumeration
//! Visor didn't cause) or Visor restarts. A recovery Visor may not send (no permission, or a port
//! that can't be switched) is said once, and is never an error for the camera.

use std::path::Path;
use std::time::Duration;

use log::{info, warn};

use crate::settings::Said;
use crate::usb::{Back, Link, Outlook, PowerPort, SUPER_SPEED_MBPS, Usb};

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

/// What a recovery sent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tried {
    Reset,
    PowerCycle,
    ResetAndPowerCycle,
}

impl Tried {
    /// What was tried, with a power cycle after it.
    fn then_power_cycle(tried: Option<Tried>) -> Tried {
        match tried {
            Some(Tried::Reset | Tried::ResetAndPowerCycle) => Tried::ResetAndPowerCycle,
            Some(Tried::PowerCycle) | None => Tried::PowerCycle,
        }
    }

    fn said(self) -> &'static str {
        match self {
            Tried::Reset => "a USB reset",
            Tried::PowerCycle => "a power cycle of its port",
            Tried::ResetAndPowerCycle => "a USB reset and a power cycle of its port",
        }
    }
}

/// What a recovery came to, to say with what Visor then runs, once the camera streams.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Outcome {
    /// At SuperSpeed now.
    Fast { on: Link, by: Tried },
    /// Back, but still below SuperSpeed.
    Slow { on: Link, by: Tried },
}

/// Whether a recovery was tried since the camera last appeared, and left it below SuperSpeed.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Recovered {
    #[default]
    NotTried,
    TriedStillSlow,
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
        self.recover_slow(usb, name, device, link)
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
        match self.power_cycle(usb, name, device, &last, &why) {
            Some(on) => self.settle(usb, name, on, Tried::PowerCycle),
            None => Opening::on(None),
        }
    }

    fn recover_slow(
        &mut self,
        usb: &mut impl Usb,
        name: &str,
        device: &Path,
        link: Link,
    ) -> Opening {
        let mut tried = None;
        let mut on = Some(link.clone());
        if self.reset == Allowed::Yes {
            info!(
                "{name}: on {link}, below the {SUPER_SPEED_MBPS} Mb/s it can do: a USB reset, \
                 once, to let it link at USB 3"
            );
            match usb.reset(device, &link) {
                Ok(back) => {
                    tried = Some(Tried::Reset);
                    on = after(usb, device, back);
                }
                Err(why) if why.kind() == std::io::ErrorKind::PermissionDenied => {
                    self.reset = Allowed::Refused;
                    self.warn_once(&format!(
                        "{name}: Visor may not reset USB devices ({why}: the release's udev rule \
                         gives the video group that), so it doesn't try"
                    ));
                }
                Err(why) => {
                    tried = Some(Tried::Reset);
                    warn!("{name}: the USB reset failed: {why}");
                    on = usb.link(device).ok().flatten();
                }
            }
        }
        let fast = on.as_ref().is_some_and(|l| l.outlook() == Outlook::Fast);
        if !fast {
            let why = match &on {
                Some(now) => format!("still on {now}"),
                None => "not back after its USB reset".to_string(),
            };
            let port_of = on.clone().unwrap_or(link);
            if let Some(powered) = self.power_cycle(usb, name, device, &port_of, &why) {
                tried = Some(Tried::then_power_cycle(tried));
                on = powered;
            }
        }
        match tried {
            Some(tried) => self.settle(usb, name, on, tried),
            None => Opening::on(on),
        }
    }

    /// One power cycle of the port feeding the camera last on `link`, when it can be switched and
    /// Visor may: the link it came back on (none: not back). None when nothing was sent.
    fn power_cycle(
        &mut self,
        usb: &mut impl Usb,
        name: &str,
        device: &Path,
        link: &Link,
        why: &str,
    ) -> Option<Option<Link>> {
        if self.power == Allowed::Refused {
            return None;
        }
        let port = match usb.power_port(link) {
            Ok(PowerPort::Switchable(port)) => port,
            Ok(PowerPort::Fixed(fixed)) => {
                self.info_once(&format!(
                    "{name}: its port's power can't be switched: {fixed}"
                ));
                return None;
            }
            Err(error) => {
                self.warn_once(&format!(
                    "{name}: whether its port's power can be switched can't be read: {error}"
                ));
                return None;
            }
        };
        info!(
            "{name}: {why}: powering its USB port off for {} s and on again, once ({})",
            crate::typec::OFF_FOR.as_secs(),
            port.display()
        );
        match usb.power_cycle(device, &port) {
            Ok(back) => Some(after(usb, device, back)),
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
                Some(usb.link(device).ok().flatten())
            }
        }
    }

    /// Where recovery stands after `by` left the camera on `on` (none: not back).
    fn settle(&mut self, usb: &mut impl Usb, name: &str, on: Option<Link>, by: Tried) -> Opening {
        let at = usb.now();
        let Some(link) = on else {
            self.standing = Standing::Spent { after: None, at };
            warn!(
                "{name}: not back after {}; Visor keeps looking for it, and tries nothing more \
                 until it comes back by itself or Visor restarts",
                by.said()
            );
            return Opening::on(None);
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

/// The camera's link after a wait for it: none when it wasn't back.
fn after(usb: &mut impl Usb, device: &Path, back: Back) -> Option<Link> {
    match back {
        Back::Returned => usb.link(device).ok().flatten(),
        Back::NotYet => None,
    }
}

/// What a recovery came to, said once the camera streams, with what Visor runs it at (`running`).
pub fn say(name: &str, outcome: &Outcome, running: &str) {
    match outcome {
        Outcome::Fast { on, by } => info!("{name}: on {on} after {}; {running}", by.said()),
        Outcome::Slow { on, by } => warn!(
            "{name}: still on {on} after {}, so its USB 3 contacts are probably not connecting: \
             reseat its plug or change its cable. Visor runs it as it is, {running}, and tries \
             nothing more until it's replugged or Visor restarts",
            by.said()
        ),
    }
}

#[cfg(test)]
mod tests;
