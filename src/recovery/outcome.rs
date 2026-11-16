//! What a recovery sent, what it came to, and how the log says it.

use log::{info, warn};

use crate::usb::Link;

/// What a recovery sent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tried {
    Reset,
    PowerCycle,
    ResetAndPowerCycle,
}

impl Tried {
    /// What was tried, with a power cycle after it.
    pub(super) fn then_power_cycle(tried: Option<Tried>) -> Tried {
        match tried {
            Some(Tried::Reset | Tried::ResetAndPowerCycle) => Tried::ResetAndPowerCycle,
            Some(Tried::PowerCycle) | None => Tried::PowerCycle,
        }
    }

    pub(super) fn said(self) -> &'static str {
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
    /// With its video device, but still below SuperSpeed.
    Slow { on: Link, by: Tried },
}

/// Whether a recovery was tried since the camera last appeared, and left it below SuperSpeed.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Recovered {
    #[default]
    NotTried,
    TriedStillSlow,
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

/// A camera on USB with no video device, as the log says it once.
pub(super) fn without_video(name: &str, on: &Link, by: Option<Tried>) -> String {
    let after = by.map_or(String::new(), |by| format!(" after {}", by.said()));
    format!(
        "{name}: on {on}{after}, where it offers no video at all (USB 1 speed carries none of \
         its formats), so it can't run until its plug is reseated; Visor tries nothing more until \
         it's replugged or Visor restarts"
    )
}

/// A camera below SuperSpeed that works, on a port whose power Visor can't switch, as the log
/// says it once.
pub(super) fn left_as_it_is(name: &str, on: &Link) -> String {
    format!(
        "{name}: on {on}, below the USB 3 it can do, and Visor can't switch its port's power, so \
         it sends no USB reset: one that left the camera not answering couldn't be undone from \
         here. Visor runs it as it is; reseat its plug to bring it back to USB 3"
    )
}
