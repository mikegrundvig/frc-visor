//! A camera's focus step: a page's button starts it, Visor remembers the exposure and gain in
//! effect and dials in settings for focusing ([`dial`]), and another button ends it and puts the
//! remembered settings back. They go back too when the last watcher leaves. A manual change while
//! it's on stops the dial and is left alone; "set it for me again" dials in afresh.
//!
//! This is the bookkeeping only. The server says what was pressed, the view's thread gives it each
//! frame, and each answers with what to ask the camera's capture thread for ([`Trial`]).
//!
//! [`Trial`]: crate::settings::Trial

use std::time::Duration;

use crate::settings::{Asked, Base, InEffect, Ranges, Tried};
use crate::stream::dial::{self, Dial, Goal, Seen};

/// Whether the step is on, and who sets the exposure and gain while it is.
#[derive(Debug, Default)]
pub struct Step {
    on: Option<On>,
}

#[derive(Debug)]
struct On {
    /// The settings in effect when it started, put back when it ends: none until a frame has come.
    remembered: Option<InEffect>,
    by: By,
}

/// Who sets the exposure and gain while the step is on.
#[derive(Debug)]
enum By {
    /// The dial.
    Dial(Dial),
    /// Someone, by hand: the dial has stopped.
    Hand,
}

/// What the step is doing, to show.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Phase {
    /// Off: the camera has the settings it had.
    #[default]
    Off,
    /// On, the dial at work, in this state.
    Dialling(dial::State),
    /// On, the exposure and gain set by hand.
    ByHand,
}

impl Step {
    /// Starts the step, or, when it's on, starts the dial again. Nothing is asked of the camera
    /// until the next frame, whose settings are the ones remembered.
    pub fn start(&mut self) {
        match &mut self.on {
            Some(on) => on.by = By::Dial(Dial::new(Goal::Focusing)),
            None => {
                self.on = Some(On {
                    remembered: None,
                    by: By::Dial(Dial::new(Goal::Focusing)),
                })
            }
        }
    }

    /// Whether it's on.
    pub fn is_on(&self) -> bool {
        self.on.is_some()
    }

    /// Ends the step: what to ask for to put the remembered settings back, if any were.
    pub fn end(&mut self) -> Option<Asked> {
        let on = self.on.take()?;
        on.remembered.map(put_back)
    }

    /// A change made by hand: the dial stops, and the change is left alone.
    pub fn by_hand(&mut self) {
        if let Some(on) = &mut self.on {
            on.by = By::Hand;
        }
    }

    /// What it's doing now.
    pub fn phase(&self) -> Phase {
        match &self.on {
            None => Phase::Off,
            Some(On {
                by: By::Dial(dial), ..
            }) => Phase::Dialling(dial.state()),
            Some(On { by: By::Hand, .. }) => Phase::ByHand,
        }
    }

    /// Takes a frame: what to ask the camera for, if anything.
    pub fn frame(
        &mut self,
        seen: &Seen,
        ranges: Option<&Ranges>,
        period: Duration,
    ) -> Option<Tried> {
        let on = self.on.as_mut()?;
        if on.remembered.is_none() {
            on.remembered = Some(seen.in_effect);
        }
        let By::Dial(dial) = &mut on.by else {
            return None;
        };
        // The camera not yet opened: nothing to keep to, so nothing is asked.
        let ranges = ranges?;
        dial.step(seen, ranges, period).ask
    }
}

/// What puts settings back as they were: the exposure and gain as numbers, or, where the camera's
/// own automatic exposure was on, the configuration's settings (the only way back to it) with the
/// gain there was.
fn put_back(was: InEffect) -> Asked {
    let gain = was.gain.map(i64::from);
    match (was.auto_exposure, was.exposure_us) {
        (Some(true), _) | (_, None) => Asked {
            from: Base::Configured,
            tried: Tried {
                exposure_us: None,
                gain,
            },
        },
        (_, Some(us)) => Asked {
            from: Base::AsItIs,
            tried: Tried {
                exposure_us: Some(us),
                gain,
            },
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::Range;

    const PERIOD: Duration = Duration::from_nanos(16_666_667);

    fn ranges() -> Ranges {
        Ranges {
            exposure_us: Some(Range {
                least: 100,
                most: 819_200,
            }),
            gain: Some(Range {
                least: 16,
                most: 255,
            }),
        }
    }

    fn seen(exposure_us: u32, gain: i32, white: Option<u8>, at_ms: i64) -> Seen {
        Seen {
            in_effect: InEffect {
                exposure_us: Some(exposure_us),
                gain: Some(gain),
                auto_exposure: Some(false),
            },
            white,
            at_nanos: at_ms * 1_000_000,
        }
    }

    #[test]
    fn the_step_remembers_dials_stops_by_hand_and_puts_back_what_was() {
        let mut step = Step::default();
        assert_eq!(step.phase(), Phase::Off);
        assert_eq!(
            step.frame(&seen(1000, 40, Some(100), 0), Some(&ranges()), PERIOD),
            None
        );
        assert_eq!(step.end(), None, "off: nothing to put back");

        step.start();
        assert!(step.is_on());
        // The first frame's settings are remembered, and the dial asks for the least gain.
        let asked = step.frame(&seen(1000, 40, Some(100), 0), Some(&ranges()), PERIOD);
        assert_eq!(asked.and_then(|t| t.gain), Some(16));
        assert!(matches!(
            step.phase(),
            Phase::Dialling(dial::State::Steering)
        ));
        // By hand: the dial stops and asks nothing more.
        step.by_hand();
        assert_eq!(step.phase(), Phase::ByHand);
        assert_eq!(
            step.frame(&seen(1000, 40, Some(100), 5000), Some(&ranges()), PERIOD),
            None
        );
        // Set for me again: the dial afresh, the remembered settings kept.
        step.start();
        assert!(matches!(step.phase(), Phase::Dialling(_)));
        assert!(
            step.frame(&seen(3000, 16, Some(100), 6000), Some(&ranges()), PERIOD)
                .is_some()
        );
        assert_eq!(
            step.end(),
            Some(Asked {
                from: Base::AsItIs,
                tried: Tried {
                    exposure_us: Some(1000),
                    gain: Some(40)
                }
            })
        );
        assert_eq!(step.phase(), Phase::Off);

        // A camera on its own automatic exposure goes back to it by way of the configuration.
        step.start();
        let auto = Seen {
            in_effect: InEffect {
                exposure_us: None,
                gain: Some(16),
                auto_exposure: Some(true),
            },
            white: None,
            at_nanos: 0,
        };
        step.frame(&auto, None, PERIOD);
        assert_eq!(
            step.end(),
            Some(Asked {
                from: Base::Configured,
                tried: Tried {
                    exposure_us: None,
                    gain: Some(16)
                }
            })
        );
    }
}
