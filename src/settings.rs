//! A camera's exposure and gain, set from its configuration every time the camera is opened (after a
//! replug too), then read back for its frames to carry; its other controls, by name; and the list
//! of controls `visor cameras` prints.
//!
//! Visor has no opinion of its own: it sets what the configuration gives, and where it gives no
//! exposure or gain, it puts back the camera's own default, so what the camera does follows from the
//! configuration alone, whatever was set before. A camera without one of these controls is said
//! once, and keeps capturing: the robot sees what's in effect in every frame.
//!
//! Exposure and gain are found by V4L2's standard control IDs rather than their names, which the
//! kernel has changed ("Exposure, Auto" became "Auto Exposure").

use std::collections::HashSet;
use std::io;

use v4l::control::{self, Control as V4l2Control, Value};

use crate::config::Camera;

/// `auto_exposure` (`V4L2_CID_EXPOSURE_AUTO`): a menu, its manual mode [`MANUAL`].
pub const AUTO_EXPOSURE: u32 = 0x009a_0901;
/// `exposure_time_absolute` (`V4L2_CID_EXPOSURE_ABSOLUTE`), in units of [`EXPOSURE_UNIT_US`].
pub const EXPOSURE_TIME: u32 = 0x009a_0902;
/// `gain` (`V4L2_CID_GAIN`), in the camera's own units.
pub const GAIN: u32 = 0x0098_0913;
/// `auto_exposure`'s manual mode, `V4L2_EXPOSURE_MANUAL`: the exposure is what's set.
pub const MANUAL: i64 = 1;
/// V4L2's unit of exposure time: 100 µs.
pub const EXPOSURE_UNIT_US: u32 = 100;

/// One of a camera's controls, as it describes it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Control {
    pub id: u32,
    /// As `v4l2-ctl --list-ctrls` prints it.
    pub name: String,
    pub kind: Kind,
    pub minimum: i64,
    pub maximum: i64,
    pub step: u64,
    pub default: i64,
    /// A menu's choices: each value and its name.
    pub menu: Vec<(i64, String)>,
    /// Whether another control makes it count for nothing now, as auto exposure does the exposure.
    pub inactive: bool,
}

/// What kind of value a control takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Integer,
    Boolean,
    Menu,
    /// One Visor neither sets nor lists the range of (a button, a string).
    Other,
}

/// A camera's controls: a V4L2 device's, or a test's.
pub trait Controls {
    /// Every control, in the camera's order.
    fn list(&self) -> io::Result<Vec<Control>>;
    /// A control's value now.
    fn get(&self, control: &Control) -> io::Result<i64>;
    fn set(&self, control: &Control, value: i64) -> io::Result<()>;
}

impl Controls for v4l::Device {
    fn list(&self) -> io::Result<Vec<Control>> {
        let described = match self.query_controls() {
            Ok(described) => described,
            // What a device with no controls at all answers the first query with.
            Err(error) if error.kind() == io::ErrorKind::InvalidInput => Vec::new(),
            Err(error) => return Err(error),
        };
        Ok(described
            .into_iter()
            .filter(|c| c.typ != control::Type::CtrlClass)
            .map(|c| Control {
                id: c.id,
                name: control_key(&c.name),
                kind: match c.typ {
                    control::Type::Integer | control::Type::Integer64 => Kind::Integer,
                    control::Type::Boolean => Kind::Boolean,
                    control::Type::Menu | control::Type::IntegerMenu => Kind::Menu,
                    _ => Kind::Other,
                },
                minimum: c.minimum,
                maximum: c.maximum,
                step: c.step,
                default: c.default,
                menu: c
                    .items
                    .unwrap_or_default()
                    .into_iter()
                    .map(|(value, item)| (i64::from(value), item.to_string()))
                    .collect(),
                inactive: c.flags.contains(control::Flags::INACTIVE),
            })
            .collect())
    }

    fn get(&self, control: &Control) -> io::Result<i64> {
        match self.control(control.id)?.value {
            Value::Integer(value) => Ok(value),
            Value::Boolean(on) => Ok(i64::from(on)),
            _ => Err(io::Error::new(
                io::ErrorKind::Unsupported,
                format!("{} isn't a number", control.name),
            )),
        }
    }

    fn set(&self, control: &Control, value: i64) -> io::Result<()> {
        let value = match control.kind {
            Kind::Boolean => Value::Boolean(value != 0),
            _ => Value::Integer(value),
        };
        self.set_control(V4l2Control {
            id: control.id,
            value,
        })
    }
}

/// The exposure and gain a camera's frames were taken at, as read back from it once they were set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InEffect {
    /// The exposure, in microseconds; none while the camera chooses it (its automatic exposure is
    /// on), or where it can't be read.
    pub exposure_us: Option<u32>,
    /// The `gain` control's value; none where the camera has none.
    pub gain: Option<i32>,
    /// Whether the camera's own automatic exposure is on; none where it has no such control.
    pub auto_exposure: Option<bool>,
}

/// What setting a camera's controls did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Applied {
    pub in_effect: InEffect,
    /// What didn't take as configured, each a sentence to log; capture goes on regardless.
    pub warnings: Vec<String>,
}

/// Sets a camera's exposure and gain from its configuration, then its other controls in order, and
/// reads back what's in effect.
///
/// A camera without one of the exposure or gain controls, a value outside its range (set to the
/// nearest end), or a value the camera refuses or keeps otherwise (rounded to its step, say), is a
/// warning. A control under `controls` the camera doesn't have fails the open, unlike those: it's
/// named by hand, so a misspelling is the likelier cause.
pub fn apply(device: &impl Controls, camera: &Camera) -> io::Result<Applied> {
    let known = match device.list() {
        Ok(known) => known,
        // Without named controls to set, a camera whose controls can't be read still captures.
        Err(error) if camera.controls.is_empty() => {
            return Ok(Applied {
                in_effect: InEffect::default(),
                warnings: vec![format!(
                    "its controls can't be read ({error}), so its exposure and gain stay its own"
                )],
            });
        }
        Err(error) => return Err(error),
    };
    let find = |id: u32| known.iter().find(|c| c.id == id);
    let mut warnings = Vec::new();
    let mut written = Written::default();

    match camera.exposure_us {
        Some(us) => {
            match find(AUTO_EXPOSURE) {
                Some(auto) => written.set(device, auto, MANUAL, &mut warnings, plain),
                None => warnings.push(
                    "the camera has no auto_exposure control, so its automatic exposure can't be \
                     turned off: exposure_us may not hold"
                        .into(),
                ),
            }
            match find(EXPOSURE_TIME) {
                Some(time) => {
                    let units = (f64::from(us) / f64::from(EXPOSURE_UNIT_US)).round() as i64;
                    let value = within(time, units, &mut warnings, microseconds);
                    written.set(device, time, value, &mut warnings, microseconds);
                }
                None => warnings.push(
                    "the camera has no exposure_time_absolute control, so its exposure stays its \
                     own"
                    .into(),
                ),
            }
        }
        // The camera's own: its automatic exposure back on, as the camera starts.
        None => {
            if let Some(auto) = find(AUTO_EXPOSURE) {
                written.set(device, auto, auto.default, &mut warnings, plain);
            }
        }
    }

    match (camera.gain, find(GAIN)) {
        (Some(gain), Some(control)) => {
            let value = within(control, gain, &mut warnings, plain);
            written.set(device, control, value, &mut warnings, plain);
        }
        (Some(_), None) => {
            warnings.push("the camera has no gain control, so its gain stays its own".into())
        }
        (None, Some(control)) => {
            written.set(device, control, control.default, &mut warnings, plain)
        }
        (None, None) => {}
    }

    set_named(device, &known, &camera.controls)?;

    let read = |control: Option<&Control>| {
        let control = control?;
        device
            .get(control)
            .ok()
            .or_else(|| written.value(control.id))
    };
    let auto_exposure = read(find(AUTO_EXPOSURE)).map(|mode| mode != MANUAL);
    let exposure_us = match auto_exposure {
        Some(true) => None,
        _ => read(find(EXPOSURE_TIME))
            .and_then(|units| u32::try_from(units * i64::from(EXPOSURE_UNIT_US)).ok()),
    };
    let gain = read(find(GAIN)).and_then(|gain| i32::try_from(gain).ok());
    Ok(Applied {
        in_effect: InEffect {
            exposure_us,
            gain,
            auto_exposure,
        },
        warnings,
    })
}

/// A control's value as the configuration has it: the exposure in µs, from V4L2's units.
fn microseconds(units: i64) -> String {
    format!("{} µs", units * i64::from(EXPOSURE_UNIT_US))
}

/// A control's value as it is: the gain's, and the auto exposure menu's.
fn plain(value: i64) -> String {
    value.to_string()
}

/// A value held to a control's range, with a warning when it isn't in it; `shown` writes a value
/// in the configuration's terms.
fn within(
    control: &Control,
    value: i64,
    warnings: &mut Vec<String>,
    shown: impl Fn(i64) -> String,
) -> i64 {
    let held = value.clamp(control.minimum, control.maximum);
    if held != value {
        warnings.push(format!(
            "{} {} is outside what the camera takes ({} to {}): set to {}",
            control.name,
            shown(value),
            shown(control.minimum),
            shown(control.maximum),
            shown(held)
        ));
    }
    held
}

/// The values Visor wrote, for what's in effect where the camera can't be read back.
#[derive(Default)]
struct Written(Vec<(u32, i64)>);

impl Written {
    /// Sets a control, and reads it back: a camera may round a value to its step, or hold it
    /// tighter than its range says (an exposure to the frame time). A value it refuses, or keeps
    /// otherwise, is a warning; `shown` writes a value in the configuration's terms.
    fn set(
        &mut self,
        device: &impl Controls,
        control: &Control,
        value: i64,
        warnings: &mut Vec<String>,
        shown: impl Fn(i64) -> String,
    ) {
        if let Err(error) = device.set(control, value) {
            warnings.push(format!(
                "setting {} to {}: {error}",
                control.name,
                shown(value)
            ));
            return;
        }
        self.0.push((control.id, value));
        match device.get(control) {
            Ok(kept) if kept != value => warnings.push(format!(
                "{}: the camera took {}, not {}",
                control.name,
                shown(kept),
                shown(value)
            )),
            _ => {}
        }
    }

    fn value(&self, id: u32) -> Option<i64> {
        self.0.iter().rev().find(|(i, _)| *i == id).map(|(_, v)| *v)
    }
}

/// Sets each control under `controls`, in order, by its `v4l2-ctl` name.
fn set_named(
    device: &impl Controls,
    known: &[Control],
    controls: &[(String, i64)],
) -> io::Result<()> {
    for (name, value) in controls {
        let Some(control) = known.iter().find(|c| c.name == *name) else {
            let names: Vec<&str> = known.iter().map(|c| c.name.as_str()).collect();
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("no control {name}; the camera has {}", names.join(", ")),
            ));
        };
        device
            .set(control, *value)
            .map_err(|e| io::Error::new(e.kind(), format!("setting {name}: {e}")))?;
    }
    Ok(())
}

/// A control's name as `v4l2-ctl` prints it: "Exposure Time, Absolute" is
/// `exposure_time_absolute`.
pub fn control_key(name: &str) -> String {
    let mut key = String::new();
    for c in name.chars() {
        if c.is_ascii_alphanumeric() {
            key.push(c.to_ascii_lowercase());
        } else if !key.is_empty() && !key.ends_with('_') {
            key.push('_');
        }
    }
    key.trim_end_matches('_').to_string()
}

/// Warnings already said, so a camera that reopens after a replug says each one once.
#[derive(Default)]
pub struct Said(HashSet<String>);

impl Said {
    /// Whether this is the first time `warning` is said.
    pub fn first_time(&mut self, warning: &str) -> bool {
        self.0.insert(warning.to_string())
    }
}

/// Each control, one line each, as `visor cameras` lists them: its range, default and value now,
/// and what Visor's configuration sets it by.
pub fn describe(device: &impl Controls) -> io::Result<Vec<String>> {
    let mut lines = Vec::new();
    for control in device.list()? {
        let now = match device.get(&control) {
            Ok(value) => shown(&control, value),
            Err(_) => "unreadable".into(),
        };
        let what = match control.kind {
            Kind::Integer => format!(
                "{} to {}{}",
                control.minimum,
                control.maximum,
                if control.step > 1 {
                    format!(" in steps of {}", control.step)
                } else {
                    String::new()
                }
            ),
            Kind::Boolean => "on or off".into(),
            Kind::Menu => {
                let items: Vec<String> = control
                    .menu
                    .iter()
                    .map(|(value, name)| format!("{value} {name}"))
                    .collect();
                format!("one of {}", items.join(", "))
            }
            Kind::Other => "not a setting".into(),
        };
        let mut line = format!(
            "{}: {what}; default {}, now {now}",
            control.name,
            shown(&control, control.default)
        );
        if control.inactive {
            line.push_str(" (inactive)");
        }
        match control.id {
            AUTO_EXPOSURE => line.push_str(&format!(
                " [{MANUAL} turns it off: Visor's exposure_us does that]"
            )),
            EXPOSURE_TIME => line.push_str(" [in 100 µs units: Visor's exposure_us]"),
            GAIN => line.push_str(" [Visor's gain]"),
            _ => {}
        }
        lines.push(line);
    }
    Ok(lines)
}

fn shown(control: &Control, value: i64) -> String {
    match control.kind {
        Kind::Boolean if value != 0 => "on".into(),
        Kind::Boolean => "off".into(),
        _ => value.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::path::PathBuf;

    /// A camera as V4L2 describes it, holding its values; it starts, as one plugged in does, at its
    /// controls' defaults.
    struct Fake {
        controls: Vec<Control>,
        values: RefCell<Vec<(u32, i64)>>,
        writes: RefCell<Vec<(String, i64)>>,
        /// A control that can't be read back.
        unreadable: Option<u32>,
        /// Whether its controls can't be listed.
        unlisted: bool,
        /// A control the camera refuses to set, as one busy streaming does.
        refuses: Option<u32>,
        /// What the camera keeps of a value it's given: the value itself, or one it rounds or
        /// clamps.
        keeps: fn(&Control, i64) -> i64,
    }

    impl Fake {
        fn new(controls: Vec<Control>) -> Fake {
            let values = controls.iter().map(|c| (c.id, c.default)).collect();
            Fake {
                controls,
                values: RefCell::new(values),
                writes: RefCell::new(Vec::new()),
                unreadable: None,
                unlisted: false,
                refuses: None,
                keeps: |_, value| value,
            }
        }

        fn writes(&self) -> Vec<(String, i64)> {
            self.writes.borrow().clone()
        }
    }

    impl Controls for Fake {
        fn list(&self) -> io::Result<Vec<Control>> {
            if self.unlisted {
                return Err(io::Error::other("no reply"));
            }
            Ok(self.controls.clone())
        }
        fn get(&self, control: &Control) -> io::Result<i64> {
            if self.unreadable == Some(control.id) {
                return Err(io::Error::other("unreadable"));
            }
            let values = self.values.borrow();
            Ok(values.iter().find(|(id, _)| *id == control.id).unwrap().1)
        }
        fn set(&self, control: &Control, value: i64) -> io::Result<()> {
            assert!(
                (control.minimum..=control.maximum).contains(&value),
                "{} {value}",
                control.name
            );
            if self.refuses == Some(control.id) {
                return Err(io::Error::from_raw_os_error(libc::EBUSY));
            }
            self.writes.borrow_mut().push((control.name.clone(), value));
            let value = (self.keeps)(control, value);
            let mut values = self.values.borrow_mut();
            values
                .iter_mut()
                .find(|(id, _)| *id == control.id)
                .unwrap()
                .1 = value;
            Ok(())
        }
    }

    fn integer(id: u32, name: &str, minimum: i64, maximum: i64, default: i64) -> Control {
        Control {
            id,
            name: name.into(),
            kind: Kind::Integer,
            minimum,
            maximum,
            step: 1,
            default,
            menu: Vec::new(),
            inactive: false,
        }
    }

    fn auto_exposure() -> Control {
        Control {
            kind: Kind::Menu,
            menu: vec![
                (1, "Manual Mode".into()),
                (3, "Aperture Priority Mode".into()),
            ],
            ..integer(AUTO_EXPOSURE, "auto_exposure", 0, 3, 3)
        }
    }

    fn exposure_time() -> Control {
        integer(EXPOSURE_TIME, "exposure_time_absolute", 1, 5000, 156)
    }

    fn gain() -> Control {
        integer(GAIN, "gain", 0, 100, 16)
    }

    fn brightness() -> Control {
        integer(0x0098_0900, "brightness", -64, 64, 0)
    }

    fn camera(exposure_us: Option<u32>, gain: Option<i64>) -> Camera {
        Camera {
            name: "front-left".into(),
            index: 0,
            device: PathBuf::from("/dev/null"),
            width: 1600,
            height: 1304,
            fps: 60,
            format: None,
            min_tag_px: 16,
            time_offset_nanos: 0,
            exposure_us,
            gain,
            controls: Vec::new(),
            lens: None,
            tag_size_m: 0.1651,
            stream: crate::config::Stream::default(),
            config_id: 0,
        }
    }

    /// A camera's controls, what's configured, then what Visor writes, what's in effect, and part
    /// of each warning.
    struct Case {
        name: &'static str,
        controls: Vec<Control>,
        camera_does: Does,
        exposure_us: Option<u32>,
        gain: Option<i64>,
        writes: Vec<(&'static str, i64)>,
        in_effect: InEffect,
        warnings: Vec<&'static str>,
    }

    /// What a camera does with a value it's given.
    enum Does {
        /// Takes it as given.
        Takes,
        /// Refuses this control: busy, as some are while streaming.
        Refuses(u32),
        /// Takes it, but keeps another value: rounded to its step, or clamped further than its
        /// range says.
        Keeps(fn(&Control, i64) -> i64),
    }

    #[test]
    fn exposure_and_gain_are_set_as_configured_and_read_back() {
        let all = || vec![auto_exposure(), exposure_time(), gain(), brightness()];
        let cases = [
            Case {
                name: "a camera with every control: auto exposure off, 1 ms, gain 40",
                controls: all(),
                camera_does: Does::Takes,
                exposure_us: Some(1000),
                gain: Some(40),
                writes: vec![
                    ("auto_exposure", 1),
                    ("exposure_time_absolute", 10),
                    ("gain", 40),
                ],
                in_effect: InEffect {
                    exposure_us: Some(1000),
                    gain: Some(40),
                    auto_exposure: Some(false),
                },
                warnings: vec![],
            },
            Case {
                name: "nothing configured: the camera's own, put back",
                controls: all(),
                camera_does: Does::Takes,
                exposure_us: None,
                gain: None,
                writes: vec![("auto_exposure", 3), ("gain", 16)],
                in_effect: InEffect {
                    exposure_us: None,
                    gain: Some(16),
                    auto_exposure: Some(true),
                },
                warnings: vec![],
            },
            Case {
                name: "an exposure alone: the gain is the camera's own",
                controls: all(),
                camera_does: Does::Takes,
                exposure_us: Some(1250),
                gain: None,
                writes: vec![
                    ("auto_exposure", 1),
                    ("exposure_time_absolute", 13),
                    ("gain", 16),
                ],
                in_effect: InEffect {
                    exposure_us: Some(1300),
                    gain: Some(16),
                    auto_exposure: Some(false),
                },
                warnings: vec![],
            },
            Case {
                name: "a camera with no gain control",
                controls: vec![auto_exposure(), exposure_time()],
                camera_does: Does::Takes,
                exposure_us: Some(1000),
                gain: Some(40),
                writes: vec![("auto_exposure", 1), ("exposure_time_absolute", 10)],
                in_effect: InEffect {
                    exposure_us: Some(1000),
                    gain: None,
                    auto_exposure: Some(false),
                },
                warnings: vec!["no gain control"],
            },
            Case {
                name: "a camera with no manual exposure: no auto_exposure control",
                controls: vec![exposure_time(), gain()],
                camera_does: Does::Takes,
                exposure_us: Some(1000),
                gain: Some(40),
                writes: vec![("exposure_time_absolute", 10), ("gain", 40)],
                in_effect: InEffect {
                    exposure_us: Some(1000),
                    gain: Some(40),
                    auto_exposure: None,
                },
                warnings: vec!["no auto_exposure control"],
            },
            Case {
                name: "a camera with no exposure time",
                controls: vec![auto_exposure(), gain()],
                camera_does: Does::Takes,
                exposure_us: Some(1000),
                gain: None,
                writes: vec![("auto_exposure", 1), ("gain", 16)],
                in_effect: InEffect {
                    exposure_us: None,
                    gain: Some(16),
                    auto_exposure: Some(false),
                },
                warnings: vec!["no exposure_time_absolute control"],
            },
            Case {
                name: "values outside the camera's ranges: the nearest end",
                controls: all(),
                camera_does: Does::Takes,
                exposure_us: Some(1_000_000),
                gain: Some(400),
                writes: vec![
                    ("auto_exposure", 1),
                    ("exposure_time_absolute", 5000),
                    ("gain", 100),
                ],
                in_effect: InEffect {
                    exposure_us: Some(500_000),
                    gain: Some(100),
                    auto_exposure: Some(false),
                },
                warnings: vec![
                    "exposure_time_absolute 1000000 µs is outside what the camera takes (100 µs to \
                     500000 µs): set to 500000 µs",
                    "gain 400 is outside what the camera takes (0 to 100): set to 100",
                ],
            },
            Case {
                name: "a camera with none of them",
                controls: vec![brightness()],
                camera_does: Does::Takes,
                exposure_us: None,
                gain: None,
                writes: vec![],
                in_effect: InEffect::default(),
                warnings: vec![],
            },
            Case {
                name: "the camera refuses the exposure: it keeps its own, and says so",
                controls: all(),
                camera_does: Does::Refuses(EXPOSURE_TIME),
                exposure_us: Some(1000),
                gain: Some(40),
                writes: vec![("auto_exposure", 1), ("gain", 40)],
                in_effect: InEffect {
                    exposure_us: Some(15_600),
                    gain: Some(40),
                    auto_exposure: Some(false),
                },
                warnings: vec![
                    "setting exposure_time_absolute to 1000 µs: Device or resource busy",
                ],
            },
            Case {
                name: "the camera refuses the gain: it keeps its own, and says so",
                controls: all(),
                camera_does: Does::Refuses(GAIN),
                exposure_us: Some(1000),
                gain: Some(40),
                writes: vec![("auto_exposure", 1), ("exposure_time_absolute", 10)],
                in_effect: InEffect {
                    exposure_us: Some(1000),
                    gain: Some(16),
                    auto_exposure: Some(false),
                },
                warnings: vec!["setting gain to 40: Device or resource busy"],
            },
            Case {
                name: "the camera rounds the exposure to its step: what it took, and a warning",
                controls: vec![
                    auto_exposure(),
                    Control {
                        step: 4,
                        ..exposure_time()
                    },
                    gain(),
                ],
                // As the kernel rounds a value to a control's step: 10 is 9, a step of 4 up from
                // the minimum of 1.
                camera_does: Does::Keeps(|control, value| {
                    let step = control.step as i64;
                    control.minimum + (value - control.minimum + step / 2) / step * step
                }),
                exposure_us: Some(1000),
                gain: Some(40),
                writes: vec![
                    ("auto_exposure", 1),
                    ("exposure_time_absolute", 10),
                    ("gain", 40),
                ],
                in_effect: InEffect {
                    exposure_us: Some(900),
                    gain: Some(40),
                    auto_exposure: Some(false),
                },
                warnings: vec!["exposure_time_absolute: the camera took 900 µs, not 1000 µs"],
            },
            Case {
                name: "the camera clamps the exposure to its frame time, inside its range",
                controls: all(),
                // At 60 fps, a frame is 16.6 ms: longer than that is cut to it.
                camera_does: Does::Keeps(|control, value| match control.id {
                    EXPOSURE_TIME => value.min(166),
                    _ => value,
                }),
                exposure_us: Some(30_000),
                gain: Some(40),
                writes: vec![
                    ("auto_exposure", 1),
                    ("exposure_time_absolute", 300),
                    ("gain", 40),
                ],
                in_effect: InEffect {
                    exposure_us: Some(16_600),
                    gain: Some(40),
                    auto_exposure: Some(false),
                },
                warnings: vec!["exposure_time_absolute: the camera took 16600 µs, not 30000 µs"],
            },
        ];
        for case in cases {
            let mut device = Fake::new(case.controls);
            match case.camera_does {
                Does::Takes => {}
                Does::Refuses(id) => device.refuses = Some(id),
                Does::Keeps(keeps) => device.keeps = keeps,
            }
            let applied = apply(&device, &camera(case.exposure_us, case.gain)).unwrap();
            let writes: Vec<(String, i64)> = case
                .writes
                .iter()
                .map(|(name, value)| (name.to_string(), *value))
                .collect();
            assert_eq!(device.writes(), writes, "{}", case.name);
            assert_eq!(applied.in_effect, case.in_effect, "{}", case.name);
            assert_eq!(
                applied.warnings.len(),
                case.warnings.len(),
                "{}: {:?}",
                case.name,
                applied.warnings
            );
            for (warning, part) in applied.warnings.iter().zip(&case.warnings) {
                assert!(warning.contains(part), "{}: {warning}", case.name);
            }
        }
    }

    #[test]
    fn named_controls_come_after_and_an_unknown_one_fails_the_open() {
        let device = Fake::new(vec![auto_exposure(), exposure_time(), gain(), brightness()]);
        let mut named = camera(Some(1000), None);
        named.controls = vec![("brightness".into(), 10), ("auto_exposure".into(), 3)];
        let applied = apply(&device, &named).unwrap();
        assert_eq!(
            device.writes().last(),
            Some(&("auto_exposure".to_string(), 3)),
            "a named control is set last, so it wins"
        );
        assert_eq!(applied.in_effect.auto_exposure, Some(true));
        assert_eq!(applied.in_effect.exposure_us, None, "the camera chooses it");

        let mut misspelt = camera(Some(1000), None);
        misspelt.controls = vec![("gian".into(), 10)];
        let error = apply(&device, &misspelt).unwrap_err().to_string();
        assert!(
            error.contains("no control gian; the camera has auto_exposure"),
            "{error}"
        );
    }

    #[test]
    fn a_replugged_camera_gets_its_settings_again_and_says_its_warnings_once() {
        let configured = camera(Some(1000), Some(40));
        let mut said = Said::default();
        let mut first = Vec::new();
        for plug in 0..2 {
            // Plugged in again, it's back at its defaults, and has no gain control.
            let device = Fake::new(vec![auto_exposure(), exposure_time()]);
            let applied = apply(&device, &configured).unwrap();
            assert_eq!(
                device.writes(),
                [
                    ("auto_exposure".to_string(), 1),
                    ("exposure_time_absolute".to_string(), 10)
                ],
                "plug {plug}"
            );
            assert_eq!(applied.in_effect.exposure_us, Some(1000));
            let to_say: Vec<String> = applied
                .warnings
                .into_iter()
                .filter(|w| said.first_time(w))
                .collect();
            if plug == 0 {
                first = to_say;
            } else {
                assert!(to_say.is_empty(), "said again: {to_say:?}");
            }
        }
        assert_eq!(first.len(), 1);
    }

    #[test]
    fn controls_that_can_t_be_listed_leave_capture_running_unless_one_is_named() {
        let mut device = Fake::new(vec![gain()]);
        device.unlisted = true;
        let applied = apply(&device, &camera(Some(1000), Some(40))).unwrap();
        assert_eq!(applied.in_effect, InEffect::default());
        assert!(
            applied.warnings[0].contains("can't be read (no reply)"),
            "{:?}",
            applied.warnings
        );

        let mut named = camera(None, None);
        named.controls = vec![("brightness".into(), 10)];
        assert!(apply(&device, &named).is_err());
    }

    #[test]
    fn what_can_t_be_read_back_is_what_visor_wrote() {
        let mut device = Fake::new(vec![auto_exposure(), exposure_time(), gain()]);
        device.unreadable = Some(GAIN);
        let applied = apply(&device, &camera(Some(1000), Some(40))).unwrap();
        assert_eq!(applied.in_effect.gain, Some(40));
    }

    #[test]
    fn the_list_gives_each_control_s_range_default_and_value() {
        let mut inactive = exposure_time();
        inactive.inactive = true;
        let mut white_balance = integer(0x0098_090c, "white_balance_automatic", 0, 1, 1);
        white_balance.kind = Kind::Boolean;
        let device = Fake::new(vec![auto_exposure(), inactive, gain(), white_balance]);
        assert_eq!(
            describe(&device).unwrap(),
            [
                "auto_exposure: one of 1 Manual Mode, 3 Aperture Priority Mode; default 3, now 3 \
                 [1 turns it off: Visor's exposure_us does that]",
                "exposure_time_absolute: 1 to 5000; default 156, now 156 (inactive) [in 100 µs \
                 units: Visor's exposure_us]",
                "gain: 0 to 100; default 16, now 16 [Visor's gain]",
                "white_balance_automatic: on or off; default on, now on",
            ]
        );
    }

    #[test]
    fn control_names_match_v4l2_ctl() {
        let cases = [
            ("Exposure Time, Absolute", "exposure_time_absolute"),
            ("Auto Exposure", "auto_exposure"),
            (
                "White Balance Temperature, Auto",
                "white_balance_temperature_auto",
            ),
            ("Gain", "gain"),
            ("  Power Line Frequency ", "power_line_frequency"),
        ];
        for (name, key) in cases {
            assert_eq!(control_key(name), key);
        }
    }
}
