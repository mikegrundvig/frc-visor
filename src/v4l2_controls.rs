//! A V4L2 device's controls, as its kernel describes them.
//!
//! This is the walk the `v4l` crate's `query_controls` does, done here because the crate (0.14.0)
//! panics on a control of a type it doesn't know, as a newer kernel's region of interest rectangle
//! is. The ioctl itself, its request numbers and the kernel's structures are still the crate's.
//!
//! A control of a type Visor doesn't know is listed by its name and its type's number
//! ([`Kind::Other`]), and never read: exposure and gain, which Visor sets, are integers and a menu.

use std::io;
use std::os::raw::c_void;
use std::ptr;

use v4l::control::{Flags, Type};
use v4l::v4l_sys::{v4l2_query_ext_ctrl, v4l2_querymenu, v4l2_querymenu__bindgen_ty_1};
use v4l::v4l2;
use v4l::v4l2::vidioc::{VIDIOC_QUERY_EXT_CTRL, VIDIOC_QUERYMENU};

use crate::settings::{Control, Kind, control_key};

/// The two questions the walk asks a device, so a test can answer them for a camera that isn't
/// plugged in.
pub(crate) trait Kernel {
    /// `VIDIOC_QUERY_EXT_CTRL`: fills in the control that `control.id` asks for.
    fn control(&self, control: &mut v4l2_query_ext_ctrl) -> io::Result<()>;
    /// `VIDIOC_QUERYMENU`: fills in the choice of menu `item.id` at `item.index`.
    fn menu_item(&self, item: &mut v4l2_querymenu) -> io::Result<()>;
}

impl Kernel for v4l::Device {
    fn control(&self, control: &mut v4l2_query_ext_ctrl) -> io::Result<()> {
        let argument = ptr::from_mut(control).cast::<c_void>();
        // Safety: the request takes this structure, which outlives the call.
        unsafe { v4l2::ioctl(self.handle().fd(), VIDIOC_QUERY_EXT_CTRL, argument) }
    }

    fn menu_item(&self, item: &mut v4l2_querymenu) -> io::Result<()> {
        let argument = ptr::from_mut(item).cast::<c_void>();
        // Safety: the request takes this structure, which outlives the call.
        unsafe { v4l2::ioctl(self.handle().fd(), VIDIOC_QUERYMENU, argument) }
    }
}

/// Every control of a device, in the device's order, without the classes that only head a group
/// of them. A device with no controls gives none.
pub(crate) fn list(device: &impl Kernel) -> io::Result<Vec<Control>> {
    let next = (Flags::NEXT_CTRL | Flags::NEXT_COMPOUND).bits();
    let mut controls = Vec::new();
    let mut raw = nothing_yet();
    loop {
        // Asks for the control after the last one answered, those with a payload included.
        raw.id |= next;
        match device.control(&mut raw) {
            Ok(()) => {}
            // How the kernel says there are no more.
            Err(error) if error.kind() == io::ErrorKind::InvalidInput => return Ok(controls),
            Err(error) => return Err(error),
        }
        if Type::try_from(raw.type_) == Ok(Type::CtrlClass) {
            continue;
        }
        controls.push(described(device, &raw));
    }
}

/// The question before the first: every field zero.
fn nothing_yet() -> v4l2_query_ext_ctrl {
    v4l2_query_ext_ctrl {
        id: 0,
        type_: 0,
        name: [0; 32],
        minimum: 0,
        maximum: 0,
        step: 0,
        default_value: 0,
        flags: 0,
        elem_size: 0,
        elems: 0,
        nr_of_dims: 0,
        dims: [0; 4],
        reserved: [0; 32],
    }
}

/// A control as Visor lists it, from the kernel's answer for it.
fn described(device: &impl Kernel, raw: &v4l2_query_ext_ctrl) -> Control {
    // The name's bytes: C's `char` is signed on some processors and unsigned on others.
    let name: Vec<u8> = raw.name.iter().map(|c| c.to_ne_bytes()[0]).collect();
    let name = text(&name);
    let kind = match Type::try_from(raw.type_) {
        // The crate, which reads a control's value, panics on a name that isn't UTF-8 as it does
        // on a type it doesn't know: neither is read.
        _ if name.is_none() => Kind::Other(raw.type_),
        Ok(Type::Integer | Type::Integer64) => Kind::Integer,
        Ok(Type::Boolean) => Kind::Boolean,
        Ok(Type::Menu | Type::IntegerMenu) => Kind::Menu,
        _ => Kind::Other(raw.type_),
    };
    let menu = match kind {
        Kind::Menu => choices(device, raw),
        _ => Vec::new(),
    };
    Control {
        id: raw.id,
        name: control_key(&name.unwrap_or_else(|| "unnamed".into())),
        kind,
        minimum: raw.minimum,
        maximum: raw.maximum,
        step: raw.step,
        default: raw.default_value,
        menu,
        inactive: raw.flags & Flags::INACTIVE.bits() != 0,
    }
}

/// A menu's choices: each value and its name, or for a menu of numbers, the number.
fn choices(device: &impl Kernel, raw: &v4l2_query_ext_ctrl) -> Vec<(i64, String)> {
    let numbers = Type::try_from(raw.type_) == Ok(Type::IntegerMenu);
    let mut choices = Vec::new();
    for value in raw.minimum..=raw.maximum {
        let Ok(index) = u32::try_from(value) else {
            continue;
        };
        let mut item = v4l2_querymenu {
            id: raw.id,
            index,
            __bindgen_anon_1: v4l2_querymenu__bindgen_ty_1 { name: [0; 32] },
            reserved: 0,
        };
        // A camera needn't offer every value in a menu's range: the kernel refuses the others.
        if device.menu_item(&mut item).is_err() {
            continue;
        }
        // Safety: the two are plain bytes and an integer over the same memory, which was all set
        // above, so either reads a valid value; the kernel fills in the one for the menu's type.
        let named = if numbers {
            unsafe { item.__bindgen_anon_1.value }.to_string()
        } else {
            let name = unsafe { item.__bindgen_anon_1.name };
            text(&name).unwrap_or_else(|| "unnamed".into())
        };
        choices.push((value, named));
    }
    choices
}

/// A C string in a fixed array: what's before its first zero byte, when that's UTF-8.
fn text(bytes: &[u8]) -> Option<String> {
    let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
    String::from_utf8(bytes[..end].to_vec()).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `V4L2_CTRL_TYPE_RECT`, which the `v4l` crate (0.14.0) has no name for.
    const RECT: u32 = 0x107;
    /// `V4L2_CTRL_FLAG_HAS_PAYLOAD`: a control the kernel only lists when asked for compound ones.
    const HAS_PAYLOAD: u32 = 0x100;
    const INACTIVE: u32 = 0x10;

    /// A device as its kernel answers for it.
    #[derive(Default)]
    struct Fake {
        controls: Vec<v4l2_query_ext_ctrl>,
        /// (menu's control, index, the choice's name).
        names: Vec<(u32, u32, &'static [u8])>,
        /// (menu's control, index, the choice's number).
        numbers: Vec<(u32, u32, i64)>,
        /// A control the device fails to answer for, as one unplugged mid-walk does.
        fails_at: Option<u32>,
    }

    impl Kernel for Fake {
        fn control(&self, control: &mut v4l2_query_ext_ctrl) -> io::Result<()> {
            let next = Flags::NEXT_CTRL.bits();
            let compound = Flags::NEXT_COMPOUND.bits();
            assert_ne!(control.id & next, 0, "the walk asks for the next control");
            let after = control.id & !(next | compound);
            let found = self.controls.iter().find(|c| {
                c.id > after && (control.id & compound != 0 || c.flags & HAS_PAYLOAD == 0)
            });
            match found {
                Some(found) if self.fails_at == Some(found.id) => {
                    Err(io::Error::from_raw_os_error(libc::ENODEV))
                }
                Some(found) => {
                    *control = *found;
                    Ok(())
                }
                None => Err(io::Error::from_raw_os_error(libc::EINVAL)),
            }
        }

        fn menu_item(&self, item: &mut v4l2_querymenu) -> io::Result<()> {
            let (id, index) = (item.id, item.index);
            let of = |menu: u32, at: u32| menu == id && at == index;
            if let Some((_, _, name)) = self.names.iter().find(|(m, i, _)| of(*m, *i)) {
                let mut bytes = [0; 32];
                bytes[..name.len()].copy_from_slice(name);
                item.__bindgen_anon_1.name = bytes;
                return Ok(());
            }
            if let Some((_, _, number)) = self.numbers.iter().find(|(m, i, _)| of(*m, *i)) {
                item.__bindgen_anon_1.value = *number;
                return Ok(());
            }
            Err(io::Error::from_raw_os_error(libc::EINVAL))
        }
    }

    /// One control as the kernel describes it: (ID, type, name, minimum, maximum, step, default,
    /// flags).
    type Row = (u32, u32, &'static [u8], i64, i64, u64, i64, u32);

    fn raw(row: Row) -> v4l2_query_ext_ctrl {
        let (id, type_, name, minimum, maximum, step, default_value, flags) = row;
        let mut described = v4l2_query_ext_ctrl {
            id,
            type_,
            minimum,
            maximum,
            step,
            default_value,
            flags,
            ..nothing_yet()
        };
        for (to, from) in described.name.iter_mut().zip(name) {
            *to = std::ffi::c_char::from_ne_bytes([*from]);
        }
        described
    }

    /// A laptop's UVC webcam on a kernel that gives it region of interest controls.
    fn webcam() -> Fake {
        let rows: [Row; 12] = [
            (0x0098_0001, 6, b"User Controls", 0, 0, 0, 0, 0x44),
            (0x0098_0900, 1, b"Brightness", 0, 255, 1, 128, 0),
            (0x0098_090c, 2, b"White Balance, Automatic", 0, 1, 1, 1, 0),
            (0x0098_0918, 3, b"Power Line Frequency", 0, 3, 1, 3, 0),
            (
                0x0098_1ae1,
                RECT,
                b"Region of Interest Rectangle",
                0,
                0,
                0,
                0,
                HAS_PAYLOAD,
            ),
            (
                0x0098_1ae2,
                8,
                b"Region of Interest Auto Ctrls",
                0,
                9,
                0,
                9,
                0,
            ),
            (0x009a_0001, 6, b"Camera Controls", 0, 0, 0, 0, 0x44),
            (0x009a_0901, 3, b"Auto Exposure", 0, 3, 1, 3, 0),
            (
                0x009a_0902,
                1,
                b"Exposure Time, Absolute",
                2,
                1250,
                1,
                156,
                INACTIVE,
            ),
            (
                0x009a_0903,
                2,
                b"Exposure, Dynamic Framerate",
                0,
                1,
                1,
                0,
                0,
            ),
            (0x009f_0903, 9, b"Link Frequency", 0, 1, 1, 0, 0),
            (0x009f_0904, 5, b"Pixel Rate", 0, i64::MAX, 1, 0, 0),
        ];
        Fake {
            controls: rows.into_iter().map(raw).collect(),
            names: vec![
                (0x0098_0918, 0, b"Disabled"),
                (0x0098_0918, 1, b"50 Hz"),
                (0x0098_0918, 2, b"60 Hz"),
                (0x0098_0918, 3, b"Auto"),
                // A UVC camera's exposure menu: values 0 and 2 aren't offered.
                (0x009a_0901, 1, b"Manual Mode"),
                (0x009a_0901, 3, b"Aperture Priority Mode"),
            ],
            numbers: vec![(0x009f_0903, 0, 297_000_000), (0x009f_0903, 1, 594_000_000)],
            fails_at: None,
        }
    }

    #[test]
    fn a_control_of_a_type_the_crate_doesn_t_know_is_listed_and_the_walk_goes_on() {
        let listed = list(&webcam()).unwrap();
        let kinds: Vec<(&str, Kind)> = listed.iter().map(|c| (c.name.as_str(), c.kind)).collect();
        assert_eq!(
            kinds,
            [
                ("brightness", Kind::Integer),
                ("white_balance_automatic", Kind::Boolean),
                ("power_line_frequency", Kind::Menu),
                ("region_of_interest_rectangle", Kind::Other(RECT)),
                ("region_of_interest_auto_ctrls", Kind::Other(8)),
                ("auto_exposure", Kind::Menu),
                ("exposure_time_absolute", Kind::Integer),
                ("exposure_dynamic_framerate", Kind::Boolean),
                ("link_frequency", Kind::Menu),
                ("pixel_rate", Kind::Integer),
            ]
        );
    }

    #[test]
    fn a_control_carries_its_range_default_choices_and_whether_it_counts() {
        let listed = list(&webcam()).unwrap();
        let named = |name: &str| listed.iter().find(|c| c.name == name).unwrap().clone();
        assert_eq!(
            named("exposure_time_absolute"),
            Control {
                id: 0x009a_0902,
                name: "exposure_time_absolute".into(),
                kind: Kind::Integer,
                minimum: 2,
                maximum: 1250,
                step: 1,
                default: 156,
                menu: Vec::new(),
                inactive: true,
            }
        );
        assert_eq!(
            named("auto_exposure").menu,
            [
                (1, "Manual Mode".to_string()),
                (3, "Aperture Priority Mode".to_string())
            ]
        );
        assert_eq!(
            named("link_frequency").menu,
            [(0, "297000000".to_string()), (1, "594000000".to_string())]
        );
        assert!(!named("auto_exposure").inactive);
    }

    #[test]
    fn a_name_that_isn_t_text_makes_a_control_one_that_s_never_read() {
        let device = Fake {
            controls: vec![
                raw((0x0098_0900, 1, b"Bright\xff", 0, 255, 1, 128, 0)),
                // A name that fills its array, with no zero byte after it.
                raw((
                    0x0098_0901,
                    1,
                    b"Contrast of the whole of a frame",
                    0,
                    9,
                    1,
                    5,
                    0,
                )),
            ],
            ..Fake::default()
        };
        let listed = list(&device).unwrap();
        let kinds: Vec<(&str, Kind)> = listed.iter().map(|c| (c.name.as_str(), c.kind)).collect();
        assert_eq!(
            kinds,
            [
                ("unnamed", Kind::Other(1)),
                ("contrast_of_the_whole_of_a_frame", Kind::Integer),
            ]
        );
    }

    #[test]
    fn no_controls_is_an_empty_list_and_a_device_that_stops_answering_is_an_error() {
        assert_eq!(list(&Fake::default()).unwrap(), []);

        let mut unplugged = webcam();
        unplugged.fails_at = Some(0x009a_0901);
        let error = list(&unplugged).unwrap_err();
        assert_eq!(error.raw_os_error(), Some(libc::ENODEV));
    }
}
