//! `visor cameras`: the cameras plugged in, each with its device to give Visor, what it offers,
//! and its controls; or only their names.

use std::path::Path;

use visor::cameras;
use visor::usb::{self, Outlook};

use super::Result;

/// `visor cameras`: each camera described, or only its name.
pub fn list_cameras(names: bool, by_id: &Path, by_path: &Path, sysfs: &Path) -> Result<()> {
    let plugged = cameras::plugged(by_id, by_path, sysfs)?;
    if names {
        camera_names(&plugged);
        return Ok(());
    }
    cameras(&plugged, sysfs)
}

/// Each camera's name, one per line, for Visor's Spotter pack.
fn camera_names(plugged: &[cameras::Plugged]) {
    for name in cameras::capture_names(plugged) {
        println!("{name}");
    }
}

/// Each camera plugged in: the device to give Visor, its USB link, what it offers, and its
/// controls. A video device that doesn't capture (a UVC camera's second, metadata, device) is left
/// out.
fn cameras(plugged: &[cameras::Plugged], sysfs: &Path) -> Result<()> {
    let mut listed = 0;
    for each in plugged.iter().filter(|each| each.captures()) {
        let offers = match offered(&each.device) {
            Ok(offers) if offers.is_empty() => vec!["no capture formats".to_string()],
            Ok(offers) => offers,
            Err(why) => vec![format!("can't be read: {why}")],
        };
        listed += 1;
        println!("{} ({})", each.name().display(), each.device.display());
        if each.serial.is_none() {
            println!(
                "  no USB serial, so named by the USB port it's in, always: label the port and \
                 its cable"
            );
        } else if each.by_id.is_empty() {
            println!(
                "  its serial is another camera's, so named by the USB port it's in: label the \
                 port and its cable"
            );
        }
        println!("  {}", link_line(sysfs, &each.device));
        for line in offers {
            println!("  {line}");
        }
        match controls(&each.device) {
            Ok(lines) if lines.is_empty() => println!("  no controls"),
            Ok(lines) => {
                println!("  controls, as v4l2-ctl names them:");
                for line in lines {
                    println!("    {line}");
                }
            }
            Err(why) => println!("  its controls can't be read: {why}"),
        }
    }
    if listed == 0 {
        println!("no cameras plugged in");
    } else {
        println!(
            "Visor reads GREY, YUYV and MJPG, taking the first of those that gives the size at \
             the frame rate; give a camera its device and one of these sizes. Its exposure and \
             gain are set by the robot's configuration (exposure_us, gain)."
        );
    }
    Ok(())
}

/// A camera's USB link as `visor cameras` says it: its speed, and whether it could do better.
fn link_line(sysfs: &Path, device: &Path) -> String {
    match usb::link_of(sysfs, device) {
        Ok(Some(link)) => match link.outlook() {
            Outlook::MayDoBetter => format!(
                "on {link}: below SuperSpeed though it reports USB {}, likely a USB 3 camera \
                 whose USB 3 contacts aren't connecting; Visor tries once to bring it to USB 3 \
                 as it opens it",
                link.version
            ),
            Outlook::Fast | Outlook::AtItsBest => format!("on {link}"),
        },
        Ok(None) => "not on USB, or its USB link can't be found".to_string(),
        Err(why) => format!("its USB link can't be read: {why}"),
    }
}

/// A device's controls: each one's range, default and value now.
fn controls(device: &Path) -> Result<Vec<String>> {
    let opened = v4l::Device::with_path(device)?;
    Ok(visor::settings::describe(&opened)?)
}

/// A device's capture formats, each with its frame sizes.
fn offered(device: &Path) -> Result<Vec<String>> {
    use v4l::video::Capture;
    let opened = v4l::Device::with_path(device)?;
    let mut lines = Vec::new();
    for format in opened.enum_formats()? {
        let sizes: Vec<String> = opened
            .enum_framesizes(format.fourcc)?
            .into_iter()
            .map(|size| match size.size {
                v4l::framesize::FrameSizeEnum::Discrete(d) => format!("{}x{}", d.width, d.height),
                v4l::framesize::FrameSizeEnum::Stepwise(s) => format!(
                    "{}x{} to {}x{}",
                    s.min_width, s.min_height, s.max_width, s.max_height
                ),
            })
            .collect();
        lines.push(format!("{}: {}", format.fourcc, sizes.join(", ")));
    }
    Ok(lines)
}
