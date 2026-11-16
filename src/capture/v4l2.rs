//! The cameras V4L2 gives, their frames memory-mapped, and their USB side from sysfs and usbfs.

use std::io;
use std::path::Path;
use std::time::Duration;

use v4l::buffer::{Metadata, Type};
use v4l::fraction::Fraction;
use v4l::frameinterval::FrameIntervalEnum;
use v4l::framesize::FrameSizeEnum;
use v4l::io::mmap::Stream;
use v4l::io::traits::CaptureStream;
use v4l::video::Capture;
use v4l::video::capture::Parameters;
use v4l::{Device, Format, FourCC};

use super::{Opened, Source};
use crate::formats::Rates;
use crate::settings::{Control, Controls};
use crate::usb::{Back, Found, Identity, Link, PowerPort, SystemUsb, Usb};

const BUFFERS: u32 = 4;

/// The cameras V4L2 gives, on the system's USB.
#[derive(Debug, Clone, Default)]
pub struct V4l2 {
    pub usb: SystemUsb,
}

impl Usb for V4l2 {
    fn link(&mut self, device: &Path) -> io::Result<Option<Link>> {
        self.usb.link(device)
    }

    fn reset(&mut self, device: &Path, link: &Link) -> io::Result<Back> {
        self.usb.reset(device, link)
    }

    fn find(&mut self, identity: &Identity) -> io::Result<Found> {
        self.usb.find(identity)
    }

    fn power_port(&mut self, link: &Link) -> io::Result<PowerPort> {
        self.usb.power_port(link)
    }

    fn power_cycle(&mut self, device: &Path, port: &Path) -> io::Result<Back> {
        self.usb.power_cycle(device, port)
    }

    fn now(&mut self) -> Duration {
        self.usb.now()
    }
}

impl Source for V4l2 {
    type Opened<'a> = V4l2Camera;

    fn open(&mut self, device: &Path) -> io::Result<V4l2Camera> {
        Ok(V4l2Camera {
            stream: None,
            device: Device::with_path(device)?,
        })
    }
}

/// An open V4L2 device, and its stream once started.
pub struct V4l2Camera {
    // Before the device, so its buffers are given back before the device is closed.
    stream: Option<Stream<'static>>,
    device: Device,
}

impl Controls for V4l2Camera {
    fn list(&self) -> io::Result<Vec<Control>> {
        self.device.list()
    }

    fn get(&self, control: &Control) -> io::Result<i64> {
        Controls::get(&self.device, control)
    }

    fn set(&self, control: &Control, value: i64) -> io::Result<()> {
        Controls::set(&self.device, control, value)
    }
}

impl Opened for V4l2Camera {
    fn formats(&self) -> io::Result<Vec<[u8; 4]>> {
        Ok(self
            .device
            .enum_formats()?
            .iter()
            .map(|f| f.fourcc.repr)
            .collect())
    }

    fn rates(&self, fourcc: [u8; 4], width: u32, height: u32) -> Rates {
        rates(&self.device, FourCC::new(&fourcc), width, height)
    }

    fn set_format(&self, wanted: &Format) -> io::Result<Format> {
        Capture::set_format(&self.device, wanted)
    }

    fn set_fps(&self, fps: u32) -> io::Result<Fraction> {
        Ok(self.device.set_params(&Parameters::with_fps(fps))?.interval)
    }

    fn start(&mut self, stall: Duration) -> io::Result<()> {
        let mut stream = Stream::with_buffers(&self.device, Type::VideoCapture, BUFFERS)?;
        stream.set_timeout(stall);
        self.stream = Some(stream);
        Ok(())
    }

    fn next(&mut self) -> io::Result<(&[u8], &Metadata)> {
        let Some(stream) = self.stream.as_mut() else {
            return Err(io::Error::other("the camera isn't streaming"));
        };
        // The stream's wait running out is VIDIOC_DQBUF's TimedOut, which the capture loop says.
        stream.next()
    }
}

/// The frame rates `device` lists for a format at a size: none at a size it doesn't list, and
/// unknown where it lists sizes or rates no way Visor reads.
fn rates(device: &Device, fourcc: FourCC, width: u32, height: u32) -> Rates {
    let Ok(sizes) = device.enum_framesizes(fourcc) else {
        return Rates::Unknown;
    };
    let offered = sizes.iter().any(|size| match &size.size {
        FrameSizeEnum::Discrete(d) => (d.width, d.height) == (width, height),
        FrameSizeEnum::Stepwise(s) => {
            (s.min_width..=s.max_width).contains(&width)
                && (s.min_height..=s.max_height).contains(&height)
        }
    });
    if !offered {
        return Rates::NotAtThisSize;
    }
    let Ok(intervals) = device.enum_frameintervals(fourcc, width, height) else {
        return Rates::Unknown;
    };
    let intervals: Vec<Fraction> = intervals
        .into_iter()
        .flat_map(|each| match each.interval {
            FrameIntervalEnum::Discrete(interval) => vec![interval],
            FrameIntervalEnum::Stepwise(range) => vec![range.min, range.max],
        })
        .collect();
    Rates::of_intervals(&intervals)
}
