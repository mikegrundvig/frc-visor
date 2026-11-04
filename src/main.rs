//! The `visor` command.

use std::net::UdpSocket;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};
use log::{error, info};

use visor::cameras;
use visor::clock::SystemClocks;
use visor::config::{Camera, Config, PixelFormat};
use visor::image::GrayImage;
use visor::latest::Latest;
use visor::lens::Lens;
use visor::napa::Detector;
use visor::pose::solve_square;
use visor::{capture, pipeline, wire};

/// Where the robot's deploy puts the configuration, beside the program (deploy/visor.service).
const DEFAULT_CONFIG: &str = "/opt/visor/visor.toml";

#[derive(Parser)]
#[command(version, about = "AprilTag vision for an FRC robot's coprocessor")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Capture, detect and send, for every configured camera (the service).
    Run {
        #[arg(long, default_value = DEFAULT_CONFIG)]
        config: PathBuf,
    },
    /// Print the tags NAPA finds in 8-bit PGM images, and how long it takes.
    Detect {
        files: Vec<PathBuf>,
        #[arg(long, default_value_t = 16)]
        min_tag_px: i32,
        /// Detect each image this many times and print the timing.
        #[arg(long, default_value_t = 1)]
        repeat: u32,
        /// Solve each tag's poses with this lens: fx,fy,cx,cy then 0, 4, 5 or 8 distortion
        /// coefficients, comma-separated.
        #[arg(long, value_delimiter = ',', allow_negative_numbers = true)]
        lens: Option<Vec<f64>>,
        /// The tags' black square, in metres.
        #[arg(long, default_value_t = visor::config::DEFAULT_TAG_SIZE_M)]
        tag_size_m: f64,
        /// Take the lens, tag size and min_tag_px from this camera's entry in a configuration
        /// instead, as `visor run` would, and check the images are its size.
        #[arg(long, requires = "config", conflicts_with_all = ["lens", "tag_size_m", "min_tag_px"])]
        camera: Option<String>,
        #[arg(long, requires = "camera")]
        config: Option<PathBuf>,
    },
    /// Save frames from one camera as PGM files, for lens calibration. Stop the service first: a
    /// camera can only be open once.
    Record {
        #[arg(long, default_value = DEFAULT_CONFIG)]
        config: PathBuf,
        /// The camera's name in the configuration, once the robot has configured the board.
        #[arg(long, required_unless_present = "device", conflicts_with = "device")]
        camera: Option<String>,
        /// Or the camera's device, as `visor cameras` names it, at the size it's to run.
        #[arg(long, requires_all = ["width", "height"])]
        device: Option<PathBuf>,
        #[arg(long, requires = "device")]
        width: Option<u32>,
        #[arg(long, requires = "device")]
        height: Option<u32>,
        #[arg(long, requires = "device", default_value_t = visor::config::DEFAULT_FPS)]
        fps: u32,
        /// GREY or YUYV; without one, GREY if the camera offers it.
        #[arg(long, requires = "device", value_parser = ["GREY", "YUYV"])]
        format: Option<String>,
        /// In RAM on a read-only board, and gone at the next boot.
        #[arg(long, default_value = "/tmp/visor-frames")]
        out: PathBuf,
        #[arg(long, default_value_t = 40)]
        count: u32,
        /// Milliseconds between saved frames, to move the board between them.
        #[arg(long, default_value_t = 1000)]
        every_ms: u64,
    },
    /// Check a configuration as `visor run` reads it, and say what it sets up. The robot's deploy
    /// runs it with the new program on the new configuration before putting either in place.
    Check {
        #[arg(long, default_value = DEFAULT_CONFIG)]
        config: PathBuf,
    },
    /// List the cameras plugged in: each one's device to give Visor, and the formats and sizes it
    /// offers. Works while the service runs.
    Cameras,
    /// Print the version, its version ID, and the licence notices.
    Version,
}

fn main() -> ExitCode {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let result = match Cli::parse().command {
        Command::Run { config } => run(&config),
        Command::Detect {
            files,
            min_tag_px,
            repeat,
            lens,
            tag_size_m,
            camera,
            config,
        } => match (config, camera) {
            (Some(config), Some(camera)) => detect_as(&files, repeat, &config, &camera),
            _ => detect(&files, min_tag_px, repeat, lens.as_deref(), tag_size_m),
        },
        Command::Record {
            config,
            camera,
            device,
            width,
            height,
            fps,
            format,
            out,
            count,
            every_ms,
        } => {
            let every = Duration::from_millis(every_ms);
            match (camera, device, width, height) {
                (Some(name), _, _, _) => record_configured(&config, &name, &out, count, every),
                (None, Some(device), Some(width), Some(height)) => {
                    let camera = unconfigured(device, width, height, fps, format.as_deref());
                    record(&camera, &out, count, every)
                }
                _ => Err("give --camera, or --device with --width and --height".into()),
            }
        }
        Command::Check { config } => check(&config),
        Command::Cameras => cameras(),
        Command::Version => {
            version();
            Ok(())
        }
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(why) => {
            error!("{why}");
            ExitCode::FAILURE
        }
    }
}

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// Two threads per camera, until the process is stopped. A thread that panics ends the process, and
/// systemd restarts it.
fn run(path: &Path) -> Result<()> {
    let config = Config::load(path)?;
    info!(
        "visor {} (version ID {}): {} cameras, frames to {}",
        env!("CARGO_PKG_VERSION"),
        wire::VERSION_ID,
        config.cameras.len(),
        config.robot
    );
    let mut detectors = Vec::new();
    let mut sockets = Vec::new();
    for camera in &config.cameras {
        detectors.push(Detector::new(camera.min_tag_px)?);
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        socket.connect(config.robot)?;
        sockets.push(socket);
    }
    let slots: Vec<Latest> = config.cameras.iter().map(|_| Latest::new()).collect();
    let stop = AtomicBool::new(false);
    let stop = &stop;
    thread::scope(|scope| {
        let cameras = config
            .cameras
            .iter()
            .zip(detectors)
            .zip(&slots)
            .zip(sockets);
        for (((camera, mut detector), latest), socket) in cameras {
            scope.spawn(move || capture::run(camera, latest, &SystemClocks, stop));
            scope.spawn(move || {
                let mut send = |bytes: &[u8]| socket.send(bytes).map(|_| ());
                pipeline::run(
                    camera,
                    latest,
                    &mut detector,
                    &SystemClocks,
                    &mut send,
                    stop,
                )
            });
        }
    });
    Ok(())
}

/// `detect` with a configured camera's settings.
fn detect_as(files: &[PathBuf], repeat: u32, path: &Path, name: &str) -> Result<()> {
    let config = Config::load(path)?;
    let camera = find_camera(&config, path, name)?;
    for file in files {
        let image = GrayImage::read_pgm(file)?;
        if (image.width, image.height) != (camera.width, camera.height) {
            return Err(format!(
                "{} is {}x{}, but camera {name:?} runs at {}x{}",
                file.display(),
                image.width,
                image.height,
                camera.width,
                camera.height
            )
            .into());
        }
    }
    detect_with(
        files,
        camera.min_tag_px,
        repeat,
        camera.lens.clone(),
        camera.tag_size_m,
    )
}

fn detect(
    files: &[PathBuf],
    min_tag_px: i32,
    repeat: u32,
    lens: Option<&[f64]>,
    tag_size_m: f64,
) -> Result<()> {
    let lens = match lens {
        Some(values) if values.len() >= 4 => Some(Lens::new(
            values[0],
            values[1],
            values[2],
            values[3],
            &values[4..],
        )?),
        Some(_) => return Err("--lens is fx,fy,cx,cy and then any distortion coefficients".into()),
        None => None,
    };
    detect_with(files, min_tag_px, repeat, lens, tag_size_m)
}

fn detect_with(
    files: &[PathBuf],
    min_tag_px: i32,
    repeat: u32,
    lens: Option<Lens>,
    tag_size_m: f64,
) -> Result<()> {
    if files.is_empty() {
        return Err("give one or more PGM files".into());
    }
    let mut detector = Detector::new(min_tag_px)?;
    for file in files {
        let image = GrayImage::read_pgm(file)?;
        let mut times = Vec::new();
        for _ in 0..repeat.max(1) {
            let started = Instant::now();
            detector.detect(&image)?;
            times.push(started.elapsed());
        }
        let found = detector.detect(&image)?;
        println!(
            "{}: {}x{}, {} tags",
            file.display(),
            image.width,
            image.height,
            found.len()
        );
        for tag in found {
            let c = tag.corners;
            println!(
                "  tag {:3}  hamming {}  margin {:5.1}  corners ({:.2}, {:.2}) ({:.2}, {:.2}) ({:.2}, {:.2}) ({:.2}, {:.2})",
                tag.id,
                tag.hamming,
                tag.decision_margin,
                c[0],
                c[1],
                c[2],
                c[3],
                c[4],
                c[5],
                c[6],
                c[7]
            );
        }
        if repeat > 1 {
            times.sort();
            let ms = |d: Duration| d.as_secs_f64() * 1000.0;
            println!(
                "  {} runs: median {:.2} ms, 95% {:.2} ms, worst {:.2} ms",
                times.len(),
                ms(times[times.len() / 2]),
                ms(times[times.len() * 95 / 100]),
                ms(times[times.len() - 1])
            );
        }
        if let Some(lens) = &lens {
            let mut times = Vec::new();
            for tag in found {
                for _ in 0..repeat.max(1) {
                    let started = Instant::now();
                    let _ = solve_square(&tag.corners, lens, tag_size_m);
                    times.push(started.elapsed());
                }
                match solve_square(&tag.corners, lens, tag_size_m) {
                    Some([best, alt]) => println!(
                        "  tag {:3}  best at ({:.3}, {:.3}, {:.3}) m, {:.3} px; other {:.3} px",
                        tag.id, best.tvec.x, best.tvec.y, best.tvec.z, best.error_px, alt.error_px
                    ),
                    None => println!("  tag {:3}  no pose", tag.id),
                }
            }
            if repeat > 1 && !times.is_empty() {
                times.sort();
                println!(
                    "  pose solve: median {:.1} µs per tag",
                    times[times.len() / 2].as_secs_f64() * 1e6
                );
            }
        }
    }
    Ok(())
}

fn record_configured(
    path: &Path,
    name: &str,
    out: &Path,
    count: u32,
    every: Duration,
) -> Result<()> {
    let config = Config::load(path)?;
    let camera = find_camera(&config, path, name)?;
    record(camera, out, count, every)
}

/// A camera by its device alone, before any robot has configured the board: no controls, no lens.
/// Its frames are named after the device.
fn unconfigured(
    device: PathBuf,
    width: u32,
    height: u32,
    fps: u32,
    format: Option<&str>,
) -> Camera {
    let name = device
        .file_name()
        .map_or("camera".into(), |n| n.to_string_lossy().into_owned());
    Camera {
        name,
        index: 0,
        device,
        width,
        height,
        fps,
        format: match format {
            Some("GREY") => Some(PixelFormat::Grey),
            Some("YUYV") => Some(PixelFormat::Yuyv),
            _ => None,
        },
        min_tag_px: visor::config::DEFAULT_MIN_TAG_PX,
        time_offset_nanos: 0,
        controls: Vec::new(),
        lens: None,
        tag_size_m: visor::config::DEFAULT_TAG_SIZE_M,
    }
}

fn record(camera: &Camera, out: &Path, count: u32, every: Duration) -> Result<()> {
    std::fs::create_dir_all(out)?;
    let latest = Latest::new();
    let stop = AtomicBool::new(false);
    thread::scope(|scope| -> Result<()> {
        scope.spawn(|| capture::run(camera, &latest, &SystemClocks, &stop));
        let saved = save_frames(camera, &latest, out, count, every);
        stop.store(true, Ordering::Relaxed);
        saved
    })
}

fn find_camera<'a>(config: &'a Config, path: &Path, name: &str) -> Result<&'a Camera> {
    config
        .cameras
        .iter()
        .find(|c| c.name == name)
        .ok_or_else(|| format!("no camera named {name:?} in {}", path.display()).into())
}

fn save_frames(
    camera: &Camera,
    latest: &Latest,
    out: &Path,
    count: u32,
    every: Duration,
) -> Result<()> {
    for n in 0..count {
        thread::sleep(every);
        let frame = latest.take(Duration::from_secs(5)).ok_or(
            "no frame from the camera for 5 s: if Visor's service is running, it has the \
                 camera (systemctl stop visor first)",
        )?;
        let file = out.join(format!("{}-{n:03}.pgm", camera.name));
        frame.image.write_pgm(&file)?;
        println!("{}", file.display());
        latest.recycle(frame.image);
    }
    Ok(())
}

/// Each camera plugged in: the device to give Visor, and what it offers. A video device that offers
/// no capture format (a UVC camera's second, metadata, device) is left out.
fn cameras() -> Result<()> {
    let plugged = cameras::plugged(
        Path::new(cameras::BY_ID),
        Path::new(cameras::BY_PATH),
        Path::new(cameras::SYSFS),
    )?;
    let mut listed = 0;
    for each in &plugged {
        let offers = match offered(&each.device) {
            Ok(offers) if offers.is_empty() => continue,
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
        for line in offers {
            println!("  {line}");
        }
    }
    if listed == 0 {
        println!("no cameras plugged in");
    } else {
        println!("Visor reads GREY and YUYV; give a camera its device and one of these sizes.");
    }
    Ok(())
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

fn check(path: &Path) -> Result<()> {
    let config = Config::load(path)?;
    println!(
        "{}: visor {} (version ID {}), frames to {}",
        path.display(),
        env!("CARGO_PKG_VERSION"),
        wire::VERSION_ID,
        config.robot
    );
    for camera in &config.cameras {
        println!(
            "  camera {} {:?}: {}x{} at {} fps, {}, {}",
            camera.index,
            camera.name,
            camera.width,
            camera.height,
            camera.fps,
            camera.device.display(),
            if camera.lens.is_some() {
                "lens calibrated"
            } else {
                "no lens: corners only"
            }
        );
    }
    Ok(())
}

fn version() {
    println!(
        "visor {} (version ID {})",
        env!("CARGO_PKG_VERSION"),
        wire::VERSION_ID
    );
    println!("Includes NAPA, the AprilTag detector, by Zachary Wilson (github.com/zwatx/NAPA).");
    println!(
        "Visor and NAPA are free software under the GNU GPL, version 3 only, with no warranty."
    );
}
