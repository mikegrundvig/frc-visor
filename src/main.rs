//! The `visor` command.

use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr, TcpListener, UdpSocket};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::Arc;
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
use visor::stream::feed::Feed;
use visor::stream::view::{self, View};
use visor::stream::{self, server};
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
    /// List the cameras plugged in: each one's device to give Visor, the formats and sizes it
    /// offers, and its controls, with their ranges and defaults. Works while the service runs.
    Cameras {
        /// Only each camera's name, one per line, as Visor's Spotter pack reports them to the
        /// robot (spotter/visor/status): no camera is opened.
        #[arg(long)]
        names: bool,
        /// udev's folders of names, and the kernel's video devices, for an image laid out
        /// otherwise (the pack's VISOR_BY_ID, VISOR_BY_PATH and VISOR_SYSFS), and tests.
        #[arg(long, default_value = cameras::BY_ID)]
        by_id: PathBuf,
        #[arg(long, default_value = cameras::BY_PATH)]
        by_path: PathBuf,
        #[arg(long, default_value = cameras::SYSFS)]
        sysfs: PathBuf,
    },
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
        Command::Cameras {
            names,
            by_id,
            by_path,
            sysfs,
        } => list_cameras(names, &by_id, &by_path, &sysfs),
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

/// Three threads per camera (capture, detect, live view) and the live view's server, until the
/// process is stopped. A thread that panics ends the process only because panics abort
/// (Cargo.toml's profiles): the scope would otherwise wait for the other threads, which never
/// finish. systemd then restarts Visor (deploy/visor.service).
fn run(path: &Path) -> Result<()> {
    let (config, text) = Config::read(path)?;
    info!(
        "visor {} (version ID {}): {} cameras, frames to {}",
        env!("CARGO_PKG_VERSION"),
        wire::VERSION_ID,
        config.cameras.len(),
        config.robot
    );
    let mut detectors = Vec::new();
    let mut sockets = Vec::new();
    // Bound, not connected, and each frame sent to the robot's address: connecting fails while
    // the network has no route to the robot yet, and the cameras would wait for it to open.
    let any = match config.robot {
        SocketAddr::V4(_) => IpAddr::V4(Ipv4Addr::UNSPECIFIED),
        SocketAddr::V6(_) => IpAddr::V6(Ipv6Addr::UNSPECIFIED),
    };
    for camera in &config.cameras {
        detectors.push(Detector::new(camera.min_tag_px)?);
        sockets.push(UdpSocket::bind((any, 0))?);
    }
    let robot = config.robot;
    let slots: Vec<Latest> = config.cameras.iter().map(|_| Latest::new()).collect();
    let feeds: Vec<Feed> = config.cameras.iter().map(|_| Feed::new()).collect();
    let views: Arc<Vec<View>> =
        Arc::new(config.cameras.iter().map(|c| View::new(&c.name)).collect());
    // Without its port, Visor detects all the same.
    let listener = match TcpListener::bind((Ipv4Addr::UNSPECIFIED, stream::PORT)) {
        Ok(listener) => Some(listener),
        Err(why) => {
            error!(
                "the live view can't have port {} ({why}): detection goes on without it",
                stream::PORT
            );
            None
        }
    };
    say_running(&text);
    let stop = AtomicBool::new(false);
    let stop = &stop;
    thread::scope(|scope| {
        if let Some(listener) = listener {
            let views = Arc::clone(&views);
            scope.spawn(move || server::run(listener, views, server::Options::default(), stop));
        }
        let cameras = config
            .cameras
            .iter()
            .zip(detectors)
            .zip(&slots)
            .zip(sockets)
            .zip(&feeds)
            .zip(views.iter());
        for (((((camera, mut detector), latest), socket), feed), shown) in cameras {
            scope.spawn(move || capture::run(camera, latest, &SystemClocks, stop));
            scope.spawn(move || {
                panic_if_a_test_asks(&camera.name);
                let mut send = |bytes: &[u8]| socket.send_to(bytes, robot).map(|_| ());
                pipeline::run(
                    camera,
                    latest,
                    &mut detector,
                    &SystemClocks,
                    &mut send,
                    feed,
                    stop,
                )
            });
            scope.spawn(move || view::run(camera, feed, latest, shown, stop));
        }
    });
    Ok(())
}

/// Says what this process runs, for Visor's Spotter pack, once all it needs is open: the cameras'
/// threads start next. Without systemd's runtime folder (run by hand) there's nowhere to say it.
/// A file that can't be written is said in the log, and detection goes on: the robot then sees
/// the board as not running its Visor, and installs it again.
fn say_running(config: &str) {
    let Some(folder) = visor::running::folder() else {
        return;
    };
    let text = visor::running::text(
        std::process::id(),
        env!("CARGO_PKG_VERSION"),
        config.as_bytes(),
    );
    if let Err(why) = visor::running::write(&folder, &text) {
        error!(
            "can't say what's running in {} ({why}): the robot won't see this Visor running",
            folder.display()
        );
    }
}

/// A fault switch for tests/run.rs, in debug builds only: the detect thread of the camera that
/// VISOR_TEST_PANIC names panics as it starts, so the test can check that a panic ends the process.
#[cfg(debug_assertions)]
fn panic_if_a_test_asks(camera: &str) {
    let Some(asked) = std::env::var_os("VISOR_TEST_PANIC") else {
        return;
    };
    if asked == camera {
        panic!("{camera}: the panic VISOR_TEST_PANIC asks for");
    }
}

/// A release build has no fault switch.
#[cfg(not(debug_assertions))]
fn panic_if_a_test_asks(_camera: &str) {}

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

/// A camera by its device alone, before any robot has configured the board: its own exposure and
/// gain, no controls, no lens. Its frames are named after the device.
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
        exposure_us: None,
        gain: None,
        controls: Vec::new(),
        lens: None,
        tag_size_m: visor::config::DEFAULT_TAG_SIZE_M,
        stream: visor::config::Stream::default(),
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

/// `visor cameras`: each camera described, or only its name.
fn list_cameras(names: bool, by_id: &Path, by_path: &Path, sysfs: &Path) -> Result<()> {
    let plugged = cameras::plugged(by_id, by_path, sysfs)?;
    if names {
        camera_names(&plugged);
        return Ok(());
    }
    cameras(&plugged)
}

/// Each camera's name, one per line, for Visor's Spotter pack.
fn camera_names(plugged: &[cameras::Plugged]) {
    for name in cameras::capture_names(plugged) {
        println!("{name}");
    }
}

/// Each camera plugged in: the device to give Visor, what it offers, and its controls. A video
/// device that doesn't capture (a UVC camera's second, metadata, device) is left out.
fn cameras(plugged: &[cameras::Plugged]) -> Result<()> {
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
            "Visor reads GREY and YUYV; give a camera its device and one of these sizes. Its \
             exposure and gain are set by the robot's configuration (exposure_us, gain)."
        );
    }
    Ok(())
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
        let exposure = match camera.exposure_us {
            Some(us) => format!("exposure {:.1} ms", f64::from(us) / 1000.0),
            None => "the camera's own exposure".to_string(),
        };
        let gain = match camera.gain {
            Some(gain) => format!("gain {gain}"),
            None => "its own gain".to_string(),
        };
        let (width, height) = camera.stream.size(camera.width, camera.height);
        println!(
            "  camera {} {:?}: {}x{} at {} fps, {}, {exposure}, {gain}, {}; live view {width}x{height} \
             at {} fps, quality {}",
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
            },
            camera.stream.fps,
            camera.stream.quality
        );
    }
    println!(
        "  live view on port {}: /stream/<camera>, and / lists them",
        stream::PORT
    );
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
