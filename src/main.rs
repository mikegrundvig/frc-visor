//! The `visor` command.

mod commands;

use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;
use std::time::Duration;

use clap::{Parser, Subcommand};
use log::{error, info};

use visor::cameras;
use visor::capture::V4l2;
use visor::clock::SystemClocks;
use visor::config::Config;
use visor::latest::Latest;
use visor::napa::Detector;
use visor::service::{self, Wired};
use visor::stream::{self, server};
use visor::{watchdog, wire};

use commands::Result;

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
            (Some(config), Some(camera)) => {
                commands::detect::detect_as(&files, repeat, &config, &camera)
            }
            _ => commands::detect::detect(&files, min_tag_px, repeat, lens.as_deref(), tag_size_m),
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
                (Some(name), _, _, _) => {
                    commands::record::record_configured(&config, &name, &out, count, every)
                }
                (None, Some(device), Some(width), Some(height)) => {
                    let camera = commands::record::unconfigured(
                        device,
                        width,
                        height,
                        fps,
                        format.as_deref(),
                    );
                    commands::record::record(&camera, &out, count, every)
                }
                _ => Err("give --camera, or --device with --width and --height".into()),
            }
        }
        Command::Check { config } => commands::check::check(&config),
        Command::Cameras {
            names,
            by_id,
            by_path,
            sysfs,
        } => commands::cameras::list_cameras(names, &by_id, &by_path, &sysfs),
        Command::Version => {
            commands::version::version();
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

/// The service: each configured camera's threads and the live view, until the process is stopped
/// ([`service::run`]).
fn run(path: &Path) -> Result<()> {
    let (config, text) = Config::read(path)?;
    info!(
        "visor {} (version ID {}): {} cameras, frames to {}",
        env!("CARGO_PKG_VERSION"),
        wire::VERSION_ID,
        config.cameras.len(),
        config.robot
    );
    // Bound, not connected, and each frame sent to the robot's address: connecting fails while
    // the network has no route to the robot yet, and the cameras would wait for it to open.
    let any = match config.robot {
        SocketAddr::V4(_) => IpAddr::V4(Ipv4Addr::UNSPECIFIED),
        SocketAddr::V6(_) => IpAddr::V6(Ipv6Addr::UNSPECIFIED),
    };
    let mut cameras = Vec::new();
    for camera in &config.cameras {
        cameras.push(Wired {
            camera,
            source: V4l2,
            detector: Detector::new(camera.min_tag_px)?,
            socket: UdpSocket::bind((any, 0))?,
            latest: Latest::new(),
        });
    }
    let listener = service::listen(SocketAddr::from((Ipv4Addr::UNSPECIFIED, stream::PORT)));
    say_running(&text);
    // Ready: what each camera needs is open, and its threads start next. systemd watches from here
    // (deploy/visor.service: Type=notify, WatchdogSec=).
    watchdog::ready();
    let stop = AtomicBool::new(false);
    service::run(
        cameras,
        config.robot,
        listener,
        server::Options {
            try_settings: config.try_settings,
            ..server::Options::default()
        },
        watchdog::interval(),
        &SystemClocks,
        &stop,
    );
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
