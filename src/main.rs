//! The `visor` command.

use std::net::UdpSocket;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};
use log::{error, info};

use visor::clock::SystemClocks;
use visor::config::{Camera, Config};
use visor::image::GrayImage;
use visor::latest::Latest;
use visor::napa::Detector;
use visor::{capture, pipeline, wire};

const DEFAULT_CONFIG: &str = "/etc/visor/visor.toml";

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
    },
    /// Save frames from one camera as PGM files, for lens calibration. Stop the service first: a
    /// camera can only be open once.
    Record {
        #[arg(long, default_value = DEFAULT_CONFIG)]
        config: PathBuf,
        /// The camera's name in the configuration.
        #[arg(long)]
        camera: String,
        #[arg(long)]
        out: PathBuf,
        #[arg(long, default_value_t = 40)]
        count: u32,
        /// Milliseconds between saved frames, to move the board between them.
        #[arg(long, default_value_t = 1000)]
        every_ms: u64,
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
        } => detect(&files, min_tag_px, repeat),
        Command::Record {
            config,
            camera,
            out,
            count,
            every_ms,
        } => record(
            &config,
            &camera,
            &out,
            count,
            Duration::from_millis(every_ms),
        ),
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

fn detect(files: &[PathBuf], min_tag_px: i32, repeat: u32) -> Result<()> {
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
    }
    Ok(())
}

fn record(path: &Path, name: &str, out: &Path, count: u32, every: Duration) -> Result<()> {
    let config = Config::load(path)?;
    let camera: &Camera = config
        .cameras
        .iter()
        .find(|c| c.name == name)
        .ok_or_else(|| format!("no camera named {name:?} in {}", path.display()))?;
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

fn save_frames(
    camera: &Camera,
    latest: &Latest,
    out: &Path,
    count: u32,
    every: Duration,
) -> Result<()> {
    for n in 0..count {
        thread::sleep(every);
        let frame = latest
            .take(Duration::from_secs(5))
            .ok_or("no frame from the camera for 5 s")?;
        let file = out.join(format!("{}-{n:03}.pgm", camera.name));
        frame.image.write_pgm(&file)?;
        println!("{}", file.display());
        latest.recycle(frame.image);
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
