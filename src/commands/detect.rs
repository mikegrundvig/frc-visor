//! `visor detect`: the tags NAPA finds in PGM images, how long it takes, and each tag's poses.

use std::hint::black_box;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use visor::config::Config;
use visor::image::GrayImage;
use visor::lens::Lens;
use visor::napa::Detector;
use visor::pose::solve_square;

use super::{Result, find_camera};

/// `detect` with a configured camera's settings.
pub fn detect_as(files: &[PathBuf], repeat: u32, path: &Path, name: &str) -> Result<()> {
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

/// `detect` with the settings given on the command line: the lens as its numbers.
pub fn detect(
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

/// Detects each file's tags `repeat` times, and solves each tag's poses when there's a lens.
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
                    // Kept, so the solve can't be optimised away.
                    black_box(solve_square(&tag.corners, lens, tag_size_m));
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
