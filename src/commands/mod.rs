//! The `visor` command's subcommands other than `run`, the service ([`visor::service`]): each in
//! its own file. Part of the program, not the library.

pub mod cameras;
pub mod check;
pub mod detect;
pub mod record;
pub mod version;

use std::path::Path;

use visor::config::{Camera, Config};

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// The camera named `name` in the configuration read from `path`.
pub fn find_camera<'a>(config: &'a Config, path: &Path, name: &str) -> Result<&'a Camera> {
    config
        .cameras
        .iter()
        .find(|c| c.name == name)
        .ok_or_else(|| format!("no camera named {name:?} in {}", path.display()).into())
}
