//! What the running service says it is, for Visor's Spotter pack (spotter/visor/status): its
//! process ID, its version and its configuration's SHA-256, written as it starts to a small file
//! in the folder systemd makes for it (`RuntimeDirectory=`, deploy/visor.service). The pack
//! reports the program and configuration on disk too, so the robot can tell one put in place but
//! not yet running (a restart that failed) from one running. The pack believes the file only when
//! its process ID is the unit's main process's: a file left by a process that has ended says
//! nothing.
//!
//! The file is lines of `key=value`:
//!
//! ```text
//! pid=1234
//! version=2026.11.8
//! config=<the configuration's SHA-256, lowercase hex>
//! ```

use std::io;
use std::path::{Path, PathBuf};

use sha2::{Digest, Sha256};

/// The file's name, in the runtime folder.
pub const FILE: &str = "running";

/// The runtime folder systemd made for the service, from `RUNTIME_DIRECTORY` (the first, if it
/// names several); none when Visor isn't run by a unit that has one.
pub fn folder() -> Option<PathBuf> {
    let named = std::env::var_os("RUNTIME_DIRECTORY")?;
    let first = named.to_str()?.split(':').next()?;
    if first.is_empty() {
        return None;
    }
    Some(PathBuf::from(first))
}

/// The file's text, for this process, version and configuration (its text exactly as read, so
/// its hash is the one `sha256sum` gives the file).
pub fn text(pid: u32, version: &str, config: &[u8]) -> String {
    format!("pid={pid}\nversion={version}\nconfig={}\n", sha256(config))
}

/// A SHA-256, as lowercase hex.
pub fn sha256(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

/// Writes the file in `folder`, whole: to a file beside it, then renamed over it, so the pack
/// never reads half of one.
pub fn write(folder: &Path, text: &str) -> io::Result<()> {
    let partial = folder.join(format!("{FILE}.new"));
    std::fs::write(&partial, text)?;
    std::fs::rename(&partial, folder.join(FILE))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_file_names_the_process_its_version_and_its_configuration_s_hash() {
        // sha256sum's answers, for the empty file and for one line.
        let cases: [(&[u8], &str); 2] = [
            (
                b"",
                "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
            ),
            (
                b"team = 1234\n",
                "6a5d27df0fe1c29c85dde1a3e70dca2aaf0c32305ae3598a612142d37193f7a0",
            ),
        ];
        for (config, hash) in cases {
            assert_eq!(
                text(42, "2026.11.8", config),
                format!("pid=42\nversion=2026.11.8\nconfig={hash}\n")
            );
        }
    }
}
