//! `visor version`: the version, its version ID, and the licence notices.

use visor::wire;

/// Prints the version, its version ID, and the licence notices.
pub fn version() {
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
