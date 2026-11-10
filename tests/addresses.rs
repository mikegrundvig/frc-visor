//! The ports and the live view's paths Visor and the robot client must agree on, from
//! testdata/addresses.json, which the client's VisorStreamTest reads too: the port frames go to, the
//! live view's port, and each camera's stream's path, its name escaped.

use std::path::Path;

use serde_json::Value;
use visor::config::DEFAULT_PORT;
use visor::stream::{self, server};

#[test]
fn the_ports_and_stream_paths_are_the_ones_the_client_uses() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("testdata/addresses.json");
    let shared: Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    assert_eq!(shared["frames_port"], u64::from(DEFAULT_PORT));
    assert_eq!(shared["live_view_port"], u64::from(stream::PORT));
    for row in shared["streams"].as_array().unwrap() {
        let camera = row["camera"].as_str().unwrap();
        assert_eq!(server::stream_path(camera), row["path"], "{camera}");
    }
}
