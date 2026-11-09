//! `visor run`, the service, as the built program: how it ends, and what it doesn't end on.

use std::os::unix::process::ExitStatusExt;
use std::path::PathBuf;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::thread;
use std::time::{Duration, Instant};

/// How long a test waits for the program to end before calling it still running.
const PATIENCE: Duration = Duration::from_secs(10);

/// A camera that isn't plugged in, so its capture thread keeps trying to open it, as it would on a
/// board whose camera is unplugged.
const CAMERA: &str = r#"
    [[camera]]
    name = "front-left"
    index = 0
    device = "/dev/v4l/by-id/no-such-camera-video-index0"
    width = 1600
    height = 1304
"#;

/// The program, started on a configuration with frames going to `robot`. `panic_in` names a
/// camera whose detect thread is to panic (VISOR_TEST_PANIC, a debug build's fault switch).
fn start(test: &str, robot: &str, panic_in: Option<&str>) -> Child {
    let dir = std::env::temp_dir().join(format!("visor-run-{test}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let config: PathBuf = dir.join("visor.toml");
    std::fs::write(&config, format!("robot = \"{robot}\"\nport = 9\n{CAMERA}")).unwrap();
    let mut command = Command::new(env!("CARGO_BIN_EXE_visor"));
    command
        .arg("run")
        .arg("--config")
        .arg(&config)
        .stdout(Stdio::null())
        .stderr(Stdio::null());
    if let Some(camera) = panic_in {
        command.env("VISOR_TEST_PANIC", camera);
    }
    command.spawn().unwrap()
}

/// The program's exit status once it ends, or None if it's still running after `wait`; then it's
/// killed.
fn ended_within(child: &mut Child, wait: Duration) -> Option<ExitStatus> {
    let started = Instant::now();
    while started.elapsed() < wait {
        if let Some(status) = child.try_wait().unwrap() {
            return Some(status);
        }
        thread::sleep(Duration::from_millis(20));
    }
    child.kill().unwrap();
    child.wait().unwrap();
    None
}

#[test]
fn a_panic_in_one_camera_s_thread_ends_the_process() {
    // The other threads (the camera's capture and live view, and the live view's server) run on.
    // Unless the panic aborts, the process stays up with this camera dead, and systemd never
    // restarts it.
    let mut child = start("panic", "127.0.0.1", Some("front-left"));
    let status = ended_within(&mut child, PATIENCE).expect("the process ends");
    assert_eq!(status.signal(), Some(libc::SIGABRT), "{status}");
}

#[test]
fn no_route_to_the_robot_yet_is_not_a_reason_to_stop() {
    // A broadcast address can't be sent to without SO_BROADCAST, as an address with no route
    // can't be while the network comes up: each frame fails to send, and the cameras run.
    let mut child = start("no-route", "255.255.255.255", None);
    let ended = ended_within(&mut child, Duration::from_millis(1500));
    assert_eq!(ended, None, "the program ended");
}
