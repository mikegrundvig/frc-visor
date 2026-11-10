//! `visor run`, the service, as the built program: how it ends, and what it doesn't end on.

use std::os::unix::process::ExitStatusExt;
use std::path::{Path, PathBuf};
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

/// The program, started on a configuration with frames going to `robot`, and the directory the
/// configuration is in. `panic_in` names a camera whose detect thread is to panic
/// (VISOR_TEST_PANIC, a debug build's fault switch).
fn start(test: &str, robot: &str, panic_in: Option<&str>) -> (Child, PathBuf) {
    let config = format!("robot = \"{robot}\"\nport = 9\n{CAMERA}");
    let panic: Vec<(&str, &str)> = panic_in
        .map(|camera| ("VISOR_TEST_PANIC", camera))
        .into_iter()
        .collect();
    start_with(test, &config, &panic)
}

/// The program, started on the configuration `config` with `env` beside its own environment, and
/// the directory the configuration is in.
fn start_with(test: &str, config: &str, env: &[(&str, &str)]) -> (Child, PathBuf) {
    let dir = std::env::temp_dir().join(format!("visor-run-{test}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path: PathBuf = dir.join("visor.toml");
    std::fs::write(&path, config).unwrap();
    let mut command = Command::new(env!("CARGO_BIN_EXE_visor"));
    command
        .arg("run")
        .arg("--config")
        .arg(&path)
        .envs(env.iter().copied())
        .stdout(Stdio::null())
        .stderr(Stdio::null());
    (command.spawn().unwrap(), dir)
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
    let (mut child, dir) = start("panic", "127.0.0.1", Some("front-left"));
    let status = ended_within(&mut child, PATIENCE);
    std::fs::remove_dir_all(dir).unwrap();
    let status = status.expect("the process ends");
    assert_eq!(status.signal(), Some(libc::SIGABRT), "{status}");
}

#[test]
fn no_route_to_the_robot_yet_is_not_a_reason_to_stop() {
    // A broadcast address can't be sent to without SO_BROADCAST, as an address with no route
    // can't be while the network comes up: each frame fails to send, and the cameras run.
    let (mut child, dir) = start("no-route", "255.255.255.255", None);
    let ended = ended_within(&mut child, Duration::from_millis(1500));
    std::fs::remove_dir_all(dir).unwrap();
    assert_eq!(ended, None, "the program ended");
}

/// What systemd hears from the program on its notification socket, as systemd would run it with a
/// watchdog of `watchdog`: each message, until `listening` has passed.
fn told_systemd(test: &str, config: &str, watchdog: Duration, listening: Duration) -> Vec<String> {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("notify-{test}"));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("notify");
    let socket = std::os::unix::net::UnixDatagram::bind(&path).unwrap();
    socket
        .set_read_timeout(Some(Duration::from_millis(50)))
        .unwrap();
    let micros = watchdog.as_micros().to_string();
    let env = [
        ("NOTIFY_SOCKET", path.to_str().unwrap()),
        ("WATCHDOG_USEC", micros.as_str()),
    ];
    let (mut child, config_dir) = start_with(test, config, &env);
    let mut heard = Vec::new();
    let started = Instant::now();
    let mut buffer = [0u8; 256];
    while started.elapsed() < listening {
        if let Ok(n) = socket.recv(&mut buffer) {
            heard.push(String::from_utf8_lossy(&buffer[..n]).trim_end().to_string());
        }
    }
    let ended = child.try_wait().unwrap();
    child.kill().unwrap();
    child.wait().unwrap();
    std::fs::remove_dir_all(config_dir).unwrap();
    std::fs::remove_dir_all(dir).unwrap();
    assert_eq!(ended, None, "{test}: the program ended");
    heard
}

#[test]
fn visor_tells_systemd_it_s_ready_then_alive_while_its_detect_threads_run() {
    let watchdog = Duration::from_millis(400);
    let listening = Duration::from_secs(2);
    // (a short name for the row's files, the board, its configuration)
    let rows = [
        (
            "absent",
            "a camera absent: its capture thread retries, its detect thread waits",
            format!("robot = \"127.0.0.1\"\nport = 9\n{CAMERA}"),
        ),
        (
            "none",
            "no camera configured: it sits idle",
            "robot = \"127.0.0.1\"\nport = 9\n".to_string(),
        ),
    ];
    for (short, board, config) in rows {
        let heard = told_systemd(short, &config, watchdog, listening);
        assert_eq!(
            heard.first().map(String::as_str),
            Some("READY=1"),
            "{board}: {heard:?}"
        );
        // Every half of the watchdog's time: 10 in 2 s, less a few for a busy machine.
        let alive = heard.iter().filter(|said| *said == "WATCHDOG=1").count();
        assert!(alive >= 5, "{board}: {heard:?}");
        assert_eq!(alive + 1, heard.len(), "{board}: {heard:?}");
    }
}

/// The service as systemd runs it, with its runtime folder (RUNTIME_DIRECTORY) in a folder of the
/// test's, `root/run`, and installed as the pack finds it: the program and its configuration in
/// `root/opt-visor`.
fn start_as_a_service(test: &str) -> (Child, PathBuf) {
    let root = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("service-{test}"));
    let _ = std::fs::remove_dir_all(&root);
    let installed = root.join("opt-visor");
    std::fs::create_dir_all(&installed).unwrap();
    std::fs::create_dir_all(root.join("run")).unwrap();
    std::os::unix::fs::symlink(env!("CARGO_BIN_EXE_visor"), installed.join("visor")).unwrap();
    let config = installed.join("visor.toml");
    std::fs::write(
        &config,
        format!("robot = \"127.0.0.1\"\nport = 9\n{CAMERA}"),
    )
    .unwrap();
    let child = Command::new(installed.join("visor"))
        .arg("run")
        .arg("--config")
        .arg(&config)
        .env("RUNTIME_DIRECTORY", root.join("run"))
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    (child, root)
}

/// The pack's status, as Spotter's agent runs it on a board laid out as in `root`, with
/// `main_pid` as the unit's main process ID.
fn status(root: &Path, main_pid: u32) -> serde_json::Value {
    let says = root.join(format!("main-pid-{main_pid}"));
    std::fs::write(&says, format!("#!/bin/sh\necho {main_pid}\n")).unwrap();
    std::fs::set_permissions(&says, std::os::unix::fs::PermissionsExt::from_mode(0o755)).unwrap();
    let out = Command::new(concat!(env!("CARGO_MANIFEST_DIR"), "/spotter/visor/status"))
        .env("VISOR_DIR", root.join("opt-visor"))
        .env("VISOR_RUN", root.join("run"))
        .env("VISOR_MAIN_PID", &says)
        .env("VISOR_BY_ID", root.join("by-id"))
        .env("VISOR_BY_PATH", root.join("by-path"))
        .env("VISOR_SYSFS", root.join("sys"))
        .output()
        .unwrap();
    assert!(out.status.success(), "{out:?}");
    serde_json::from_slice(&out.stdout).unwrap()
}

/// A file's SHA-256 as `sha256sum` gives it, as the pack hashes the configuration on disk.
fn sha256sum(file: &Path) -> String {
    let out = Command::new("sha256sum").arg(file).output().unwrap();
    let text = String::from_utf8(out.stdout).unwrap();
    text.split(' ').next().unwrap().to_owned()
}

#[test]
fn the_pack_reports_what_the_running_service_runs_beside_what_is_on_disk() {
    let (mut child, root) = start_as_a_service("running");
    let file = root.join("run/running");
    let started = Instant::now();
    while !file.exists() && started.elapsed() < PATIENCE {
        thread::sleep(Duration::from_millis(20));
    }
    let config = root.join("opt-visor/visor.toml");
    let on_disk = sha256sum(&config);
    let pid = child.id();
    assert_eq!(
        std::fs::read_to_string(&file).unwrap(),
        format!(
            "pid={pid}\nversion={}\nconfig={on_disk}\n",
            env!("CARGO_PKG_VERSION")
        ),
        "the file it writes as it starts"
    );
    let version = env!("CARGO_PKG_VERSION");
    // Each row: what changes, the unit's main process ID, and the running version and
    // configuration the pack then reports.
    let other = pid + 1;
    let rows: [(&str, Option<&str>, u32, &str, bool); 4] = [
        ("the service runs what's on disk", None, pid, version, true),
        (
            "a process that has ended left the file: another is the unit's main process now",
            None,
            other,
            "",
            false,
        ),
        (
            "the service isn't running: no main process",
            None,
            0,
            "",
            false,
        ),
        (
            "a new configuration was put in place, but the restart failed",
            Some("robot = \"127.0.0.1\"\nport = 10\n"),
            pid,
            version,
            true,
        ),
    ];
    for (why, new_config, main_pid, running_version, running) in rows {
        if let Some(text) = new_config {
            std::fs::write(&config, format!("{text}{CAMERA}")).unwrap();
        }
        let report = status(&root, main_pid);
        assert_eq!(report["version"], version, "{why}");
        assert_eq!(report["config"], sha256sum(&config), "{why}");
        assert_eq!(report["running-version"], running_version, "{why}");
        let running_config = if running { on_disk.as_str() } else { "" };
        assert_eq!(report["running-config"], running_config, "{why}");
    }
    child.kill().unwrap();
    child.wait().unwrap();
    std::fs::remove_dir_all(root).unwrap();
}
