# Visor

Visor is AprilTag vision for an FRC robot's coprocessor. It captures frames from USB cameras, finds
tags with [NAPA](https://github.com/zwatx/NAPA), solves each tag's two candidate poses, stamps each
frame with its capture time on the robot's clock, and sends it to the robot over UDP, raw corners
included. The robot chooses between the poses and does the rest.

**Visor hasn't run on robot hardware yet.** It's built and tested against rendered frames; it
needs real cameras and coprocessors.

## Why

NAPA finds tags much faster than WPILib's AprilTag detector, and much farther away. On rendered
1600 × 1304 frames it took about 3 ms on one laptop core and found tags to 10 m, where WPILib's
detector at decimate 2 found them to about 3 m. On a
coprocessor that range is the difference between seeing a tag from most of the field and not.

Visor is deliberately small: capture, detect, solve, send. No web UI, no NetworkTables, no field
layout, no multi-tag solving.

## How it works

- **Per camera, two threads.** One captures (V4L2, memory-mapped), the other detects. They share
  one slot holding the newest frame, so a slow detection drops frames rather than falling behind.
- **USB recovery.** On any capture error the camera is closed and reopened every 300 ms, and its
  settings reapplied.
- **The clock.** chrony keeps the coprocessor's clock on the robot program's, through a small NTP
  responder in the robot program (see [docs/wire.md](docs/wire.md#the-capture-time)). Each frame's
  kernel timestamp becomes robot time, plus a per-camera offset you measure.
- **Poses.** With a camera's lens calibration in the configuration, each tag's two poses are solved
  as OpenCV's IPPE does it (`SOLVEPNP_IPPE_SQUARE`): about 3 µs per tag on a laptop. Visor's
  undistortion runs until it converges, where OpenCV's stops at five steps.
- **The wire.** One UDP datagram per frame, every frame: [docs/wire.md](docs/wire.md).

## Building

Rust (stable), a C compiler, and libclang for the V4L2 bindings:

```bash
sudo apt-get install -y clang libclang-dev
cargo build --release
cargo test
```

The binary is `target/release/visor`. Build on the coprocessor's architecture (arm64 for an Orange
Pi); NAPA uses SSE2 on x86-64 and NEON on arm64.

## Running

```bash
visor run                                  # the service (--config /opt/visor/visor.toml)
visor check                                # is the configuration valid, and what it sets up
visor detect --repeat 100 frame.pgm         # what NAPA finds in an image, and how long it takes
visor detect --lens 1230,1230,799.5,651.5 frame.pgm   # and each tag's pose, with this lens
visor detect --config visor.toml --camera front-left frame.pgm   # with a configured camera's lens
visor record --camera front-left --out frames/   # frames for lens calibration
visor version
```

- **Configuration:** [deploy/visor.example.toml](deploy/visor.example.toml), at
  `/opt/visor/visor.toml` beside the program.
- **Installed by the robot:** a robot built on
  [frc-robot-runtime](https://github.com/mikegrundvig/frc-robot-runtime) carries Visor's release
  and, as it starts, checks each coprocessor through [Spotter](https://github.com/mikegrundvig/frc-spotter)'s
  agent with Visor's pack ([spotter/visor](spotter/visor/README.md)): it installs the release where
  the board has another, and writes the configuration from the cameras the board has plugged in.
  The new configuration is checked (`visor check`) before it replaces the old.
- **Service:** [deploy/visor.service](deploy/visor.service), which runs as a `visor` user in the
  `video` group.
- **chrony:** [deploy/chrony.conf](deploy/chrony.conf).

Camera controls use the names `v4l2-ctl --list-ctrls` prints. Aim for an exposure near 1 ms with
the gain raised: blur from the robot turning is the main error, and short exposures cut it.

## Releases

A release is a date, `year.month.day` in Cargo.toml, and its version ID is that date as one number;
the robot client is released at the same version. Pushing the tag `v<version>` runs
[the release workflow](.github/workflows/release.yml), which builds and tests the program on an
arm64 runner and attaches `visor-<version>-linux-arm64.tar.gz` (the program, its licence and
notices, the systemd unit, the chrony configuration and the Spotter pack) and its SHA-256 to the
tag's GitHub release. `tools/release.sh` then publishes the client and that archive into the FRC
libraries' Maven repository, from which a robot's build takes the archive for its client.

## Lens calibration

Each camera needs its lens calibrated once, at the frame size it runs, for Visor to solve poses.
The tool is [tools/calibrate.py](tools/calibrate.py), run on a laptop with
[uv](https://docs.astral.sh/uv/) (it fetches OpenCV itself):

```bash
uv run tools/calibrate.py board --out board.png
```

1. Print `board.png` at 100% scale and fix it flat to something rigid.
2. On the coprocessor, stop the service and run `visor record --camera front-left --out frames/`.
   It saves 40 frames, one a second, while you move the board: near and far, tilted every way,
   and into every part of the frame, its corners and edges above all.
3. Copy the frames to the laptop and solve:

```bash
uv run tools/calibrate.py solve frames/*.pgm --out lens.toml
```

It prints the error of each view and how well each part of the frame was covered, and writes a
`[camera.lens]` block for that camera's entry in the configuration. It refuses a session it can't
trust (too few frames with the board, part of the frame never reached, or too large an error) and
says which. Add `--rational` for a wide lens with strong distortion. Then check a tag at a measured
distance with `visor detect --config ... --camera ...`.

Its tests (`uv run tools/test_calibrate.py`) render boards through lenses whose values are known,
including a strongly distorted one, and check the tool gives them back; then a tag drawn through
that lens near the frame's corner, solved by Visor with the tool's block, lands within 1 mm of its
true place 4.3 m out, where leaving the distortion out puts it 63 cm off.

## The robot's side

[client/](client/README.md) is a Java library for the robot program: it receives frames on its own
thread and hands them to the loop without allocating, answers chrony with the robot's clock, and
gives each pose in WPILib's axes. A simulation can feed it frames through the same path.

## Licence

Visor is free software under the GNU General Public License, version 3 only: see
[LICENSE](LICENSE) and [NOTICE](NOTICE). It includes NAPA, by Zachary Wilson, under the same
licence, in [vendor/napa](vendor/napa).

The robot only exchanges network messages with Visor, so robot code doesn't link Visor's code. The
client in [client/](client) is under the BSD 3-Clause License instead.
