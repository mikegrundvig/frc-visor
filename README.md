# Visor

Visor is AprilTag vision for an FRC robot's coprocessor. It captures frames from USB cameras, finds
tags with [NAPA](https://github.com/zwatx/NAPA), solves each tag's two candidate poses, stamps each
frame with its capture time on the robot's clock, and sends it to the robot over UDP, raw corners
included. The robot chooses between the poses and does the rest.

**Visor hasn't run on robot hardware yet.** It's built and tested against rendered frames; it
needs real cameras and coprocessors.

## Why

NAPA finds tags much faster than the AprilTag detector in WPILib and PhotonVision, and much
farther away. On rendered 1600 × 1304 frames it took about 3 ms on one laptop core and found tags
to 10 m, where the usual detector at its fastest setting found them to about 3 m. On a
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
visor run --config /etc/visor/visor.toml    # the service
visor detect --repeat 100 frame.pgm         # what NAPA finds in an image, and how long it takes
visor detect --lens 1230,1230,799.5,651.5 frame.pgm   # and each tag's pose, with this lens
visor record --camera front-left --out frames/   # frames for lens calibration
visor version
```

- **Configuration:** [deploy/visor.example.toml](deploy/visor.example.toml).
- **Service:** [deploy/visor.service](deploy/visor.service), which runs as a `visor` user in the
  `video` group.
- **chrony:** [deploy/chrony.conf](deploy/chrony.conf).

Camera controls use the names `v4l2-ctl --list-ctrls` prints. Aim for an exposure near 1 ms with
the gain raised: blur from the robot turning is the main error, and short exposures cut it.

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
