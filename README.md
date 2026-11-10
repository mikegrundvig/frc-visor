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

Visor is deliberately small: capture, detect, solve, send. No settings UI, no NetworkTables, no
field layout, no multi-tag solving. Its one page is each camera's live view, to aim and focus by.

## How it works

- **Per camera, two threads.** One captures (V4L2, memory-mapped), the other detects. They share
  one slot holding the newest frame, so a slow detection drops frames rather than falling behind.
- **USB recovery.** On any capture error the camera is closed and reopened every 300 ms, and its
  settings reapplied: its exposure and gain included ([below](#exposure-and-gain)).
- **The clock.** chrony keeps the coprocessor's clock on the robot program's, through a small NTP
  responder in the robot program (see [docs/wire.md](docs/wire.md#the-capture-time)). Each frame's
  kernel timestamp becomes robot time, plus a per-camera offset you measure.
- **Poses.** With a camera's lens calibration in the configuration, each tag's two poses are solved
  as OpenCV's IPPE does it (`SOLVEPNP_IPPE_SQUARE`): about 3 µs per tag on a laptop. Visor's
  undistortion runs until it converges, where OpenCV's stops at five steps.
- **The wire.** One UDP datagram per frame, every frame: [docs/wire.md](docs/wire.md).
- **Health.** Each frame says how long its detection took and how many of the camera's frames
  Visor has dropped (replaced before detection got to them, or broken), so the robot logs each
  camera's frame rate, drops and load ([docs/wire.md](docs/wire.md#the-cameras-health)). Every
  10 s Visor's journal says the same, one line per camera: `front: 59.8 fps, 0 dropped, detect 4.2
  ms mean, 9.8 ms worst, ...`.

- **Live view.** Each camera's frames, made smaller, with its tags outlined and a line of its
  numbers, over HTTP on port 5809 while anyone watches: nothing is made while no one does, and
  detection never waits for it ([below](#the-live-view)).

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
  The new configuration is checked (`visor check`) before it replaces the old. Each frame names
  the configuration it came from ([docs/wire.md](docs/wire.md#the-configuration-id)), so the robot
  refuses frames from one it didn't give the board.
- **Service:** [deploy/visor.service](deploy/visor.service), which runs as a `visor` user in the
  `video` group. Each release carries it under `os/` with the rest of what a board's image installs
  for Visor: the account, `/opt/visor`, and what Spotter's agent needs for Visor's pack
  ([spotter/visor](spotter/visor/README.md#what-the-boards-image-gives-it)). Visor tells systemd
  it's alive each 1.5 s while every camera's detect thread runs, a camera that's unplugged too;
  one that hangs for 3 s gets Visor restarted ([src/watchdog.rs](src/watchdog.rs)).
- **chrony:** [deploy/chrony.conf](deploy/chrony.conf), with the robot controller's address
  (`10.TE.AM.2`) in its `server` line.

### Checking the clock

With the robot program running, on the coprocessor:

```bash
chronyc sources     # the robot's line starts ^*: chrony has chosen it
chronyc tracking    # Reference ID names 10.TE.AM.2, Leap status Normal, System time under 0.001 s
timedatectl show -p NTPSynchronized   # yes: the kernel's clock is marked synced, as Visor reads it
```

That's the pass: frames are stamped in robot time, to well under the 1 ms Visor aims for (chrony
held about 0.1 ms on a test bench; a robot's network is yet to be measured). Anything else, and
Visor marks every frame unsynced: the robot leaves them all out and raises the camera's
no-capture-time alert. `^?` or no line means chrony can't reach the robot program: check the
`server` address and that the robot program is running, since it answers on UDP 5806. After the
robot controller reboots, chrony steps to its new clock within about a second (`makestep`).

### Exposure and gain

Visor sets each camera's exposure and gain from its configuration every time it opens the camera,
after a replug too, and has no default of its own: `exposure_us` turns the camera's automatic
exposure off and fixes the exposure; `gain` sets its `gain` control. Left out, Visor puts back the
camera's own automatic exposure and its default gain, so what a camera does follows from the
configuration alone. A robot's configuration gives them from the robot's description of each
camera, where the defaults live (frc-robot-runtime's docs/visor.md): a fixed 1 ms, since blur from
the robot turning is the main error and short exposures cut it, with the gain the camera's own
until a team sets one.

Visor finds the controls by V4L2's standard IDs (`auto_exposure`, its manual mode 1;
`exposure_time_absolute`, in 100 µs; `gain`), whatever the kernel names them. A camera without one,
a value outside its range, or a value the camera refuses or keeps otherwise (rounded to its step, or
an exposure held to its frame time), is a warning in Visor's log, said once, and the camera keeps
capturing at what it has. Any other control goes under `[camera.controls]` by the name
`v4l2-ctl --list-ctrls` prints, set after these; a name the camera doesn't have stops it opening.

`visor cameras` lists each camera's controls, with their ranges, defaults and values now: on a new
camera, that's the check of its names and its gain's range, without `v4l2-ctl`.

Every frame carries the exposure and gain in effect, read back from the camera, and how bright it
is: the median and 99th percentile of a sample of its pixels, and the share at full white
([docs/wire.md](docs/wire.md#exposure-and-brightness)). Every ten seconds, when most frames were
too dark to read tags far (the brightest 1% under 64 of 255) or had their whites clipping (over 1%
of the frame at 250 or more), Visor's log says so, with the exposure and gain, and what to change:
the exposure, the gain, or the light.

### The live view

`http://<coprocessor>:5809/` shows every camera's live view, and `/stream/<camera>` is one camera's
by its name (escaped as an address needs: `/stream/front%20left`), an MJPEG stream any browser
shows in an `<img>`. A robot's Pitwall shows each camera's on its card. The picture is the camera's
frame at half its size (800 x 652 of a ThriftyCam), 10 a second at JPEG quality 60: about 2 Mbit/s,
20 to 25 KB a picture. Each tag found is outlined with its ID, and the line along the top gives
detection's frame rate, the exposure and gain, and the brightness (median / 99th percentile). Each
camera's `stream_scale`, `stream_fps` and `stream_quality` change it
([deploy/visor.example.toml](deploy/visor.example.toml)).

It never costs detection. With no one watching, nothing is made. While someone watches, the view's
own thread asks for a frame at the view's rate, the detect thread hands over its next one after
sending it to the robot, without waiting, and the picture is made on that thread at a lower
priority (nice 10): about 2.2 ms a picture on a laptop core, about 2% of one core at the defaults.
Each watcher gets the newest picture when its connection is ready for one, so a slow watcher skips
pictures; one that stops reading is closed after 5 s, and at most 8 watch at once. More in
[docs/live-view.md](docs/live-view.md).

## Releases

A release is a date, `year.month.day` in Cargo.toml, and its version ID is that date as one number;
the robot client is released at the same version. Pushing the tag `v<version>` runs
[the release workflow](.github/workflows/release.yml), which builds and tests the program on an
arm64 runner and attaches `visor-<version>-linux-arm64.tar.gz` (the program, its licence and
notices, the chrony configuration, the Spotter pack, and `os/`, what a board's image installs;
[tools/package.sh](tools/package.sh) makes it) and its SHA-256 to the tag's GitHub release. `tools/release.sh` then publishes the client and that archive into the FRC
libraries' Maven repository, from which a robot's build takes the archive for its client. It runs
from a clean checkout of the tag and refuses otherwise, so the client is built from the same code as
the program. The client carries the archive's SHA-256 (`VisorVersion.RELEASE_SHA256`): a robot's
build stages only an archive with that hash, and a board installs only one, the robot naming the
hash with the archive.

## Lens calibration

Each camera needs its lens calibrated once, at the frame size it runs, for Visor to solve poses.
The tool is [tools/calibrate.py](tools/calibrate.py), run on a laptop with
[uv](https://docs.astral.sh/uv/) (it fetches OpenCV itself):

```bash
uv run tools/calibrate.py board --out board.png
```

A bought ChArUco board works too: pass its `--squares`, `--square-mm`, `--marker-mm` and
`--dictionary` to both `board` and `solve` (a 15 x 15 board of 1.125 in squares and 0.875 in
markers is `--squares 15x15 --square-mm 28.575 --marker-mm 22.225 --dictionary 4X4_1000`), and
skip printing.

With the camera plugged into a Linux laptop instead of a coprocessor, `uv run tools/capture.py
<device> --out frames/` (with the board's options) is a live view for both steps. It opens paused,
for focusing: a bar shows how sharp the middle of the frame is against the best seen, `f` zooms the
middle to full size, and `r` resets the peak; turn the lens to the peak and lock it. Then `s` starts
recording: it shows the board's corners, its rough distance and tilt, each ninth of the frame's
coverage and how far the corners reach toward each corner of the frame, and saves a frame every
1.5 s while the board is steady. Solve its folder as below.

A good session has the board's corners reaching every corner of the frame (a partly visible board
counts), the board tilted 30 to 45 degrees about both axes, top toward and away as well as the
sides, at several places in the frame, at two distances, and 40 to 60 views. Corners only in the
middle of the frame leave the lens a guess out at its edges, where the distortion is largest.

1. Print `board.png` at 100% scale and fix it flat to something rigid.
2. On the coprocessor, `visor cameras` lists each camera plugged in: the device to give Visor, and
   the formats and sizes it offers. Each camera is listed once, by the video device it captures on:
   the kernel's first for its USB device, the one udev's names end `-video-index0`. Its others (a
   UVC camera's second, for metadata) are left out, told from the kernel's description of each
   rather than by opening it. `visor cameras --names` prints only the names, one a line, and opens
   no camera: it's what Visor's Spotter pack reports to the robot. A camera with a USB serial is
   named by its `/dev/v4l/by-id/` name, which follows the camera to any port. A camera with no
   serial (the ThriftyCam has none) is named by its USB port, its `/dev/v4l/by-path/` name, always,
   even when it's the only one plugged in: every camera of its model shares one by-id name, which
   breaks when a second is plugged in. So is a camera whose serial is another's. A camera named by
   its port must go back in that port: label the ports and the cables.
3. Stop the service, which has the cameras open, and record at the size the camera is to run:

   ```bash
   sudo systemctl stop visor
   visor record --device /dev/v4l/by-path/platform-...-video-index0 --width 1280 --height 800
   sudo systemctl start visor
   ```

   It saves 40 frames, one a second, to `/tmp/visor-frames/` (in RAM, gone at the next boot)
   while you move the board: near and far, tilted every way, and into every part of the frame,
   its corners and edges above all. Once the robot has configured the board, `--camera <name>`
   takes the camera's device, size and settings from the configuration instead, by the camera's
   name in the robot program.
4. Copy the frames to the laptop and solve:

```bash
scp '<user>@<coprocessor>:/tmp/visor-frames/*.pgm' frames/
uv run tools/calibrate.py solve frames/*.pgm --out lens.toml
```

It prints the error of each view and how well each part of the frame was covered, and writes a
`[camera.lens]` block for that camera's entry in the configuration. It refuses a session it can't
trust (too few frames with the board, part of the frame never reached, too few corners out toward
one of the frame's corners, or too large an error) and says which. The distortion has five terms:
the tool solves the lens with the fifth (k3) held at 0 and with it free, keeps it at 0 when the two
put every point within a pixel of each other, and keeps it otherwise, saying which and why.
`--free-k3` keeps it without comparing, and `--rational` solves eight terms, for a wide lens with
strong distortion. Then check a tag at a measured
distance with `visor detect --config ... --camera ...`.

Its tests (`uv run tools/test_calibrate.py`) render boards through lenses whose values are known,
including a strongly distorted one whose k3 the tool must keep, and check the tool gives them back;
then a tag drawn through that lens near the frame's corner, solved by Visor with the tool's block,
lands within 1 mm of its true place 4.3 m out, where leaving the distortion out puts it 63 cm off.
`uv run tools/test_capture.py` tests the live view's choices of which frames to save.

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
