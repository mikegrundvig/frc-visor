# Visor's wire format

Visor sends one UDP datagram per captured frame to the robot controller, port 5807 by default. A
frame is sent for every image, with or without tags, so silence means a camera or Visor is down.
A lost datagram is simply lost: the next frame is 16 ms behind it.

Everything is little-endian, with fixed sizes and no padding: a 40-byte header, then 96 bytes per
tag. Eight tags is 808 bytes; 14, the most a frame carries, is 1,384, inside one Ethernet packet's
1,472 bytes of UDP payload.

## Header

| Offset | Field | Type | Notes |
|---|---|---|---|
| 0 | Magic | u32 | `0x52534956`, the bytes `V I S R`; anything else isn't a Visor frame |
| 4 | Version ID | u32 | Visor's version as a number: 2026.11.2 is 20261102 |
| 8 | Sequence | u32 | Counts this camera's frames from Visor's start; a gap is a dropped frame |
| 12 | Width | u16 | The image's, in pixels |
| 14 | Height | u16 | |
| 16 | Capture time | i64 | Nanoseconds on the robot's clock (below) |
| 24 | Camera | u8 | The camera's index from Visor's configuration, unique on the robot |
| 25 | Flags | u8 | Bit 0: the clock is synced to the robot. Bit 1: the capture time is when Visor got the frame, because the driver gave no timestamp. Bit 2: the exposure below is known. Bit 3: the gain below is known. Bit 4: the camera's own automatic exposure is on. Other bits are 0 |
| 26 | Tag count | u8 | 0 to 14; with more in view, the 14 with the highest decision margins |
| 27 | Reserved | u8 | 0 |
| 28 | Exposure | u32 | Microseconds, as read back from the camera once its settings were set; 0 without bit 2 (its automatic exposure on, so the camera chooses it, or no exposure control) |
| 32 | Gain | i32 | The camera's `gain` control, in its own units, as read back; 0 without bit 3 (no gain control) |
| 36 | Median brightness | u8 | The image's middle pixel, 0 (black) to 255 (white), from every fourth pixel of every fourth row |
| 37 | High brightness | u8 | Its 99th percentile, about a tag's white where one is in view |
| 38 | Clipped | u16 | The share of those pixels at 250 or brighter, in hundredths of a percent: 0 to 10,000 |

## Each tag

| Offset | Field | Type | Notes |
|---|---|---|---|
| 0 | ID | u16 | tag36h11 |
| 2 | Hamming | u8 | Payload bits NAPA corrected |
| 3 | Flags | u8 | Bit 0: the poses below were solved. Without it (no lens calibration, or corners that can't be solved) they're zeros |
| 4 | Decision margin | f32 | Higher is clearer |
| 8 | Corners | f32 × 8 | x0, y0, x1, y1, x2, y2, x3, y3 |
| 40 | Best pose: rotation | f32 × 3 | Axis times angle, radians (OpenCV's rvec) |
| 52 | Best pose: translation | f32 × 3 | Metres (OpenCV's tvec) |
| 64 | Other pose: rotation | f32 × 3 | |
| 76 | Other pose: translation | f32 × 3 | |
| 88 | Best pose's error | f32 | Root-mean-square reprojection error per coordinate, pixels |
| 92 | Other pose's error | f32 | |

**Corners** are in OpenCV's order (the printed tag's top-left, top-right, bottom-right,
bottom-left) and in pixel-centre coordinates: pixel (0, 0)'s centre is at (0, 0). WPILib's AprilTag
detector reports corners half a pixel from that, and in a different order; a client converts once.
They're raw: not undistorted, so a replay can solve them again with a corrected calibration.

**Poses** are the tag in the camera's frame, OpenCV's axes: x right, y down, z out of the lens. The
tag's own frame, centred on it, has x right and y up as it's printed, and z out of its face, toward
the camera. A flat square fits two poses,
mirror images about the line of sight; both come, the better first, as OpenCV's
`solvePnPGeneric` with `SOLVEPNP_IPPE_SQUARE` gives them (Visor's solve matches OpenCV's to about
1e-10 from the same undistorted corners). The usual ambiguity is the best error over the
other's. A client converts to its own axes once.

## Exposure and brightness

The exposure and gain are what the camera reports, read back as Visor opens it (and again after a
replug), so a frame says what it was taken at, not only what was asked for. Where a control can't
be read back, it's what Visor set. The brightness is the image's own, from a sample of one pixel in
16, so a log shows whether the exposure and gain suit the light: tag whites at a high brightness of
100 to 200 with nothing clipped is a well-exposed frame.

## The capture time

The robot program answers NTP requests on UDP 5806 with its own clock plus 2026-01-01T00:00:00Z
(1,767,225,600 s), and chrony on the coprocessor keeps the wall clock on that. Visor converts each
frame's kernel timestamp to the wall clock, subtracts that epoch, and adds the camera's configured
offset. So the capture time is directly comparable to `RobotController.getTime()`.

A frame without bit 0 set came from an unsynced clock, and its time means nothing.

## The golden file

`testdata/golden-frame.bin` is one encoded frame. Visor's tests check their encoding against it
byte for byte, and a client's tests should decode it and check every field (`src/wire.rs` has the
values):

- version ID 20261102, sequence 4,000,000,001, 1600 × 1304, capture time 1,234,567,890,123 ns,
  camera 3, clock synced, driver timestamp, exposure 1,000 µs and gain 40 known, the camera's
  automatic exposure off, median brightness 37, high 201, clipped 125 (1.25%);
- tag 7, hamming 0, margin 61.25, corners (100.5, 200.25) (180, 201) (179.75, 280.5) (99, 279),
  poses solved: best rvec (0.125, -2.5, 0.0625), tvec (-0.5, 0.25, 3.75), error 0.1875; other
  rvec (-0.25, 2, -0.5), tvec (-0.5, 0.25, 3.5), error 1.5;
- tag 586, hamming 2, margin 12.5, corners (1500.125, 1200) (1580, 1201.5) (1579, 1290.25)
  (1499.5, 1289), no poses.

Visor and the robot are deployed together, so the format can change freely between versions; the
golden file catches one side changing without the other.
