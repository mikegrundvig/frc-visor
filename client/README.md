# Visor client

The robot's side of [Visor](../README.md): a plain Java library with no dependencies, under the
BSD 3-Clause License ([LICENSE](LICENSE)). Visor itself is GPL; the robot only exchanges network
messages with it, so robot code never links GPL code.

- `VisorReceiver` receives frames on UDP 5807, on a thread of its own, and hands them to the robot
  loop through a fixed ring. `inMemory()` makes one a simulation feeds directly.
- `VisorFrame` is one frame, reused: receiving allocates nothing once running. Its `sender()` is
  the coprocessor it came from, and its `configId()` the configuration Visor ran as it sent it.
- `VisorStream` names a camera's live view on its coprocessor, `http://<board>:5809/stream/<camera>`
  ([docs/live-view.md](../docs/live-view.md)): the browser fetches it from the board directly.
- `CameraToTag` is a tag's pose from the camera in WPILib's axes (camera-to-tag).
- `NtpResponder` answers chrony on the coprocessors with the robot's clock (UDP 5806).
- `VisorConfigFile` writes Visor's configuration for a coprocessor, as a robot's deploy does,
  refusing each setting Visor would refuse, and always naming the port the robot receives on
  (and the tags' size, `tagSizeMeters`, when given: the field's):
  `testdata/written-config.toml` is its output, which Visor's own tests read back, and
  `least-config.toml` and `most-config.toml` every setting at its least and most, which they run
  through `visor check`.
- `NtpResponder` and `VisorReceiver` wait through errors that keep coming, rather than spin, and
  the responder answers on after the robot's clock throws.
- `VisorPack` names Visor's Spotter pack ([spotter/visor](../spotter/visor/README.md)) for Spotter's
  manager: its status values, which say what's on disk and what's running, its actions and their
  responses, the parsing of a status report, a configuration's hash as the status gives it and the
  ID frames carry for it, a release archive's name, and the install's input: the archive after
  the line naming its SHA-256. `VisorVersion.RELEASE_SHA256` is the SHA-256 of the archive the
  client was published with. `VisorPackTest` runs the pack's scripts and Spotter's real agent against
  it.

```java
VisorReceiver receiver = VisorReceiver.listen(VisorReceiver.DEFAULT_PORT);
NtpResponder clock = NtpResponder.start(NtpResponder.DEFAULT_PORT, RobotController::getTime);
VisorFrame frame = new VisorFrame();
CameraToTag pose = new CameraToTag();

// Every loop:
while (receiver.poll(frame)) {
  for (int tag = 0; tag < frame.tagCount(); tag++) {
    if (frame.posesSolved(tag)) {
      frame.cameraToTag(tag, Candidate.BEST, pose);
      // pose.x(), pose.y(), pose.z(), pose.qw() … in WPILib's axes; frame.ambiguity(tag)
    }
  }
}
```

```bash
./gradlew ci    # formatting, static analysis, tests, coverage
```
