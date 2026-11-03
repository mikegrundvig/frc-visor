# Visor client

The robot's side of [Visor](../README.md): a plain Java library with no dependencies, under the
BSD 3-Clause License ([LICENSE](LICENSE)). Visor itself is GPL; the robot only exchanges network
messages with it, so robot code never links GPL code.

- `VisorReceiver` receives frames on UDP 5807, on a thread of its own, and hands them to the robot
  loop through a fixed ring. `inMemory()` makes one a simulation feeds directly.
- `VisorFrame` is one frame, reused: receiving allocates nothing once running.
- `CameraToTag` is a tag's pose from the camera in WPILib's axes (camera-to-tag).
- `NtpResponder` answers chrony on the coprocessors with the robot's clock (UDP 5806).
- `VisorConfigFile` writes Visor's configuration for a coprocessor, as a robot's deploy does:
  `testdata/written-config.toml` is its output, which Visor's own tests read back.

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
