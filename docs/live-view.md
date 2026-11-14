# Visor's live view

Each camera's live view is served over HTTP on port 5809 of its coprocessor, inside FRC's 5800 to
5810 for teams, beside Spotter's agent on 5808. It's for the pit: aiming a camera, focusing it, and
seeing what it sees and what Visor finds in it. The same server gives one frame at full size when
asked, and, on a bench, takes an exposure and gain to try.

## Addresses

| Address | What |
|---|---|
| `/` | A page with every camera's live view, by name |
| `/stream/<camera>` | One camera's view as an MJPEG stream: `multipart/x-mixed-replace; boundary=frame`, each part one JPEG with its `Content-Type` and `Content-Length` |
| `/frame/<camera>` | The camera's next frame at full size, as detection read it: a binary PGM, with what Visor found in it in the `X-Visor-Frame` header (below) |
| `/settings/<camera>` | `POST` only, and only with `try_settings = true`: an exposure and gain to try (below) |

A camera is named as in Visor's configuration, which a robot's deploy writes from the robot's
definition of it, escaped as an address needs: `front left` is `/stream/front%20left`. Any other
name is a 404 that lists the cameras Visor has. Every browser shows the stream in an `<img>`, which
is how Pitwall's camera card shows it; so does `ffplay` or VLC.

## The picture

The camera's frame, made smaller by averaging (half its size unless configured: 800 x 652 of a
ThriftyCam's 1600 x 1304), in grey as Visor sees it. Each tag Visor found in that frame is outlined,
white over black, with its ID in its middle. The line along the top gives:

- detection's frame rate, over about the last second;
- the exposure in effect, as read back from the camera, or `auto exposure` while the camera's own
  is on;
- the gain, or `no gain` for a camera without the control;
- the frame's brightness, its median and its 99th percentile (docs/wire.md, "Exposure and
  brightness");
- its focus: how steep the steps between neighbouring pixels are, on average. An edge in focus is a
  steep step, so the number peaks as the lens comes into focus: turn the lens, watch it climb, and
  stop when it falls. Noise is steps too, and so is whatever is in view, so it only compares one
  scene with itself at one exposure and gain. It says nothing between two cameras or two rooms.

## A frame at full size

`GET /frame/<camera>` answers with the camera's next frame exactly as detection read it: every
pixel, not made smaller, not compressed, nothing drawn on it. It's a binary PGM (`P5`, 8-bit;
`image/x-portable-graymap`), which `tools/calibrate.py` and `visor detect` read, and most image
viewers open. A ThriftyCam's is 2 MB.

The `X-Visor-Frame` header says what that frame was taken at and what Visor found in it, as one
line of JSON:

```json
{"camera":"front-left","sequence":18021,"width":1600,"height":1304,"exposure_us":1000,"gain":2,
 "auto_exposure":false,"brightness_median":37,"brightness_high":201,"clipped_percent":1.25,
 "focus":12.346,"tags":[{"id":7,"hamming":0,"decision_margin":81.5,
 "corners":[610.2,480.9,702.4,481.3,702.0,573.8,609.7,573.1],"distance_m":2.013,"pose_error_px":0.21}]}
```

`sequence` is the frame's number as sent to the robot, so it's the same frame the robot got. A
value the camera can't give is `null` (no gain control; automatic exposure on). `corners` are as
on the wire: x0, y0 to x3, y3, in pixel-centre coordinates, raw. `distance_m` is from the lens to
the tag's centre by its best pose, and `null` without a lens calibration. So a frame the detector
missed a tag in, saved with this header, is a test case.

It costs detection nothing, as the view costs it nothing: the frame is the one detection hands
over after sending it to the robot, copied on the view's thread. But it's a lot of network: one
frame is half a second of a field's 4 Mbit/s. So each camera gives one at most every half second,
one request at a time, and each request takes one of the watchers' 8 places while it waits. Ask
for one when it's wanted, in the pit or on a bench; it isn't a stream. A camera sending no frames
answers 503 after 2 s.

## Trying an exposure and gain

On a robot, a camera's exposure and gain are the robot program's: it writes them into Visor's
configuration, and Visor sets them every time it opens the camera. That doesn't change. For a
bench, where finding good values means trying many, a configuration can say `try_settings = true`.
Then:

```
POST /settings/<camera>?exposure_us=1500&gain=2
```

sets either or both on the running camera, before its next frame, held to the camera's own range.
The page at `/` gets a small form for it under each camera. Every frame after says what's in
effect, read back from the camera: the view's line, a full-size frame's header, and the frames
sent to the robot.

A trial lasts until the camera is opened again: Visor restarted, or the camera replugged. Then the
configuration's values are set, as always. What's found by trying is kept by writing it into the
robot program.

Without `try_settings = true` the answer is 403, and the page has no form. A robot's deploy never
writes that key. A request carrying another site's `Origin` is refused, so a web page elsewhere
can't change a camera through a laptop on the board's network.

## Settings

Each camera's, in its `[[camera]]` entry, beside `exposure_us` and `gain`:

| Key | Default | What |
|---|---|---|
| `stream_scale` | `0.5` | The picture's size as a share of the camera's, across and down: 0.05 to 1 |
| `stream_fps` | `10`, or the camera's `fps` if that's lower | Pictures a second while someone watches: 1 to the camera's `fps` |
| `stream_quality` | `60` | JPEG quality, 1 to 100 |

At the defaults a picture of a ThriftyCam's frame is 20 to 25 KB, about 2 Mbit/s at 10 a second.
`visor check` prints each camera's view: its size, rate and quality.

## What it costs, and what it never does

Detection comes first, and the view never holds it up or makes it drop a frame:

- **No one watching:** nothing is made. The detect thread checks one flag after sending each frame,
  and the view's thread looks for a watcher ten times a second.
- **Someone watching:** the view's thread asks for a frame once a picture's period (100 ms at
  10 fps). The detect thread hands over the next frame after it's sent to the robot, and never
  waits to: if the view's thread holds the hand-off at that instant, the frame is recycled and the
  next one goes instead. The picture is made on the view's thread, at nice 10 to detection's -5, so
  on a busy board the view slows, not detection. On a laptop core a picture takes about 2.2 ms at
  the defaults: about 2% of one core.
- **Many watching:** one picture is made for all of them. At most 8 watch at once, across every
  camera; more are answered 503. The robot controller's address has 2 more of its own, so a
  laptop's open pages can't shut it out.
- **Many connecting:** at most 32 connections at once (and the controller's 2 more); more are
  closed as they're accepted. A connection that sends no request's headers within 5 s, the first
  or the next, is closed.
- **Addressed elsewhere:** a request whose `Host` is neither an IP address nor the board's own name
  (`localhost`, its host name, or that `.local`) is answered 421, so a web page whose name is
  pointed at the board can't read its pictures.
- **A slow watcher:** each gets the newest picture whenever its connection is ready for another,
  so a slow one skips pictures rather than queueing them.
- **A watcher that stops reading:** its socket's send buffer is kept small (256 KB), and once a
  write to it has waited 5 s its connection is closed.
- **The port taken:** Visor says so in its log, and detects without the view.

The server is axum, its connections served by hyper (for the header timeout), on a
single-threaded tokio runtime, on a thread of its own; the picture is
fast_image_resize's averaging, embedded-graphics' drawing and jpeg-encoder's JPEG.

`tests/stream.rs` runs a camera end to end, frames in at the camera's 60 fps whatever detection is
doing and datagrams out on loopback, with the view fetched over HTTP: the outline lands on the tag
at the view's size, and at its rate, counted per second of a test clock the view is paced by; no
picture is made with no one watching; watchers and connections past the most are turned away, but
the controller's; idle connections are closed; and a request addressed elsewhere is refused. It measures the
claim above: the same camera runs with no one watching, then with a slow watcher, a stalled one and
ones that vanish mid-picture, and every frame reaches the robot with none dropped, while the worst
time from slot to robot and the worst detection (each bar the worst 1%, for a busy machine's
hiccups) stay within 4 ms of the run with no one watching. A detect thread held up 20 ms at each
hand-off fails it. The stream tests run one at a time.
