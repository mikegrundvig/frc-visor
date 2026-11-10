# Visor's live view

Each camera's live view is served over HTTP on port 5809 of its coprocessor, inside FRC's 5800 to
5810 for teams, beside Spotter's agent on 5808. It's for the pit: aiming a camera, focusing it, and
seeing what it sees and what Visor finds in it.

## Addresses

| Address | What |
|---|---|
| `/` | A page with every camera's live view, by name |
| `/stream/<camera>` | One camera's view as an MJPEG stream: `multipart/x-mixed-replace; boundary=frame`, each part one JPEG with its `Content-Type` and `Content-Length` |

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
  brightness").

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
