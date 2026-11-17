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
| `/focus/<camera>` | `POST` only: forgets the best focus the camera's view has seen (below) |
| `/focus-start/<camera>` | `POST` only, and only with `try_settings = true`: starts the camera's focus step, or sets it for focusing again (below) |
| `/focus-again/<camera>` | `POST` only, the same: sets the exposure and gain for focusing again while the focus step is on |
| `/focus-end/<camera>` | `POST` only, the same: ends the focus step and puts the settings back |
| `/status/<camera>` | `GET`, only with `try_settings = true`: what the camera's view says now, as plain text, for the page (below) |

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

Under it, in plain words: when the camera has slowed because the exposure is longer than a frame,
a line that says so (`The exposure (33.0 ms) is longer than a frame, so the camera has slowed to 30
frames a second: shorten it.`), said only when the rate measured is under 95% of the camera's
`fps` and the exposure in effect is longer than a frame at it, never blaming the gain, which
doesn't change the frame rate; then a sentence for focus and the focus step (below); then the
small print, the edge's width, with its bar.

## Focus

The small print says how sharply the camera sees a tag: `edge 2.4 px wide, 1.9 at best`, and a bar
that is full when the focus is at its best. The sentence above it says what to do, in words a
student can act on:

| The view sees | It says |
|---|---|
| No tag large enough to measure | No tag in view: point the camera at a tag, near the middle. |
| The tag's white under 80 | Too dim to measure focus: lengthen the exposure, or add light on the tag. |
| The tag's white over 170 | Too bright to measure focus: the tag's white is washed out. Shorten the exposure or lower the gain. |
| A tag well lit, but too few of its edge's lines readable | Can't read the tag's edge: check for glare on it, and that all of it is in view. |
| A reading, not yet a best | Turn the lens slowly and watch the bar. |
| A reading within 6% of the best | Sharp: as good as the best seen. Lock the lens here. |
| A reading wider than the best | Keep turning: it was sharper before. Turn back slowly until the bar is full. |

A printed tag's black square ends in a clean step to white. In the frame that step is as wide as
the lens and the sensor make it: about a pixel and a half in focus, and about two and a half more
for each pixel of blur. So the number is a width in the camera's own pixels, and **smaller is
sharper**. It is the step's height over its steepest slope, read on lines across the four sides of
the square, so what else is in view doesn't move it, and the tag's distance and angle move it
little. Two cameras of one kind, focused alike, read alike.

That holds only while the camera's response is straight across the step, and a real camera's
isn't at either end. Measured on a ThriftyCam at 1600 x 1304, at the least gain with the exposure
stepped from 2 to 10 ms: a sharp tag near the middle read 1.46 to 1.55 px with its white from 92
to 168, then falsely sharp as the white rose (1.37 px at 175, 1.18 at 190, 1.04 at 201, 0.97 at
208); another sharp tag held to a white of 192, then read 1.21 at 200; soft tags (about 1.9 px)
held to 203. A sharp edge is the first to go, from a white of about 175, and readings are flat
within 3% from about 90 to 170. Dim reads soft: 2.03 px at a white of 46 for a sharp tag, while
a soft one read no worse at 64 and 77. Raising the gain did the same as raising the white. So the
view measures each tag's light, its black and white (the 10th and 90th percentiles of the pixels
inside its black square), and a reading counts only in good light: the white from 80 to 170. Out
of it the line says why, and the best isn't touched. A line across an edge whose white side is
over 170 is left out of the reading too. The camera's response is squeezed far below full scale,
so the line is drawn at 170, not at clipping.

To focus a camera:

1. Fix the camera and put a tag in view at the distance it is to see best, large enough to read:
   its black square 100 px or more across in the frame is good, and under 32 px isn't measured.
   With several tags in view, the number is the one nearest the middle of the frame.
2. Turn the lens. The width falls as the focus comes in and rises once it's past: the best the
   width has been stays on the line, and the bar shows how near it the lens is now.
3. Turn back until the bar is full, and lock the lens.

The number is smoothed over the last few pictures, and the best is the smallest the smoothed number
has been while anyone has watched, in the same exposure and gain: a change of either forgets it,
since a best seen in other light isn't comparable. The page's button for the camera, or
`POST /focus/<camera>`, forgets it too, for another go. With no tag it can measure, the line says so and keeps the best: a lens
turned far from focus loses the tag until it's turned back.

The corner of the picture shows the frame round that tag's first corner three times its size, each
of the frame's pixels as it is, nothing smoothed: what the number measures, for the eye. A box in
the picture marks the part shown. It stays where the tag was when the tag is lost, and it's the
middle of the frame until a tag has been measured. A picture under 180 px high has none.

## The focus step

On a bench (`try_settings = true`), the page has three buttons under each camera: **focus this
camera**, **set it for me again** and **end focusing and put the settings back**. The first starts
the camera's focus step: Visor remembers the exposure and gain in effect at the next frame, and
sets them for focusing itself ("dialling in"), saying what it's doing in the sentence:

- The gain goes to the camera's least, and the exposure is steered until the tag's white is
  between 110 and 150 (aiming at 130), well inside the flat 90 to 170 (a band of 160 to 200
  settled at 200, where a 1.5 px edge read 1.1): each step in proportion to how far off the white
  is, but never more than double or half, since the squeezed top of the response understates how
  far over a bright tag is. A clipped white (250 or more) halves the light. Steps never go back past a setting
  already seen on the other side of the band.
- The exposure is never longer than 96% of a frame at the camera's `fps` (16.0 ms at 60), so the
  frame rate holds: measured, 60.3 fps to 16.0 ms, 59.6 at 16.6, 49.7 at 20, 30.1 at 33. Only when
  the exposure is that long and the tag still too dim does the gain rise. The gain doesn't change
  the frame rate.
- With no tag seen, it walks a ladder: 1, 2, 4, 8 ms and so on to the longest a frame allows, at
  the least gain; then the gain doubled at that exposure, to the most.
- After each ask it waits for the frames to say the change is in effect, then two frames more and
  at least 0.7 s since the ask (a change shows in the pixels within 0.6 s), before it looks again.
  An ask not in effect after 2 s is taken as done, and the next step goes on from what is.
- Once the white is in the band it is settled, and does nothing more: it doesn't chase the light.
  **Set it for me again** dials in afresh. At the camera's limits with the white short of the band
  but still good to focus in (80 to 170), it settles there.
- When it can't get there it stops and says why: too dark at the longest exposure and the most
  gain, too bright at the shortest exposure and the least gain, no tag at any rung of the ladder,
  the light or the tag changing for 30 asks, or a camera with no exposure control.

The ranges come from the camera's own controls, said each time the camera is opened, never from
one camera's numbers. Gain is taken to amplify in proportion to its value where its least is above
zero (the ThriftyCam's runs 16 to 255, 16 being 1x: 2 ms at 16 matched 1 ms at 32), and to double
every eighth of its range where it starts at zero; either way the next frame is measured, so a
wrong guess costs a step.

Changes by hand stay possible while the step is on, and aren't fought: a change by hand stops the
dial, and the sentence goes back to focus. **End focusing** puts back the remembered settings; so
does the last watcher leaving. A camera that was on its own automatic exposure goes back to the
configuration's settings, the only way back to it.

`POST /focus-start/<camera>` starts it (or dials in again where it's on), `POST
/focus-again/<camera>` dials in again (409 when the step isn't on), and `POST /focus-end/<camera>`
ends it. They answer in plain text, refuse another site's page (403), and a camera Visor hasn't got
is a 404 listing the ones it has. Without `try_settings = true` they answer 403 and the page has no
buttons. Whether the focus step may run is its own decision in Visor's code, read from the same
key for now.

`tests/dial.rs` runs the dial against a simulated camera with the measured one's traits (exposure
times gain, a squeezed and clipping top end, a change shown a few frames after it's asked for and
in the pixels later, the frame rate falling with an exposure past a frame) and another camera's
ranges, in daylit, dim, too dark, very bright, sunlit and tagless rooms; `tests/focus.rs` checks
that a rendered tag lit brighter and brighter, through a response squeezed like the camera's
(true to a white of 170, falsely sharp past it), never counts a reading more than 3% off the one
it gives well exposed.

What it doesn't do:

- It gives no number without a tag in view, and none for a tag under 32 px across; a tag small in
  the frame (under about 60 px) reads low once it is well out of focus, though its smallest
  reading is still at the best focus.
- It is a frame's own pixels, so a camera set to a smaller frame size reads a narrower edge for the
  same lens.
- Its light thresholds (80 and 170) and the band (110 to 150) are from one ThriftyCam's
  measurements; another camera with a different response may need others.
- The focus step is bench-only for now: it runs only where the configuration says
  `try_settings = true`, never on a robot.
- It sets exposure and gain for focusing only; it doesn't choose a robot's match settings (a short
  exposure, the gain steered), which stay the robot program's.
- Settled, it doesn't follow the light: a cloud or a lamp after it settles is the student's to
  notice, by the sentence, and **set it for me again** is the answer.
- Settings it sets last until the step ends, or until the camera is opened again; it writes nothing
  into the configuration.

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
 "edge_px":1.846,"light":"good","tags":[{"id":7,"hamming":0,"decision_margin":81.5,
 "corners":[610.2,480.9,702.4,481.3,702.0,573.8,609.7,573.1],"distance_m":2.013,"pose_error_px":0.21,
 "edge_px":1.846,"black":31,"white":184}]}
```

`sequence` is the frame's number as sent to the robot, so it's the same frame the robot got. A
value the camera can't give is `null` (no gain control; automatic exposure on). `corners` are as
on the wire: x0, y0 to x3, y3, in pixel-centre coordinates, raw. `distance_m` is from the lens to
the tag's centre by its best pose, and `null` without a lens calibration. A tag's `edge_px` is the
width of its edge in that frame ("Focus", above), `null` for a tag too small or too faint to
measure; the frame's is that of the tag the view's line goes by (the one nearest the frame's middle,
among those measured or large enough and wholly in the frame), unsmoothed, whatever its light. A
tag's `black` and `white` are the 10th and 90th percentiles of the pixels inside its black square
(`null` for one wholly off the frame), and `light` is that tag's verdict: `"too_dim"`, `"good"` or
`"too_bright"` ("Focus", above), `null` with no such tag. So whether a reading counted can be
checked from the numbers alone. So a frame the detector missed a tag in, saved with this header, is a test case.

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

sets either or both on the running camera, before its next frame, held to the camera's own range;
the answer says when a value was held (`Exposure 900.0 ms is outside what the camera takes (0.1 to
819.2 ms): held to 819.2 ms`). The page at `/` gets a slider for each under each camera, bounded by
the camera's own ranges with the range written beside it: the exposure in milliseconds on a
logarithmic scale (a linear one over 0.1 to 819 ms would put everything useful in its first pixel),
with a box beside it for an exact value, and the gain on a linear one with its own box. A change by
hand while the focus step is on stops its dial.

`GET /status/<camera>` is what the page reads twice a second to fill those in, as plain text, one
`name value` a line: `phase` (`off`, `looking`, `setting`, `settled`, `could_not` or `by_hand`),
`exposure_range_us` and `gain_range` (least and most, once the camera has been opened), the
`exposure_us` and `gain` in effect, the `sentence`, `slowed` when the camera has slowed, and
`small_print`. Every frame after says what's in
effect, read back from the camera: the view's line, a full-size frame's header, and the frames
sent to the robot.

A trial lasts until the camera is opened again: Visor restarted, or the camera replugged. Then the
configuration's values are set, as always. What's found by trying is kept by writing it into the
robot program.

Without `try_settings = true` the answer is 403, and the page has no sliders. A robot's deploy never
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
