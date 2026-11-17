# Visor's live view

Each camera's live view is served over HTTP on port 5809 of its coprocessor, inside FRC's 5800 to
5810 for teams, beside Spotter's agent on 5808. It's for the pit: aiming a camera, focusing it,
setting its brightness, and seeing what it sees and what Visor finds in it. Its page is a small
application, with every camera on one screen and a tab for each.

## Addresses

| Address | What |
|---|---|
| `/` | The page (below): `/page.css` and `/page.js` are its style and script |
| `/status` | The page's status document: every camera's facts, checks, brightness, focus and lens, as JSON (below) |
| `/stream/<camera>` | One camera's view as an MJPEG stream: `multipart/x-mixed-replace; boundary=frame`, each part one JPEG with its `Content-Type` and `Content-Length` |
| `/corner/<camera>` | The magnified corner of the tag the focus is read from, a small JPEG, while the camera is watched (503 while not) |
| `/frame/<camera>` | The camera's next frame at full size, as detection read it: a binary PGM, with what Visor found in it in the `X-Visor-Frame` header (below) |
| `/focus/<camera>` | `POST` only: forgets the best focus the camera's view has seen (below) |
| `/set/<camera>` | `POST` only, and only with `try_settings = true`: "Set exposure and gain" (below) |
| `/settings/<camera>` | `POST` only, and only with `try_settings = true`: an exposure and gain set by hand (below) |

A camera is named as in Visor's configuration, which a robot's deploy writes from the robot's
definition of it, escaped as an address needs: `front left` is `/stream/front%20left`. Any other
name is a 404 that lists the cameras Visor has. Every browser shows the stream in an `<img>`, which
is how Pitwall's camera card shows it; so does `ffplay` or VLC. The `POST` addresses refuse a
request carrying another site's `Origin` (403), so a web page elsewhere can't change a camera
through a laptop on the board's network, and answer in plain text.

## The page

Three files compiled into Visor (`src/stream/page/`), with no web toolchain and nothing loaded from
anywhere else, since a robot's network has no internet: no CDN, fonts or links. Everything it shows
comes from the status document, read twice a second, and the cameras' streams; an action's answer
is shown for a few seconds and then cleared. It works at a laptop's width and, in one column, at a
phone's.

**The cameras screen**, the first: every camera on this Visor side by side, each picture in a box
of its own shape, as large as the window allows with every camera and its list fitting at a
laptop's width (the page chooses the columns), tags outlined and labelled. Under each picture, the camera's name and USB serial number, and
its checks as plain sentences, what needs attention first:

| Check | Fine | Needs attention |
|---|---|---|
| Connection | USB 3, or USB 2 for a USB 2 camera | A USB 3 camera on USB 2 (reseat its plug or change its cable), or USB 1 |
| Frame rate | Within 95% of the `fps` asked for | Slower: said as the exposure being longer than a frame when it is, otherwise the USB link |
| Brightness | The tag's white from 80 to 170 | Too dark or too bright ("Brightness", below) |
| Focus | Sharp | Soft, or needs focusing ("Focus", below) |
| Lens calibration | The configuration has one for the camera's size | Missing: the robot gets tags' corners only, no distance or pose |
| Clock | Synced to the robot (chrony) | Not synced: frame times won't match the robot's |

Brightness and focus with no tag to judge by, and anything before the camera's first frame, are
marked as not known rather than as faults. A camera whose exposure or gain was changed from the page
says so in its list, as information, not as a fault. Choosing a camera opens its tab. A stream
holds a connection, so only the screen shown streams: a tab's camera, or every camera on the
cameras screen.

**A camera's tab**: its picture beside a panel of three rows, always live.

Each row's verdict is a label in words from one table in the page's script ("Good", "Too dark",
"Too bright", "No tag in view", "Sharp", "Soft", "Needs focusing", "Can't judge", "Calibrated",
"Missing"); the status document carries the verdicts as tokens.

- **Brightness**: its verdict and sentence, the exposure and gain in effect and the tag's white;
  with `try_settings = true`, the button **Set exposure and gain** and, behind a closed "Set by
  hand", sliders for each bounded by the camera's own ranges with the range written beside them:
  the exposure in milliseconds on a logarithmic scale (a linear one over 0.1 to 819 ms would put
  everything useful in its first pixel), with a box for an exact value, and the gain on a linear
  one with its own. Under them it compares the settings in effect with the ones the configuration
  gave the camera when Visor opened it: "These are the configuration's settings", or, plainly,
  "Gain 52, changed here from the configured 160. Not kept: it lasts until Visor restarts. To
  keep it, give this camera gain = 52 in its configuration."
- **Focus**: its verdict and sentence, the bar, the small print (`edge 2.4 px wide, 1.9 at best`),
  the magnified corner, and **Forget the best focus seen**.
- **Lens calibration**: what the configuration has, or that it's missing, and a "Calibrate the
  lens" button that's disabled: it isn't built yet. Calibration comes after focusing, since
  turning the lens again means calibrating again.

Below the panel, the camera's facts: its name, index, serial, device, link, size and frame rate.

## The status document

`GET /status` is one JSON document (serde_json, as the frame's facts are): `can_set` (whether the configuration says `try_settings =
true`), and for each camera in `cameras` its `name`, `index`, `serial` (`null` without one),
`device`, `width`, `height`, `fps_asked`, `fps` measured (`null` while nobody watches), `link`, the
addresses of its `stream`, `corner`, `frame`, `forget`, `set` and `by_hand`, and:

- `checks`: each `{topic, mark, say}`, `topic` one of `connection`, `frame_rate`, `brightness`,
  `focus`, `lens`, `clock` and `settings`, `mark` one of `attention`, `unknown`, `note` and `fine`,
  in that order;
- `brightness`: its `verdict` (`no_tag`, `too_dark`, `good`, `too_bright`) and `say`, the tag's
  `black` and `white`, the `exposure_us`, `gain` and `auto_exposure` in effect, the
  `configured_exposure_us`, `configured_gain` and `configured_auto_exposure` the configuration
  gave the camera when it was opened (read back), `changed_here` (whether those differ) and
  `settings_say`, the camera's `exposure_range_us` and `gain_range` (least and most, `null` until
  it's been opened), `can_set`, and `setting` (`idle`, `looking`, `setting`, `settled`,
  `could_not`) with `setting_say`;
- `focus`: its `verdict` (`sharp`, `soft`, `needs_focusing`, `cant_judge`), `say`, `now_px`,
  `best_px`, `bar` (0 to 1) and `small_print`;
- `lens`: `calibrated` and `say`.

It's read-only, so it's answered with or without `try_settings`. Pictures are made only while a
camera is watched, so what's said of them is too.

## The picture

The camera's frame, made smaller by averaging (half its size unless configured: 800 x 652 of a
ThriftyCam's 1600 x 1304), in grey as Visor sees it. Each tag Visor found in that frame is outlined,
white over black, with its ID in its middle. One line along the top gives detection's frame rate,
over about the last second; the exposure in effect, as read back from the camera, or `auto
exposure` while the camera's own is on; the gain, or `no gain` for a camera without the control;
and the frame's brightness, its median and its 99th percentile (docs/wire.md, "Exposure and
brightness"). Nothing else is drawn on it, since other programs show the stream too: the page's
words, bar and magnified corner are its own.

## Brightness

The brightness verdict goes by the tag the view goes by (the one nearest the frame's middle): its
white, the 90th percentile of the pixels inside its black square, from 80 to 170 is good, under
80 too dark, over 170 too bright, and with no tag there's nothing to judge by. Those lines are
where the focus number can be trusted (below).

**Set exposure and gain** (`POST /set/<camera>`, while the camera is watched) keeps the exposure
the camera's configuration asks for, since that's chosen against motion blur, and steers the gain
until the tag's white is between 110 and 150, aiming at 130, well inside the 90 to 170 where a
real camera's readings were flat. Gain is the one to spend: with a tag's brightness held steady
while exposure was traded for gain, its edge read 1.76, 1.76, 1.71, 1.69 and 1.70 px at gain 16,
31, 62, 124 and 248 (measured on a ThriftyCam). How it goes:

- Each step is in proportion to how far off the white is, but never more than double or half,
  since the squeezed top of the response understates how far over a bright tag is; a clipped
  white (250 or more) halves the light. Steps never go back past a setting already seen on the
  other side of the band.
- Only if the gain runs out, at its most or its least, does the exposure move: never longer than
  96% of a frame at the camera's `fps` (16.0 ms at 60; measured, 60.3 fps to 16.0 ms, 59.6 at
  16.6, 49.7 at 20, 30.1 at 33), so the frame rate holds. Then the page says it did.
- With no exposure configured (the camera on its own automatic exposure), it starts from a
  ladder: 1, 2, 4, 8 ms and so on to the longest a frame allows, at the least gain, then the gain
  doubled, until a tag is seen; the exposure it's seen at is then kept as a configured one is.
  With an exposure configured and no tag seen, it looks toward the light the frame says: a frame
  whose brightest 1% is under 64 walks up (the gain doubling at that exposure, then the exposure
  doubling at the most gain); one with more than 1% of it clipping walks down (the gain halving to
  its least, then the exposure halving to its shortest); one that is neither walks down first,
  then up from where it started. Down first because a tag missing from a frame that isn't dark is
  likelier washed out than too dim to see (a tag is found down to 20 grey levels of contrast),
  and the rungs down are few and only shorten the exposure.
- After each ask it waits for the frames to say the change is in effect, then two frames more and
  at least 0.7 s since the ask (a change shows in the pixels within 0.6 s), before it looks again.
  An ask not in effect after 2 s is taken as done, and the next step goes on from what is.
- Once the white is in the band it's settled, and does nothing more: it doesn't chase the light;
  pressing it again starts afresh. At the camera's limits with the white short of the band but
  still good (80 to 170), it settles there.
- When it can't get there it stops and says why: too dark at the most gain and the longest
  exposure a frame allows, too bright at the least gain and the shortest exposure, no tag at any
  rung, the light or the tag changing for 30 asks, or a camera with no exposure control. Then it
  sets the exposure and gain it started from again (a camera that was on its own automatic
  exposure gets it back by way of the configuration's settings) and says nothing was changed:
  only a settled "Set exposure and gain" leaves new settings.

The ranges come from the camera's own controls, said each time the camera is opened. Gain is taken
to amplify in proportion to its value where its least is above zero (the ThriftyCam's runs 16 to
255, 16 being 1x: 2 ms at 16 matched 1 ms at 32), and to double every eighth of its range where it
starts at zero; either way the next frame is measured, so a wrong guess costs a step.

**By hand**, `POST /settings/<camera>?exposure_us=1500&gain=2` sets either or both on the running
camera, before its next frame, held to the camera's own range; the answer says when a value was
held (`Exposure 900.0 ms is outside what the camera takes (0.1 to 819.2 ms): held to 819.2 ms`),
and says what was set as a person would (`set exposure 2.0 ms and gain 80`). A change by hand stops "Set exposure and gain" and is left alone.

Either way, what's set lasts until the camera is opened again (Visor restarted, or the camera
replugged); then the configuration's values are set, as always. Every frame after says what's in
effect, read back from the camera: the picture's line, a full-size frame's header, and the frames
sent to the robot. What's found is kept by writing it into the robot program; the page says what
values to give. On a robot, a camera's exposure and gain are the robot program's, and its deploy
never writes `try_settings`: without it, both addresses answer 403 and the page has no controls.

## Focus

A printed tag's black square ends in a clean step to white. In the frame that step is as wide as
the lens and the sensor make it: about a pixel and a half in focus, and about two and a half more
for each pixel of blur. So the focus number is a width in the camera's own pixels, and **smaller
is sharper**. It is the step's height over its steepest slope, read on lines across the four sides
of the square, so what else is in view doesn't move it, and the tag's distance and angle move it
little. Two cameras of one kind, focused alike, read alike.

That holds only while the camera's response is straight across the step, and a real camera's
isn't at either end. Measured on a ThriftyCam at 1600 x 1304, at the least gain with the exposure
stepped from 2 to 10 ms: a sharp tag near the middle read 1.46 to 1.55 px with its white from 92
to 168, then falsely sharp as the white rose (1.37 px at 175, 1.18 at 190, 1.04 at 201, 0.97 at
208); another sharp tag held to a white of 192, then read 1.21 at 200; soft tags (about 1.9 px)
held to 203. A sharp edge is the first to go, from a white of about 175, and readings are flat
within 3% from about 90 to 170. Dim reads soft: 2.03 px at a white of 46 for a sharp tag, while a
soft one read no worse at 64 and 77. So a reading counts only in good light: the white from 80 to
170. A line across an edge whose white side is over 170 is left out of the reading too.

The verdict is absolute:

| Edge | Verdict | Why |
|---|---|---|
| Up to 2.0 px | Sharp | No blur reads about 1.55 px, half a pixel of it about 1.8 |
| 2.0 to 3.0 px | Soft | A pixel of blur reads about 2.9 px and costs little even at 6 m (a far tag's corners 0.060 px off, against 0.036 sharp) |
| Over 3.0 px | Needs focusing | Two pixels of blur reads about 5.0 px and makes a far tag's corners about eight times worse (0.493 px at 6 m); three loses it |

The corner errors are this repository's detection-limit rows (`tests/detection_limits/main.rs`),
the widths its rendered tags (`tests/focus.rs`). **These lines are provisional** until a real lens
has been turned through focus against them. The verdict goes by the tag nearest the middle, and
only a tag whose centre is in the middle half of the frame across and down counts, since a lens is
softer toward its edges (a ThriftyCam read tags there at 1.7 to 1.9 px while the middle read 1.5);
with tags only at the edges it says to put one near the middle. It needs good light, or it says it
can't judge until the brightness is set. It doesn't flap: it changes only once the smoothed
reading has been 5% past a line for five pictures in a row (half a second).

While a lens is turned, the best-seen reading, the bar and the magnified corner are the guide. The
number is smoothed over the last few pictures, and the best is the smallest the smoothed number has
been while anyone has watched, in the same exposure and gain: a change of either forgets it, since
a best seen in other light isn't comparable. **Forget the best focus seen**, or `POST
/focus/<camera>`, forgets it too, for another go. The magnified corner (`GET /corner/<camera>`) is
the frame round that tag's first corner, 72 of its pixels across drawn three times their size,
nothing smoothed: what the number measures, for the eye. It stays where the tag was when the tag
is lost, and it's the middle of the frame until a tag has been measured.

To focus a camera:

1. Fix the camera and put a tag in view near the middle, at the distance it is to see best, large
   enough to read: its black square 100 px or more across is good, and under 32 px isn't measured.
2. Press **Set exposure and gain** if the brightness isn't good.
3. Turn the lens. The width falls as the focus comes in and rises once it's past: the best stays,
   and the bar shows how near it the lens is now. Turn back until the bar is full, and lock the
   lens. Then calibrate.

## What it doesn't do

- It shows one Visor's cameras, not a whole robot's: each coprocessor serves its own.
- Setting exposure and gain from the page is bench-only: it needs `try_settings = true`, which a
  robot's deploy never writes. What's set isn't kept past a restart; the page says what to write
  into the configuration, and doesn't write it.
- Calibrating the lens from the page isn't built.
- The focus lines (2.0 and 3.0 px) are provisional, from rendered frames; the light lines (80 and
  170) and the band (110 to 150) are from one ThriftyCam's measurements.
- It gives no focus number without a tag in view, none for a tag under 32 px across, and no
  verdict for a tag off the middle; a tag small in the frame (under about 60 px) reads low once
  it is well out of focus, though its smallest reading is still at the best focus.
- It is a frame's own pixels, so a camera set to a smaller frame size reads a narrower edge for the
  same lens.
- Settled, "Set exposure and gain" doesn't follow the light; pressing it again does. A tag it
  can't find in a room that's neither dark nor clipping costs it the whole search, down and up.
- A browser holds at most six connections to one address, and each stream takes one: a cameras
  screen of more than five cameras would starve the status document.

`tests/dial.rs` runs "Set exposure and gain" against a simulated camera with the measured one's
traits (exposure times gain, a squeezed and clipping top end, a change shown a few frames after
it's asked for and in the pixels later, the frame rate falling with an exposure past a frame) and
another camera's ranges, in daylit, dim, too dark, very bright, sunlit and tagless rooms, and a
bright venue where the tag is washed out at the configured settings; each that can't get there
must end on the settings it started from.
`tests/focus.rs` checks that a rendered tag lit brighter and brighter never counts a reading more
than 3% off the one it gives well exposed. `cargo run --example page` serves the page from rendered
scenes, with no cameras, to look at it in a browser.

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
measure; the frame's is that of the tag the view goes by (the one nearest the frame's middle,
among those measured or large enough and wholly in the frame), unsmoothed, whatever its light. A
tag's `black` and `white` are the 10th and 90th percentiles of the pixels inside its black square
(`null` for one wholly off the frame), and `light` is that tag's verdict: `"too_dim"`, `"good"` or
`"too_bright"` ("Focus", above), `null` with no such tag. So whether a reading counted can be
checked from the numbers alone, and a frame the detector missed a tag in, saved with this header, is a test case.

It costs detection nothing, as the view costs it nothing: the frame is the one detection hands
over after sending it to the robot, copied on the view's thread. But it's a lot of network: one
frame is half a second of a field's 4 Mbit/s. So each camera gives one at most every half second,
one request at a time, and each request takes one of the watchers' 8 places while it waits. Ask
for one when it's wanted, in the pit or on a bench; it isn't a stream. A camera sending no frames
answers 503 after 2 s.

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
