# /// script
# requires-python = ">=3.12"
# dependencies = ["opencv-python==5.0.0.93", "numpy==2.5.4"]
# ///
"""Live view for focusing a camera and recording its lens calibration, on a laptop with the camera
plugged into it.

    uv run tools/capture.py /dev/v4l/by-path/...-video-index0 --out frames/
    uv run tools/capture.py DEVICE --squares 15x15 --square-mm 28.575 --marker-mm 22.225 --dictionary 4X4_1000 --out frames/

It opens paused, for focusing: a bar shows how sharp the middle of the frame is, against the best
seen, and f zooms the middle to full size. Point the camera at a tag at the distance to focus for,
turn the lens to the bar's peak, and lock it. r resets the peak.

Then s starts recording. It shows the board's corners on the view, roughly how far away the board
is and how far it's tilted, how many corners each ninth of the frame has so far, and how many
reach out toward each corner of the frame, past the arc drawn there (green once there are as many
as calibrate.py asks for; a partly visible board counts). It saves a
frame every --every seconds (1.5) while the board is in view and steady enough not to blur, and
says when one is much like a frame saved before: move it somewhere new. s pauses it again, space saves one by hand at any time, u drops the
last, q quits. Frames already in --out count, so a run that stopped carries on: give each camera a
folder of its own. Then solve the folder: `uv run tools/calibrate.py solve frames/*.pgm ...`, with
the same board.

The distance and tilt come from a lens guessed from the camera's field of view, so they're rough:
enough to place the board, not a measurement. The frames are the camera's own pixels at the size
asked for, its YUYV luma when it offers YUYV (as Visor reads it), never a compressed MJPG frame.
"""

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from calibrate import CORNER_NAMES, Limits, board_arguments, board_from, reach, short_corners  # noqa: E402

LIMITS = Limits()
# A ninth of the frame is covered once it has this many corners: well over the solver's floor.
REGION_AIM = 3 * LIMITS.min_per_region
REGION_LIST = ["top left", "top", "top right", "left", "middle", "right", "bottom left", "bottom", "bottom right"]
# Steady: the board's corners moving slower than this, in pixels a second. At a 10 ms exposure that
# smears a corner by 0.3 px at most, well under what the solve tolerates; a hand-held board drifts.
STEADY_PX_PER_S = 30.0
# New: a frame differs from one saved before in one of these, else the view says so.
NEW_CENTER = 0.12  # of the frame's diagonal
NEW_DISTANCE = 0.25  # of the distance
NEW_TILT_DEG = 12.0
VIEW_WIDTH = 1000
ORANGE, GREEN, WHITE, YELLOW = (0, 200, 255), (0, 200, 0), (255, 255, 255), (0, 255, 255)


def open_camera(device, width, height):
    """The camera, or None when it isn't there (unplugged) or won't open."""
    if not Path(device).exists():
        return None
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not camera.isOpened():
        return None
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    # A read gives up after a second, not OpenCV's ten, so an unplugged camera never freezes the view.
    camera.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000)
    return camera


def waiting(device, status):
    """The view while there's no camera: what's wrong, kept live so the window answers."""
    panel = np.zeros((240, VIEW_WIDTH, 3), np.uint8)
    for i, text in enumerate([status, f"waiting for {device}", "plug it back in; q quits (saved frames are kept)"]):
        cv2.putText(panel, text, (10, 50 + 50 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, ORANGE, 2)
    cv2.imshow("calibration", panel)
    return cv2.waitKey(200) & 0xFF


def luma(raw, width, height):
    """The frame's brightness: YUYV's Y bytes when the camera sent YUYV, else converted to gray."""
    if raw.ndim == 3 and raw.shape[2] == 3:
        return cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
    flat = raw.reshape(-1)
    if flat.size == width * height * 2:
        return flat.reshape(height, width, 2)[:, :, 0].copy()
    if flat.size == width * height:
        return flat.reshape(height, width).copy()
    sys.exit(f"The camera sent {raw.shape}, not {width}x{height}: does it offer that size in YUYV?")


def guessed_lens(width, height, diagonal_fov_deg):
    focal = math.hypot(width, height) / 2 / math.tan(math.radians(diagonal_fov_deg) / 2)
    return np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]], float)


def pose(object_points, image_points, lens):
    """The board's distance in meters and its tilt from facing the camera, in degrees, or None."""
    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, lens, None)
    if not ok:
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    normal = rotation[:, 2]
    tilt = math.degrees(math.acos(min(1.0, abs(float(normal[2])))))
    center = rotation @ np.mean(object_points, axis=0) + tvec.reshape(3)
    return float(np.linalg.norm(center)), tilt


def middle(gray):
    height, width = gray.shape
    return gray[height // 3 : 2 * height // 3, width // 3 : 2 * width // 3]


def sharpness(patch):
    """The Laplacian's variance: it rises and falls with focus, for a scene that doesn't change."""
    return float(cv2.Laplacian(patch, cv2.CV_64F).var()) if patch.size else 0.0


def found_board(gray, board, detector, lens):
    """The board in one frame: its corners, where it is, its rough distance and tilt, and how sharp
    it is; None when too little of it is seen to count."""
    corners, ids, _, _ = detector.detectBoard(gray)
    if ids is None or len(ids) < LIMITS.min_corners or board.checkCharucoCornersCollinear(ids):
        return None
    object_points, image_points = board.matchImagePoints(corners, ids)
    image_points = image_points.reshape(-1, 2)
    measured = pose(object_points.reshape(-1, 3), image_points, lens)
    if measured is None:
        return None
    x, y, w, h = cv2.boundingRect(image_points.astype(np.float32))
    return {
        "corners": image_points,
        "ids": ids.flatten(),
        "center": image_points.mean(axis=0),
        "distance": measured[0],
        "tilt": measured[1],
        "sharpness": sharpness(gray[max(y, 0) : y + h, max(x, 0) : x + w]),
    }


def region_counts(saved, width, height):
    counts = np.zeros((3, 3), int)
    for frame in saved:
        for x, y in frame["corners"]:
            counts[min(int(3 * y / height), 2), min(int(3 * x / width), 2)] += 1
    return counts


def reach_counts(saved, width, height):
    """Corners saved out toward each of the frame's four corners, counted as calibrate.py counts
    them, in its CORNER_NAMES' order."""
    corners = np.vstack([frame["corners"] for frame in saved]) if saved else np.zeros((0, 2))
    return reach(corners, (width, height), LIMITS)


def draw_reach(image, toward):
    """The line the corners must get beyond toward each corner of the frame, green once enough
    have, with how many have."""
    height, width = image.shape[:2]
    centre = ((width - 1) / 2, (height - 1) / 2)
    axes = (round(LIMITS.corner_reach * math.sqrt(2) * centre[0]), round(LIMITS.corner_reach * math.sqrt(2) * centre[1]))
    # OpenCV's ellipse angles run from the right toward the bottom, in CORNER_NAMES' order.
    arcs = [(180, 270), (270, 360), (90, 180), (0, 90)]
    signs = [(-1, -1), (1, -1), (-1, 1), (1, 1)]
    for (start, end), (across, down), count in zip(arcs, signs, toward):
        color = GREEN if count >= LIMITS.min_per_corner else ORANGE
        cv2.ellipse(image, (round(centre[0]), round(centre[1])), axes, 0, start, end, color, 3)
        x = centre[0] + across * 0.85 * centre[0] - (110 if across > 0 else 0)
        y = centre[1] + down * 0.85 * centre[1] + 15
        cv2.putText(image, f"{count}/{LIMITS.min_per_corner}", (round(x), round(y)), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)


def is_new(candidate, saved, diagonal):
    for frame in saved:
        near = np.hypot(*(candidate["center"] - frame["center"])) < NEW_CENTER * diagonal
        same_distance = abs(candidate["distance"] - frame["distance"]) < NEW_DISTANCE * frame["distance"]
        same_tilt = abs(candidate["tilt"] - frame["tilt"]) < NEW_TILT_DEG
        if near and same_distance and same_tilt:
            return False
    return True


def speed(before, after, seconds):
    """How fast the board's corners are moving, in pixels a second: the median of the corners seen
    in both frames. None when there's no earlier frame to compare with."""
    if before is None or after is None or seconds <= 0:
        return None
    earlier = dict(zip(before["ids"].tolist(), before["corners"]))
    moved = [np.hypot(*(corner - earlier[i])) for i, corner in zip(after["ids"].tolist(), after["corners"]) if i in earlier]
    if len(moved) < LIMITS.min_corners:
        return None
    return float(np.median(moved)) / seconds


class Mode(Enum):
    PAUSED = "PAUSED"
    RECORDING = "RECORDING"


class Shown(Enum):
    """What the view shows: the whole frame with the board on it, or the middle at full size."""

    WHOLE = "whole"
    MIDDLE = "middle"


@dataclass
class Session:
    """What the live view keeps from one frame to the next."""

    mode: Mode
    status: str
    saved: list = field(default_factory=list)
    shown: Shown = Shown.WHOLE
    # The middle's sharpness, smoothed over a few frames; None until the first frame.
    focus: float | None = None
    focus_peak: float = 0.0
    board_peak: float = 0.0
    # Seconds until the next automatic save, while recording.
    countdown: float | None = None
    # The board as the frame before saw it, to tell how fast it moves, and when.
    previous: dict | None = None
    previous_time: float = 0.0
    last_save: float = 0.0


class Stop(Exception):
    """The run is over: q was pressed, or the replay ran out of frames."""


class Camera:
    """The plugged-in camera, opened again when it comes back after it was unplugged or stalled."""

    def __init__(self, device, width, height):
        self.device, self.width, self.height = device, width, height
        self.capture = open_camera(device, width, height)

    def read(self, session):
        """The next frame's luma, or None when there's none this time round."""
        # Unplugged or stalled: wait for the camera to come back, the window kept live, and keep
        # what's saved, rather than stopping or freezing.
        if self.capture is None:
            if waiting(self.device, f"no camera: {len(session.saved)} frames saved") == ord("q"):
                raise Stop()
            self.capture = open_camera(self.device, self.width, self.height)
            session.previous = None
            return None
        ok, raw = self.capture.read()
        if not ok:
            self.capture.release()
            self.capture = None
            return None
        return luma(raw, self.width, self.height)

    def key(self):
        return cv2.waitKey(1) & 0xFF

    def steady(self, found):
        return found is not None and found["speed"] is not None and found["speed"] < STEADY_PX_PER_S

    def due(self, session, now, every):
        return now - session.last_save >= every

    def show(self, image):
        cv2.imshow("calibration", image)


class Replay:
    """Saved frames read back in place of a camera, as a check: every frame with the board in it is
    taken as steady and due, with no window and no keys."""

    def __init__(self, folder):
        self.paths = sorted(folder.glob("*.pgm"))

    def read(self, session):
        if not self.paths:
            raise Stop()
        path = self.paths.pop(0)
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f"left out {path.name}: it can't be read")
        return gray

    def key(self):
        return -1

    def steady(self, found):
        return found is not None

    def due(self, session, now, every):
        return True

    def show(self, image):
        pass


def carried_on(out, board, detector, lens):
    """The frames an earlier run saved in the folder, so a run that stopped carries on where it
    was. A file it can't read, as one left half written when that run was stopped, is left out."""
    saved = []
    for path in sorted(out.glob("capture-*.pgm")):
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f"left out {path.name}: it can't be read, perhaps half written when a run stopped")
            continue
        earlier = found_board(gray, board, detector, lens)
        if earlier is not None:
            saved.append({**earlier, "path": path})
    return saved


def update_focus(session, gray):
    # Smoothed over a few frames, so sensor noise doesn't make the focus bar jump.
    focus = sharpness(middle(gray))
    session.focus = focus if session.focus is None else 0.7 * session.focus + 0.3 * focus
    session.focus_peak = max(session.focus_peak, session.focus)


def look_for_board(session, gray, board, detector, lens, now):
    """The board in this frame, with how fast it moves since the frame before; None when it isn't
    in view."""
    found = found_board(gray, board, detector, lens)
    if found is not None:
        session.board_peak = max(session.board_peak, found["sharpness"])
        found["speed"] = speed(session.previous, found, now - session.previous_time)
    session.previous, session.previous_time = found, now
    return found


def toggle(session, key):
    """s starts or pauses recording, f zooms the middle in or out, r resets the peaks."""
    if key == ord("s"):
        session.mode = Mode.PAUSED if session.mode == Mode.RECORDING else Mode.RECORDING
        session.status = "hold the board still somewhere new" if session.mode == Mode.RECORDING else "paused"
    if key == ord("f"):
        session.shown = Shown.WHOLE if session.shown == Shown.MIDDLE else Shown.MIDDLE
    if key == ord("r"):
        session.focus_peak = session.focus
        session.board_peak = 0.0


def maybe_save(session, gray, found, key, steady, due, now, args, diagonal):
    """Saves the frame when space is pressed, or while recording when the board is steady and a
    save is due, and says what happened."""
    recording = session.mode == Mode.RECORDING
    automatic = recording and steady and due
    if found is not None and (key == ord(" ") or automatic):
        repeat = not is_new(found, session.saved, diagonal)
        name = args.out / f"capture-{len(session.saved):03d}.pgm"
        while name.exists():
            name = name.with_name(name.stem + "b.pgm")
        cv2.imwrite(str(name), gray)
        session.saved.append({**found, "path": name})
        session.last_save = now
        session.status = f"saved {name.name}" + (": much like an earlier one, move somewhere new" if repeat else "")
        return
    if key == ord(" "):
        session.status = "no board in view to save"
        return
    if recording and found is None:
        session.status = "waiting for the board"
        return
    if recording and not steady and due:
        session.status = "hold steady to save"


def undo(session):
    if not session.saved:
        return
    gone = session.saved.pop()
    gone["path"].unlink(missing_ok=True)
    session.status = f"dropped {gone['path'].name}"


def bar(panel, y, value, peak, label):
    filled = int(600 * min(value / peak, 1.0)) if peak > 0 else 0
    cv2.rectangle(panel, (10, y), (10 + filled, y + 22), GREEN, -1)
    cv2.rectangle(panel, (10, y), (610, y + 22), WHITE, 1)
    cv2.putText(panel, label, (620, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 1)


def draw(gray, session, found, counts, toward, args):
    height, width = gray.shape
    if session.shown == Shown.MIDDLE:
        # The middle at full size, where focus is judged, with its outline on a small whole frame.
        crop = middle(gray)
        image = cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)
        image = cv2.resize(image, (VIEW_WIDTH, int(crop.shape[0] * VIEW_WIDTH / crop.shape[1])), interpolation=cv2.INTER_NEAREST)
    else:
        image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for row in range(3):
            for column in range(3):
                x0, y0 = column * width // 3, row * height // 3
                share = min(counts[row, column] / REGION_AIM, 1.0)
                color = GREEN if share >= 1.0 else (0, int(200 * share), 255)
                cv2.rectangle(image, (x0 + 4, y0 + 4), (x0 + width // 3 - 4, y0 + height // 3 - 4), color, 2)
                cv2.putText(image, str(counts[row, column]), (x0 + 14, y0 + 44), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
        draw_reach(image, toward)
        if found is not None:
            for x, y in found["corners"]:
                cv2.circle(image, (int(x), int(y)), 5, YELLOW, -1)
        image = cv2.resize(image, (VIEW_WIDTH, int(height * VIEW_WIDTH / width)), interpolation=cv2.INTER_AREA)

    panel = np.zeros((190, image.shape[1], 3), np.uint8)
    countdown = "" if session.countdown is None else f"   next in {session.countdown:.1f} s"
    lines = [f"{session.mode.value}   saved {len(session.saved)} / {args.count}{countdown}    {session.status}"]
    if found is not None:
        hint = "  closer than the focus: back off" if found["distance"] < args.near else ""
        moving = found.get("speed")
        motion = "" if moving is None else ("   steady" if moving < STEADY_PX_PER_S else "   moving")
        lines.append(f"board: {len(found['corners'])} corners   ~{found['distance']:.2f} m   tilt ~{found['tilt']:.0f} deg{motion}{hint}")
    else:
        lines.append("no board in view")
    short = [name for name, count in zip(REGION_LIST, counts.flatten()) if count < REGION_AIM]
    corners = short_corners(toward, LIMITS)
    needs = (", ".join(short) if short else "every ninth covered") + "   |   "
    needs += ("out toward: " + ", ".join(corners)) if corners else "every corner reached"
    lines.append("needs: " + needs)
    lines.append("s start/pause   space save   u undo   f zoom (focus)   r reset peak   q quit")
    for i, text in enumerate(lines):
        cv2.putText(panel, text, (10, 26 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.62, ORANGE, 2)
    focus_label = f"focus, middle {session.focus:.0f} (peak {session.focus_peak:.0f})"
    bar(panel, 128, session.focus, session.focus_peak, focus_label)
    if found is not None:
        bar(panel, 158, found["sharpness"], session.board_peak, "board sharpness")
    return np.vstack([image, panel])


def step(session, source, gray, board, detector, lens, args, diagonal):
    """One frame: the focus, the board, the keys, perhaps a save, and the view."""
    update_focus(session, gray)
    now = time.time()
    found = look_for_board(session, gray, board, detector, lens, now)
    steady = source.steady(found)
    due = source.due(session, now, args.every)
    recording = session.mode == Mode.RECORDING
    session.countdown = max(0.0, args.every - (now - session.last_save)) if recording else None

    key = source.key()
    toggle(session, key)
    maybe_save(session, gray, found, key, steady, due, now, args, diagonal)
    if key == ord("u"):
        undo(session)
    if key == ord("q"):
        raise Stop()
    counts = region_counts(session.saved, args.width, args.height)
    toward = reach_counts(session.saved, args.width, args.height)
    source.show(draw(gray, session, found, counts, toward, args))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("device", help="the camera's device, as `visor cameras` lists it")
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=1304)
    parser.add_argument("--fov", type=float, default=80, help="the lens's diagonal field of view, for the rough distance")
    parser.add_argument("--near", type=float, default=0.8, help="meters: closer than this, the hint says back off")
    parser.add_argument("--count", type=int, default=50, help="frames to save before it stops")
    parser.add_argument("--every", type=float, default=1.5, help="seconds between saved frames while recording")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--replay", type=Path, help="read saved .pgm frames from this folder instead (a check)")
    board_arguments(parser)
    args = parser.parse_args(argv)

    board = board_from(args).opencv()
    detector = cv2.aruco.CharucoDetector(board)
    lens = guessed_lens(args.width, args.height, args.fov)
    args.out.mkdir(parents=True, exist_ok=True)
    diagonal = math.hypot(args.width, args.height)

    saved = carried_on(args.out, board, detector, lens)
    if args.replay is not None:
        source = Replay(args.replay)
        session = Session(Mode.RECORDING, "replaying", saved)
    else:
        source = Camera(args.device, args.width, args.height)
        status = f"carrying on: {len(saved)} frames already here" if saved else "focus first, then s to start"
        session = Session(Mode.PAUSED, status, saved)
    try:
        while len(session.saved) < args.count:
            gray = source.read(session)
            if gray is None:
                continue
            step(session, source, gray, board, detector, lens, args, diagonal)
    except Stop:
        pass

    counts = region_counts(session.saved, args.width, args.height)
    print(f"{len(session.saved)} frames in {args.out}. Corners per ninth, top row first: {counts.tolist()}")
    toward = reach_counts(session.saved, args.width, args.height)
    print(
        f"Corners beyond {LIMITS.corner_reach:.0%} of the way to each corner of the frame (each needs "
        f"{LIMITS.min_per_corner}): " + ", ".join(f"{name} {n}" for name, n in zip(CORNER_NAMES, toward))
    )
    print(f"Solve: uv run tools/calibrate.py solve {args.out}/*.pgm --squares {args.squares} "
          f"--square-mm {args.square_mm} --marker-mm {args.marker_mm} --dictionary {args.dictionary} --out lens.toml")


if __name__ == "__main__":
    main()
