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
is and how far it's tilted, and how many corners each ninth of the frame has so far. It saves a
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
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from calibrate import Limits, board_arguments, board_from  # noqa: E402

# A ninth of the frame is covered once it has this many corners: well over the solver's floor.
REGION_AIM = 3 * Limits().min_per_region
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
    if ids is None or len(ids) < Limits().min_corners or board.checkCharucoCornersCollinear(ids):
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
    if len(moved) < Limits().min_corners:
        return None
    return float(np.median(moved)) / seconds


def bar(panel, y, value, peak, label):
    filled = int(600 * min(value / peak, 1.0)) if peak > 0 else 0
    cv2.rectangle(panel, (10, y), (10 + filled, y + 22), GREEN, -1)
    cv2.rectangle(panel, (10, y), (610, y + 22), WHITE, 1)
    cv2.putText(panel, label, (620, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 1)


def draw(gray, state, found, counts, saved, args):
    height, width = gray.shape
    if state["zoom"]:
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
        if found is not None:
            for x, y in found["corners"]:
                cv2.circle(image, (int(x), int(y)), 5, YELLOW, -1)
        image = cv2.resize(image, (VIEW_WIDTH, int(height * VIEW_WIDTH / width)), interpolation=cv2.INTER_AREA)

    panel = np.zeros((190, image.shape[1], 3), np.uint8)
    mode = "RECORDING" if state["recording"] else "PAUSED"
    countdown = f"   next in {state['countdown']:.1f} s" if state.get("countdown") is not None else ""
    lines = [f"{mode}   saved {len(saved)} / {args.count}{countdown}    {state['status']}"]
    if found is not None:
        hint = "  closer than the focus: back off" if found["distance"] < args.near else ""
        moving = found.get("speed")
        motion = "" if moving is None else ("   steady" if moving < STEADY_PX_PER_S else "   moving")
        lines.append(f"board: {len(found['corners'])} corners   ~{found['distance']:.2f} m   tilt ~{found['tilt']:.0f} deg{motion}{hint}")
    else:
        lines.append("no board in view")
    short = [name for name, count in zip(REGION_LIST, counts.flatten()) if count < REGION_AIM]
    lines.append("needs: " + (", ".join(short) if short else "every ninth covered"))
    lines.append("s start/pause   space save   u undo   f zoom (focus)   r reset peak   q quit")
    for i, text in enumerate(lines):
        cv2.putText(panel, text, (10, 26 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.62, ORANGE, 2)
    focus_label = f"focus, middle {state['focus']:.0f} (peak {state['focus_peak']:.0f})"
    bar(panel, 128, state["focus"], state["focus_peak"], focus_label)
    if found is not None:
        bar(panel, 158, found["sharpness"], state["board_peak"], "board sharpness")
    return np.vstack([image, panel])


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

    saved = []
    # Frames already in the folder count, so a run that stopped carries on where it was.
    for path in sorted(args.out.glob("capture-*.pgm")):
        earlier = found_board(cv2.imread(str(path), cv2.IMREAD_GRAYSCALE), board, detector, lens)
        if earlier is not None:
            saved.append({**earlier, "path": path})
    state = {
        "recording": args.replay is not None,
        "zoom": False,
        "status": f"carrying on: {len(saved)} frames already here" if saved else "focus first, then s to start",
        "focus": 0.0,
        "focus_peak": 0.0,
        "board_peak": 0.0,
    }

    replay = sorted(args.replay.glob("*.pgm")) if args.replay else None
    camera = None if replay else open_camera(args.device, args.width, args.height)
    previous, previous_time, last_save, index = None, 0.0, 0.0, 0
    while len(saved) < args.count:
        if replay is not None:
            if index >= len(replay):
                break
            gray = cv2.imread(str(replay[index]), cv2.IMREAD_GRAYSCALE)
            index += 1
        else:
            # Unplugged or stalled: wait for the camera to come back, the window kept live, and keep
            # what's saved, rather than stopping or freezing.
            if camera is None:
                if waiting(args.device, f"no camera: {len(saved)} frames saved") == ord("q"):
                    break
                camera = open_camera(args.device, args.width, args.height)
                previous = None
                continue
            ok, raw = camera.read()
            if not ok:
                camera.release()
                camera = None
                continue
            gray = luma(raw, args.width, args.height)

        # Smoothed over a few frames, so sensor noise doesn't make the focus bar jump.
        focus = sharpness(middle(gray))
        state["focus"] = focus if state["focus"] == 0 else 0.7 * state["focus"] + 0.3 * focus
        state["focus_peak"] = max(state["focus_peak"], state["focus"])

        found = found_board(gray, board, detector, lens)
        now = time.time()
        if found is not None:
            state["board_peak"] = max(state["board_peak"], found["sharpness"])
            moving = speed(previous, found, now - previous_time)
            found["speed"] = moving
        steady = found is not None and (replay is not None or (found["speed"] is not None and found["speed"] < STEADY_PX_PER_S))
        previous, previous_time = found, now
        due = replay is not None or now - last_save >= args.every
        state["countdown"] = max(0.0, args.every - (now - last_save)) if state["recording"] else None

        key = -1 if replay is not None else (cv2.waitKey(1) & 0xFF)
        if key == ord("s"):
            state["recording"] = not state["recording"]
            state["status"] = "hold the board still somewhere new" if state["recording"] else "paused"
        if key == ord("f"):
            state["zoom"] = not state["zoom"]
        if key == ord("r"):
            state["focus_peak"] = state["focus"]
            state["board_peak"] = 0.0

        automatic = state["recording"] and steady and due
        if found is not None and (key == ord(" ") or automatic):
            repeat = not is_new(found, saved, diagonal)
            name = args.out / f"capture-{len(saved):03d}.pgm"
            while name.exists():
                name = name.with_name(name.stem + "b.pgm")
            cv2.imwrite(str(name), gray)
            saved.append({**found, "path": name})
            last_save = now
            state["status"] = f"saved {name.name}" + (": much like an earlier one, move somewhere new" if repeat else "")
        elif key == ord(" "):
            state["status"] = "no board in view to save"
        elif state["recording"] and found is None:
            state["status"] = "waiting for the board"
        elif state["recording"] and not steady and due:
            state["status"] = "hold steady to save"
        if key == ord("u") and saved:
            gone = saved.pop()
            gone["path"].unlink(missing_ok=True)
            state["status"] = f"dropped {gone['path'].name}"
        if key == ord("q"):
            break

        if replay is None:
            counts = region_counts(saved, args.width, args.height)
            cv2.imshow("calibration", draw(gray, state, found, counts, saved, args))

    counts = region_counts(saved, args.width, args.height)
    print(f"{len(saved)} frames in {args.out}. Corners per ninth, top row first: {counts.tolist()}")
    print(f"Solve: uv run tools/calibrate.py solve {args.out}/*.pgm --squares {args.squares} "
          f"--square-mm {args.square_mm} --marker-mm {args.marker_mm} --dictionary {args.dictionary} --out lens.toml")


if __name__ == "__main__":
    main()
