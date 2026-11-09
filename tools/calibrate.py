# /// script
# requires-python = ">=3.12"
# dependencies = ["opencv-python-headless==5.0.0.93", "numpy==2.5.4"]
# ///
"""Lens calibration for a Visor camera, from frames `visor record` saved of a ChArUco board.

    uv run tools/calibrate.py board --out board.png
    uv run tools/calibrate.py solve frames/*.pgm --square-mm 21.9 --out lens.toml

`board` writes the board to print. Print it at 100% scale and fix it to something flat and rigid. Its
exact size doesn't change the lens, but its squares must come out square. `solve` finds the board in
each frame, solves the lens as OpenCV does, and prints it twice: as a `[camera.lens]` block for the
camera's entry in Visor's configuration, and as the robot definition takes it, for a robot whose
deploy writes that configuration. It refuses a session that can't give a good lens: too few views,
views that miss part of the frame (the edges are where the distortion is, so also when the views
near one were the outliers dropped), too large an error, or a solve that failed.

The distortion is solved with five terms, the fifth (k3) held at 0: a session that doesn't reach the
frame's corners can't tell k3 from k2, and a free k3 only swings the lens out where no corner was
seen. --free-k3 solves all five, for a lens that needs it; --rational solves eight, for a wide lens.
"""

import argparse
import sys
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import cv2
import numpy as np

DICTIONARIES = {
    "4X4_100": cv2.aruco.DICT_4X4_100,
    "4X4_1000": cv2.aruco.DICT_4X4_1000,
    "5X5_100": cv2.aruco.DICT_5X5_100,
    "6X6_100": cv2.aruco.DICT_6X6_100,
}


@dataclass(frozen=True)
class Board:
    """A ChArUco board: a chessboard with a marker in each white square, so a partly seen board
    still names every corner it shows. The defaults print on letter or A4 paper."""

    columns: int = 11
    rows: int = 8
    square_m: float = 0.022
    marker_m: float = 0.017
    dictionary: str = "5X5_100"

    def opencv(self):
        return cv2.aruco.CharucoBoard(
            (self.columns, self.rows),
            self.square_m,
            self.marker_m,
            cv2.aruco.getPredefinedDictionary(DICTIONARIES[self.dictionary]),
        )


@dataclass(frozen=True)
class Limits:
    """What a session must reach for its lens to be used."""

    # Corners in a view for it to count: four solve a view's pose, more make it trustworthy.
    min_corners: int = 8
    # Views that count, after outliers are dropped. 30 to 50 is the aim.
    min_views: int = 15
    # The solve's RMS reprojection error over every corner. A good session is well under half this.
    max_rms_px: float = 1.0
    # Corners needed in each ninth of the frame, across all views.
    min_per_region: int = 20


class Model(Enum):
    """How many distortion terms the solve fits."""

    # Five terms, the fifth (k3) held at 0.
    K3_FIXED = "k3 fixed"
    # Five terms, all free.
    FIVE = "five terms"
    # Eight terms, for a wide lens.
    RATIONAL = "eight terms"


OPENCV_FLAGS = {
    Model.K3_FIXED: cv2.CALIB_FIX_K3,
    Model.FIVE: 0,
    Model.RATIONAL: cv2.CALIB_RATIONAL_MODEL,
}
# The coefficients each model prints. OpenCV 5 gives fourteen for the rational model, the last six
# (thin prism and tilt) held at 0; Visor and the robot take 4, 5 or 8.
TERMS = {Model.K3_FIXED: 5, Model.FIVE: 5, Model.RATIONAL: 8}


# Frames named alike in two folders make views of one name, so a view is equal only to itself.
@dataclass(eq=False)
class View:
    name: str
    corners: np.ndarray  # n × 2 pixels
    object_points: np.ndarray  # n × 3 metres, on the board
    error_px: float = 0.0


@dataclass
class Lens:
    width: int
    height: int
    camera_matrix: np.ndarray
    distortion: np.ndarray
    rms_px: float
    deviations: np.ndarray  # fx, fy, cx, cy, then the distortion's, one standard deviation each
    views: list
    dropped: list  # (name, why)
    notes: list  # what the solve did that whoever pastes the lens should know
    model: Model

    @property
    def fx(self):
        return float(self.camera_matrix[0, 0])

    @property
    def fy(self):
        return float(self.camera_matrix[1, 1])

    @property
    def cx(self):
        return float(self.camera_matrix[0, 2])

    @property
    def cy(self):
        return float(self.camera_matrix[1, 2])

    def java(self):
        """The lens as a robot's definition takes it (frc-robot-model's CameraConfig.Software.Lens),
        to paste into the camera's Software.visor()...lens(...)."""
        coefficients = ", ".join(f"{c:.9g}" for c in self.distortion)
        return (
            "Lens.builder()\n"
            f"    .width({self.width})\n"
            f"    .height({self.height})\n"
            f"    .fx({self.fx:.6f})\n"
            f"    .fy({self.fy:.6f})\n"
            f"    .cx({self.cx:.6f})\n"
            f"    .cy({self.cy:.6f})\n"
            f"    .distortion(List.of({coefficients}))\n"
            "    .build()\n"
        )

    def toml(self):
        """The block for the camera's entry in Visor's configuration."""
        coefficients = ", ".join(f"{c:.9g}" for c in self.distortion)
        return (
            f"# visor calibrate: {len(self.views)} views, RMS {self.rms_px:.3f} px, {self.model.value}\n"
            "[camera.lens]\n"
            f"width = {self.width}\n"
            f"height = {self.height}\n"
            f"fx = {self.fx:.6f}\n"
            f"fy = {self.fy:.6f}\n"
            f"cx = {self.cx:.6f}\n"
            f"cy = {self.cy:.6f}\n"
            f"distortion = [{coefficients}]\n"
        )


class Refused(Exception):
    """A session whose lens shouldn't be used, and why."""


def find_board(image, board, detector, name, limits):
    """The board's corners in one frame, or why the frame doesn't count."""
    corners, ids, _, _ = detector.detectBoard(image)
    found = 0 if ids is None else len(ids)
    if found < limits.min_corners:
        return None, f"{found} corners, under {limits.min_corners}"
    object_points, image_points = board.matchImagePoints(corners, ids)
    if board.checkCharucoCornersCollinear(ids):
        return None, "its corners are in a line"
    return View(name, image_points.reshape(-1, 2), object_points.reshape(-1, 3)), None


REGION_NAMES = [
    ["top left", "top", "top right"],
    ["left", "middle", "right"],
    ["bottom left", "bottom", "bottom right"],
]


def coverage(views, width, height):
    """Corners found in each ninth of the frame."""
    counts = np.zeros((3, 3), int)
    for view in views:
        columns = np.clip((view.corners[:, 0] * 3 // width).astype(int), 0, 2)
        rows = np.clip((view.corners[:, 1] * 3 // height).astype(int), 0, 2)
        np.add.at(counts, (rows, columns), 1)
    return counts


def thin_regions(views, size, limits):
    """The ninths of the frame with too few corners across these views, by name."""
    counts = coverage(views, *size)
    return [REGION_NAMES[r][c] for r in range(3) for c in range(3) if counts[r, c] < limits.min_per_region]


def not_numbers(rms, camera_matrix, distortion, views):
    """The values a solve gave that aren't finite numbers, by name: a failed fit can come back
    with NaN, which no limit catches and no robot can use."""
    values = {
        "RMS error": rms,
        "fx": camera_matrix[0, 0],
        "fy": camera_matrix[1, 1],
        "cx": camera_matrix[0, 2],
        "cy": camera_matrix[1, 2],
    }
    for n, coefficient in enumerate(distortion):
        values[f"distortion {n + 1}"] = coefficient
    for view in views:
        values[f"{view.name}'s error"] = view.error_px
    return [name for name, value in values.items() if not np.isfinite(value)]


def solve_views(views, size, model):
    flags = OPENCV_FLAGS[model]
    rms, camera_matrix, distortion, _, _, deviations, _, errors = cv2.calibrateCameraExtended(
        [v.object_points.astype(np.float32) for v in views],
        [v.corners.astype(np.float32) for v in views],
        size,
        None,
        None,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-12),
    )
    for view, error in zip(views, errors.ravel()):
        view.error_px = float(error)
    distortion = distortion.ravel()[: TERMS[model]]
    failed = not_numbers(rms, camera_matrix, distortion, views)
    if failed:
        raise Refused(f"the solve failed: its {', '.join(failed)} came out not a number")
    return rms, camera_matrix, distortion, deviations.ravel()


def plural(count, noun):
    return f"{count} {noun}" if count == 1 else f"{count} {noun}s"


def fit(views, size, limits, model):
    """Solves the lens, drops the views far worse than the rest (blurred or misdetected ones), and
    checks and solves again, until no view is dropped. Returns the last solve, the views it used,
    the views dropped and why, and notes on what it kept."""
    dropped, notes = [], []
    while True:
        solved = solve_views(views, size, model)
        median = float(np.median([v.error_px for v in views]))
        outliers = [v for v in views if v.error_px > 3 * median]
        if not outliers:
            return solved, views, dropped, notes
        remaining = [v for v in views if v not in outliers]
        names = ", ".join(v.name for v in outliers)
        if len(remaining) < limits.min_views:
            notes.append(
                f"kept {plural(len(outliers), 'view')} over 3 times the median error ({names}): dropping "
                f"them would leave {len(remaining)}, under {limits.min_views}"
            )
            return solved, views, dropped, notes
        thin = thin_regions(remaining, size, limits)
        if thin:
            raise Refused(
                f"the views over 3 times the median error ({names}) were the only ones near the frame's "
                f"{', '.join(thin)}: hold the board there again, steady and in focus"
            )
        for view in outliers:
            dropped.append((view.name, f"error {view.error_px:.2f} px, over 3 times the median"))
        views = remaining


def solve(images, board=Board(), limits=Limits(), model=Model.K3_FIXED):
    """The lens from frames of the board: a list of (name, 8-bit image)."""
    if not images:
        raise Refused("no frames")
    sizes = {image.shape[:2] for _, image in images}
    if len(sizes) > 1:
        raise Refused(f"the frames aren't all one size: {sorted(sizes)}")
    height, width = sizes.pop()
    views, dropped = [], []
    charuco = board.opencv()
    detector = cv2.aruco.CharucoDetector(charuco)
    for name, image in images:
        view, why = find_board(image, charuco, detector, name, limits)
        if view is None:
            dropped.append((name, why))
        else:
            views.append(view)
    if len(views) < limits.min_views:
        raise Refused(f"the board was found in {len(views)} frames; it needs {limits.min_views}")
    thin = thin_regions(views, (width, height), limits)
    if thin:
        raise Refused(
            f"too few corners near the frame's {', '.join(thin)} (each ninth needs "
            f"{limits.min_per_region}): hold the board there too, the lens bends most at the edges"
        )

    solved, views, outliers, notes = fit(views, (width, height), limits, model)
    rms, camera_matrix, distortion, deviations = solved
    if rms > limits.max_rms_px:
        raise Refused(
            f"RMS error {rms:.2f} px, over {limits.max_rms_px}: blurred frames, a board that isn't flat, "
            "or a board printed out of square"
        )
    return Lens(width, height, camera_matrix, distortion, rms, deviations, views, dropped + outliers, notes, model)


def report(lens):
    lines = [f"{lens.width} × {lens.height}, {len(lens.views)} views, RMS {lens.rms_px:.3f} px, {lens.model.value}"]
    names = ["fx", "fy", "cx", "cy"]
    values = [lens.fx, lens.fy, lens.cx, lens.cy]
    lines.append("  " + "   ".join(f"{n} {v:.2f} ± {d:.2f}" for n, v, d in zip(names, values, lens.deviations)))
    counts = coverage(lens.views, lens.width, lens.height)
    lines.append("  corners by ninth of the frame:")
    lines += ["    " + " ".join(f"{n:5d}" for n in row) for row in counts]
    worst = sorted(lens.views, key=lambda v: -v.error_px)[:5]
    lines.append("  worst views: " + ", ".join(f"{v.name} {v.error_px:.2f} px" for v in worst))
    for name, why in lens.dropped:
        lines.append(f"  left out {name}: {why}")
    for note in lens.notes:
        lines.append(f"  note: {note}")
    return "\n".join(lines)


def read_frames(paths):
    images = []
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise Refused(f"can't read {path}")
        images.append((Path(path).name, image))
    return images


def board_image(board, pixels_per_square, margin=0):
    """The board drawn with a whole number of pixels to a square, as near the number asked as
    OpenCV draws (some sizes trip a rounding check in it), and that number."""
    for nudge in (0, 1, -1, 2, -2, 3, -3):
        pixels = pixels_per_square + nudge
        size = (board.columns * pixels + 2 * margin, board.rows * pixels + 2 * margin)
        try:
            return board.opencv().generateImage(size, marginSize=margin), pixels
        except cv2.error:
            continue
    raise Refused(f"OpenCV can't draw this board at about {pixels_per_square} pixels a square")


def write_board(board, out, dpi):
    pixels_per_m = dpi / 0.0254
    image, pixels = board_image(board, round(board.square_m * pixels_per_m), round(board.square_m * pixels_per_m / 2))
    cv2.imwrite(str(out), image)
    return image.shape[1] / dpi * 25.4, image.shape[0] / dpi * 25.4, pixels / pixels_per_m * 1000


def board_arguments(parser):
    defaults = Board()
    parser.add_argument("--squares", default=f"{defaults.columns}x{defaults.rows}", help="columns x rows")
    parser.add_argument("--square-mm", type=float, default=defaults.square_m * 1000)
    parser.add_argument("--marker-mm", type=float, default=defaults.marker_m * 1000)
    parser.add_argument("--dictionary", choices=sorted(DICTIONARIES), default=defaults.dictionary)


def board_from(args):
    columns, rows = (int(n) for n in args.squares.lower().split("x"))
    return Board(columns, rows, args.square_mm / 1000, args.marker_mm / 1000, args.dictionary)


def model_from(args):
    if args.rational:
        return Model.RATIONAL
    if args.free_k3:
        return Model.FIVE
    return Model.K3_FIXED


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    printing = commands.add_parser("board", help="write the board to print")
    board_arguments(printing)
    printing.add_argument("--out", type=Path, required=True)
    printing.add_argument("--dpi", type=int, default=300)
    solving = commands.add_parser("solve", help="solve the lens from frames of the board")
    board_arguments(solving)
    solving.add_argument("frames", nargs="+", type=Path)
    terms = solving.add_mutually_exclusive_group()
    terms.add_argument("--free-k3", action="store_true", help="solve the fifth distortion term too, held at 0 otherwise")
    terms.add_argument("--rational", action="store_true", help="eight distortion coefficients, for a wide lens")
    solving.add_argument("--out", type=Path, help="also write the [camera.lens] block here")
    args = parser.parse_args(argv)

    board = board_from(args)
    if args.command == "board":
        width_mm, height_mm, square_mm = write_board(board, args.out, args.dpi)
        print(f"{args.out}: {width_mm:.0f} × {height_mm:.0f} mm at {args.dpi} dpi, squares {square_mm:.2f} mm.")
        print("Print at 100% scale and fix it flat to something rigid. Check the squares came out square.")
        return 0
    try:
        lens = solve(read_frames(args.frames), board, model=model_from(args))
    except Refused as why:
        print(f"refused: {why}", file=sys.stderr)
        return 1
    print(report(lens), file=sys.stderr)
    print("\nFor the camera's .lens(...) in a robot's definition:\n" + lens.java(), file=sys.stderr)
    print(lens.toml(), end="")
    if args.out:
        args.out.write_text(lens.toml())
    return 0


if __name__ == "__main__":
    sys.exit(main())
