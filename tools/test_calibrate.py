# /// script
# requires-python = ">=3.12"
# dependencies = ["opencv-python-headless==5.0.0.93", "numpy==2.5.4"]
# ///
"""Tests for calibrate.py, on frames rendered through lenses whose answer is known.

    uv run tools/test_calibrate.py

Each frame is drawn by tracing every pixel back through the lens (its distortion undone) to the
board or the tag, so what the camera sees is exactly what a lens with those values would show.
A session must give back the lens it was rendered with, and a tag rendered through that lens must
come out of Visor's own solve at its true place. That last test runs the `visor` binary: set
VISOR_BIN, or build it first (`cargo build --release`).
"""

import contextlib
import io
import os
import re
import subprocess
import sys
import tempfile
import tomllib
import unittest
from unittest import mock
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

import calibrate

REPO = Path(__file__).resolve().parent.parent
W, H = 1600, 1304  # ThriftyCam
SS = 2  # supersampling, so edges are area-averaged as a sensor's pixels are
TAG_M = 0.1651
BLACK, WHITE = 30.0, 215.0  # print under real light: black isn't 0 and white isn't 255


@dataclass(frozen=True)
class TrueLens:
    name: str
    camera_matrix: np.ndarray
    distortion: np.ndarray


def true_lens(name, f, cx, cy, distortion):
    return TrueLens(name, np.array([[f, 0, cx], [0, f * 1.003, cy], [0, 0, 1.0]]), np.array(distortion, float))


LENSES = [
    # An 80 degree lens, a little barrel distortion.
    true_lens("mild", 1230, 806.5, 647.0, [-0.08, 0.02, 0.0004, -0.0003, 0.0]),
    # The same field with strong barrel distortion, as a cheap wide lens has.
    true_lens("strong", 1225, 791.0, 660.5, [-0.35, 0.15, 0.001, -0.0008, -0.03]),
]


class Renderer:
    """Frames through one lens: each supersampled pixel's ray, computed once."""

    def __init__(self, lens):
        self.lens = lens
        # Supersampled pixel centres, in the frame's pixel-centre coordinates.
        xs = (np.arange(W * SS) + 0.5) / SS - 0.5
        ys = (np.arange(H * SS) + 0.5) / SS - 0.5
        grid = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 1, 2)
        criteria = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-12)
        rays = cv2.undistortPoints(grid, lens.camera_matrix, lens.distortion, criteria=criteria)
        self.rays = rays.reshape(H * SS, W * SS, 2).astype(np.float32)

    def worst_ray_px(self):
        """How far the rays land from their pixels, put back through the lens: the renderer's own error."""
        rays = self.rays[::97, ::89].reshape(-1, 2).astype(np.float64)
        points = np.hstack([rays, np.ones((len(rays), 1))])
        back, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), self.lens.camera_matrix, self.lens.distortion)
        xs = (np.arange(W * SS)[::89] + 0.5) / SS - 0.5
        ys = (np.arange(H * SS)[::97] + 0.5) / SS - 0.5
        pixels = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
        return float(np.abs(back.reshape(-1, 2) - pixels).max())

    def draw(self, R, t, texture, metres_per_texel, origin_m, background):
        """A plane's texture seen through the lens: the plane is z = 0 in its own frame, placed in the
        camera's by R and t; the texture's top-left corner is at origin_m on the plane."""
        normal = R[:, 2].astype(np.float32)
        t = t.astype(np.float32)
        x, y = self.rays[..., 0], self.rays[..., 1]
        along = (normal @ t) / (x * normal[0] + y * normal[1] + normal[2])
        # Where each ray meets the plane, in the plane's frame: R transposed times (point - t).
        px, py, pz = along * x - t[0], along * y - t[1], along - t[2]
        u = px * R[0, 0] + py * R[1, 0] + pz * R[2, 0]
        v = px * R[0, 1] + py * R[1, 1] + pz * R[2, 1]
        columns = ((u - origin_m[0]) / metres_per_texel - 0.5).astype(np.float32)
        rows = ((v - origin_m[1]) / metres_per_texel - 0.5).astype(np.float32)
        seen = cv2.remap(texture, columns, rows, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=-1)
        frame = np.where((seen >= 0) & (along > 0), BLACK + seen / 255 * (WHITE - BLACK), background)
        return frame

    def finish(self, frame, rng):
        frame = cv2.resize(frame, (W, H), interpolation=cv2.INTER_AREA)
        frame = frame + rng.normal(0, 1.5, frame.shape)
        return np.clip(frame, 0, 255).astype(np.uint8)


def background(rng):
    """Low-frequency shading, as a room behind the board gives."""
    base = rng.integers(70, 160, (7, 9)).astype(np.float32)
    return cv2.resize(base, (W * SS, H * SS), interpolation=cv2.INTER_CUBIC)


BOARD = calibrate.Board()


def board_texture():
    """The printed board with a square's width of white paper round it, and its texels a square."""
    board, texels = calibrate.board_image(BOARD, 200)
    return cv2.copyMakeBorder(board, *[texels] * 4, cv2.BORDER_CONSTANT, value=255).astype(np.float32), texels


def rotation(x_deg, y_deg, z_deg):
    """Turned about x, then about the turned y, then about the turned z."""
    turns = [cv2.Rodrigues(np.radians(axis))[0] for axis in ([x_deg, 0, 0], [0, y_deg, 0], [0, 0, z_deg])]
    return turns[0] @ turns[1] @ turns[2]


def placed_at(renderer, R, centre_in_plane, where, distance):
    """The translation that puts a point of the plane at a pixel, at a distance along its ray."""
    column, row = where[0] * (W - 1), where[1] * (H - 1)
    ray = renderer.rays[round((row + 0.5) * SS - 0.5), round((column + 0.5) * SS - 0.5)].astype(np.float64)
    return np.array([ray[0], ray[1], 1.0]) * distance - R @ centre_in_plane


def session(renderer, rng, where_list, distances=(0.32, 0.6)):
    """One frame of the board for each place in the frame given, at a random distance and angle."""
    texture, texels_per_square = board_texture()
    texel_m = BOARD.square_m / texels_per_square
    margin_m = BOARD.square_m
    centre = np.array([BOARD.columns * BOARD.square_m / 2, BOARD.rows * BOARD.square_m / 2, 0.0])
    frames = []
    for n, where in enumerate(where_list):
        R = rotation(rng.uniform(-35, 35), rng.uniform(-35, 35), rng.uniform(-20, 20))
        t = placed_at(renderer, R, centre, where, rng.uniform(*distances))
        frame = renderer.draw(R, t, texture, texel_m, (-margin_m, -margin_m), background(rng))
        frames.append((f"view-{n:02d}", renderer.finish(frame, rng), R, t))
    return frames


def everywhere(rng, count):
    """Places spread over the whole frame, its corners and edges included."""
    grid = [(u, v) for v in (0.12, 0.5, 0.88) for u in (0.1, 0.5, 0.9)]
    return [grid[n % len(grid)] + rng.uniform(-0.06, 0.06, 2) for n in range(count)]


def pixel_error(truth, lens):
    """How far apart the true lens and a solved one put points, over the whole frame."""
    pixels = np.stack(np.meshgrid(np.linspace(0, W - 1, 33), np.linspace(0, H - 1, 27)), -1).reshape(-1, 1, 2)
    criteria = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-12)
    rays = cv2.undistortPoints(pixels, truth.camera_matrix, truth.distortion, criteria=criteria).reshape(-1, 2)
    points = np.hstack([rays, np.ones((len(rays), 1))])
    back, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), lens.camera_matrix, lens.distortion)
    return np.linalg.norm(back.reshape(-1, 2) - pixels.reshape(-1, 2), axis=1)


_renderers = {}


def renderer_for(lens):
    if lens.name not in _renderers:
        _renderers[lens.name] = Renderer(lens)
    return _renderers[lens.name]


class GoodSessions(unittest.TestCase):
    """40 frames reaching every part of the frame: the lens comes back, and the tool chooses the
    terms it has with no flag given."""

    def test_each_lens_is_recovered(self):
        mild, strong = LENSES
        cases = [
            # Each row: the true lens, the terms the tool should choose, and what its report says.
            (mild, calibrate.Model.K3_FIXED, "isn't needed"),
            (strong, calibrate.Model.FIVE, "needs its fifth distortion term"),
        ]
        for truth, model, says in cases:
            with self.subTest(truth.name):
                renderer = renderer_for(truth)
                self.assertLess(renderer.worst_ray_px(), 1e-3, "the renderer's rays")
                frames = frames_for(truth)
                lens = calibrate.solve([(name, image) for name, image, _, _ in frames])
                self.assertEqual(lens.model, model)
                self.assertIn(says, calibrate.report(lens))

                # The corners the board was found at are where the true lens puts them: no offset (a
                # half-pixel slip in the renderer or the tool would show here), only the detector's noise.
                self.assertGreaterEqual(len(lens.views), 34, "views the board was found in")
                offsets = []
                for view in lens.views:
                    _, _, R, t = next(f for f in frames if f[0] == view.name)
                    rvec, _ = cv2.Rodrigues(R)
                    expected, _ = cv2.projectPoints(view.object_points, rvec, t, truth.camera_matrix, truth.distortion)
                    offsets.append(view.corners - expected.reshape(-1, 2))
                offsets = np.vstack(offsets)
                self.assertLess(float(np.abs(offsets.mean(axis=0)).max()), 0.01, "mean corner offset, px")
                self.assertLess(float(np.median(np.linalg.norm(offsets, axis=1))), 0.1, "median corner error, px")

                error = pixel_error(truth, lens)
                print(
                    f"\n  {truth.name}, {lens.terms}: {len(lens.views)} views, RMS {lens.rms_px:.3f} px, the two solves "
                    f"{lens.k3_difference_px:.2f} px apart; "
                    f"against the true lens, median {np.median(error):.3f} px, worst {error.max():.3f} px; "
                    f"fx {lens.fx:.1f} (true {truth.camera_matrix[0, 0]:.1f})",
                    file=sys.stderr,
                )
                self.assertLess(lens.rms_px, 0.3)
                # The solve's own uncertainty is honest: the true values lie within three of its
                # standard deviations, and those are under a pixel.
                solved = np.array([lens.fx, lens.fy, lens.cx, lens.cy])
                true = truth.camera_matrix[[0, 1, 0, 1], [0, 1, 2, 2]]
                for name, s, v, d in zip(["fx", "fy", "cx", "cy"], solved, true, lens.deviations[:4]):
                    self.assertLess(d, 1.0, f"{name}'s standard deviation, px")
                    self.assertLess(abs(s - v), 3 * d, f"{name} against the true {v}")
                # Under a pixel apart anywhere in the frame: at 4 m, under 3 mm across the line of sight.
                self.assertLess(float(error.max()), 1.0, "worst over the frame, px")

    def test_the_limit_decides(self):
        """k3 is held when the two solves are within the limit of each other, and kept past it."""
        frames = [(name, image) for name, image, _, _ in frames_for(LENSES[0])]
        apart = calibrate.solve(frames).k3_difference_px
        self.assertGreater(apart, 0.01)
        cases = [
            ("just inside the limit", 1.05 * apart, calibrate.Model.K3_FIXED),
            ("just past it", 0.95 * apart, calibrate.Model.FIVE),
        ]
        for name, limit, model in cases:
            with self.subTest(name):
                lens = calibrate.solve(frames, limits=calibrate.Limits(max_k3_px=limit))
                self.assertEqual(lens.model, model)
                self.assertAlmostEqual(lens.k3_difference_px, apart)

    def test_the_blocks_keep_five_terms(self):
        """Whichever five terms, both printed blocks carry five coefficients, the last 0 when k3 is
        held, and the TOML's first line says which and why; eight terms print eight."""
        mild, strong = LENSES
        cases = [
            # Each row: the true lens, the terms asked for, then coefficients, k3 held, and the first line.
            (mild, calibrate.Model.CHOOSE_K3, 5, True, "k3 held at 0 (not needed)"),
            (strong, calibrate.Model.CHOOSE_K3, 5, False, "k3 solved (needed)"),
            (mild, calibrate.Model.FIVE, 5, False, "k3 solved (as asked)"),
            (mild, calibrate.Model.RATIONAL, 8, False, "eight terms (as asked)"),
        ]
        for truth, model, terms, k3_zero, first_line in cases:
            with self.subTest(f"{truth.name}, {model.value}"):
                frames = [(name, image) for name, image, _, _ in frames_for(truth)]
                lens = calibrate.solve(frames, model=model)
                block = tomllib.loads(lens.toml())["camera"]["lens"]
                self.assertEqual(len(block["distortion"]), terms)
                java = re.search(r"List\.of\(([^)]*)\)", lens.java()).group(1).split(",")
                self.assertEqual(len(java), terms)
                self.assertIn(first_line, lens.toml().splitlines()[0])
                if k3_zero:
                    self.assertEqual(block["distortion"][4], 0.0)
                    self.assertEqual(float(java[4]), 0.0)
                else:
                    self.assertNotEqual(block["distortion"][4], 0.0)


_frames = {}


def frames_for(lens):
    """The good session of 40 frames through a lens, rendered once."""
    if lens.name not in _frames:
        rng = np.random.default_rng(7)
        _frames[lens.name] = session(renderer_for(lens), rng, everywhere(rng, 40))
    return _frames[lens.name]


class RefusedSessions(unittest.TestCase):
    """Sessions that can't give a good lens are refused, saying why."""

    def test_refusals(self):
        renderer = renderer_for(LENSES[1])
        rng = np.random.default_rng(11)
        middle_only = [(0.5, 0.5) + rng.uniform(-0.05, 0.05, 2) for _ in range(20)]
        cases = [
            ("the board kept in the middle", session(renderer, rng, middle_only, (0.8, 1.0)), "top left"),
            ("too few frames", session(renderer, rng, everywhere(rng, 9)), "it needs 15"),
            ("nothing in view", [("blank", np.full((H, W), 128, np.uint8), None, None)] * 20, "found in 0"),
        ]
        small = cv2.resize(cases[1][1][0][1], (W // 2, H // 2))
        cases.append(("frames of two sizes", cases[1][1] + [("small", small, None, None)], "one size"))
        for name, frames, says in cases:
            with self.subTest(name):
                with self.assertRaises(calibrate.Refused) as refused:
                    calibrate.solve([(n, image) for n, image, _, _ in frames])
                self.assertIn(says, str(refused.exception))


EVERY_NINTH = [(row, column) for row in range(3) for column in range(3)]
BUT_TOP_LEFT = EVERY_NINTH[1:]


def flat_view(name, ninths, rng):
    """A view with 25 corners in each ninth of the frame named. OpenCV's solve is never run on it:
    the tests that use it stand in their own, so only where its corners fall matters."""
    corners = [
        np.column_stack([rng.uniform(c * W / 3, (c + 1) * W / 3, 25), rng.uniform(r * H / 3, (r + 1) * H / 3, 25)])
        for r, c in ninths
    ]
    corners = np.vstack(corners)
    return calibrate.View(name, corners, np.zeros((len(corners), 3)))


def standing_in(views):
    """What solve() needs to take these views as the ones it found: a frame of the right size for
    each, and a find_board that hands back the view for each frame in turn."""
    frames = [(view.name, np.broadcast_to(np.uint8(0), (H, W))) for view in views]
    queue = []

    def find_board(image, board, detector, name, limits):
        if not queue:
            queue.extend(views)
        return queue.pop(0), None

    return frames, mock.patch.object(calibrate, "find_board", find_board)


def solver_giving(error_of):
    """A solve_views that gives each view the error error_of(view, every name in the solve)
    returns, as OpenCV would after fitting that set of views."""

    def solve_views(views, size, model):
        present = {view.name for view in views}
        for view in views:
            view.error_px = error_of(view, present)
        rms = float(np.sqrt(np.mean([view.error_px**2 for view in views])))
        return rms, LENSES[0].camera_matrix.copy(), LENSES[0].distortion.copy(), np.full(9, 0.1)

    return solve_views


def heavy_hides_moderate(view, present):
    """Two badly wrong views pull the fit, so the good ones' errors rise and hide a third, milder
    outlier until the two are gone."""
    if view.name.startswith("heavy"):
        return 5.0
    if view.name.startswith("moderate"):
        return 1.0
    return 0.4 if "heavy-0" in present else 0.1


def bad_is_bad(view, present):
    return 2.0 if view.name.startswith("bad") else 0.1


class OutlierDrops(unittest.TestCase):
    """Views far worse than the rest are dropped, and what is left is checked and solved again until
    nothing more is dropped; a drop that would leave too little is refused, or said."""

    def test_drops(self):
        rng = np.random.default_rng(5)
        good = [flat_view(f"good-{n:02d}", EVERY_NINTH, rng) for n in range(16)]
        no_corner = [flat_view(f"good-{n:02d}", BUT_TOP_LEFT, rng) for n in range(16)]
        heavy = [flat_view(f"heavy-{n}", EVERY_NINTH, rng) for n in range(2)]
        moderate = [flat_view("moderate-0", EVERY_NINTH, rng)]
        bad = [flat_view(f"bad-{n}", EVERY_NINTH, rng) for n in range(2)]
        # Frames named alike in two folders: a bad view named as a good one is dropped alone.
        twin = flat_view("good-00", EVERY_NINTH, rng)
        cases = [
            # Each row: views, their errors, then views kept, names dropped, and a note or a refusal.
            ("nothing to drop", good, bad_is_bad, 16, [], None, None),
            ("a second round finds what the first hid", good + heavy + moderate, heavy_hides_moderate, 16,
             ["heavy-0", "heavy-1", "moderate-0"], None, None),
            ("outliers were the only views near an edge", no_corner + bad, bad_is_bad, None, None, None,
             "were the only ones near the frame's top left"),
            ("too few would remain", good[:14] + bad[:1], bad_is_bad, 15, [], "kept 1 view over 3 times", None),
            ("views that share a name", good + [twin], lambda view, present: 2.0 if view is twin else 0.1, 16,
             ["good-00"], None, None),
        ]
        for name, views, error_of, kept, dropped, note, refusal in cases:
            with self.subTest(name):
                frames, found = standing_in(views)
                with found, mock.patch.object(calibrate, "solve_views", solver_giving(error_of)):
                    if refusal is not None:
                        with self.assertRaises(calibrate.Refused) as refused:
                            calibrate.solve(frames)
                        self.assertIn(refusal, str(refused.exception))
                        continue
                    lens = calibrate.solve(frames)
                self.assertEqual(len(lens.views), kept, "views kept")
                self.assertEqual(sorted(n for n, _ in lens.dropped), dropped, "views dropped")
                if note is None:
                    self.assertEqual(lens.notes, [])
                else:
                    self.assertEqual(len(lens.notes), 1)
                    self.assertIn(note, lens.notes[0])
                    self.assertIn(note, calibrate.report(lens))


def reaching(name, rng, reach=(0.65,) * 4, extra=(0,) * 4):
    """A view with corners spread over the frame, none further toward each of the frame's corners
    (top left, top right, bottom left, bottom right) than reach says, as a share of the way from the
    middle, and extra corners just past 70% of the way toward each."""
    across, down = rng.uniform(-1, 1, (2, 3000))
    quarter = 2 * (down >= 0) + (across >= 0)
    keep = np.hypot(across, down) / np.sqrt(2) <= np.array(reach)[quarter]
    across, down = list(across[keep]), list(down[keep])
    for (sign_across, sign_down), count in zip([(-1, -1), (1, -1), (-1, 1), (1, 1)], extra):
        across += [sign_across * 0.72] * count
        down += [sign_down * 0.72] * count
    corners = np.column_stack([(np.array(across) + 1) * (W - 1) / 2, (np.array(down) + 1) * (H - 1) / 2])
    return calibrate.View(name, corners, np.zeros((len(corners), 3)))


COUNTS = "a partly visible board counts"


class CornerReach(unittest.TestCase):
    """A session is refused unless its corners reach out toward each of the frame's four corners:
    the ninths alone pass a board that never left the inner part of the corner ninths."""

    def test_rows(self):
        rng = np.random.default_rng(12)
        inner = [reaching(f"good-{n:02d}", rng) for n in range(15)]
        cases = [
            # Each row: the views, then what the refusal says (None when the session is taken), and a
            # corner it must not name.
            ("every corner reached", inner[:14] + [reaching("good-14", rng, (1.0,) * 4)], None, None),
            ("nothing past 65%, as a board kept well in from the corners", inner,
             ["top left, top right, bottom left, bottom right corners", COUNTS], None),
            ("one corner short", inner[:14] + [reaching("good-14", rng, (1.0, 0.65, 1.0, 1.0))],
             ["toward the frame's top right corner", COUNTS], "top left"),
            ("ten corners just past 70% toward each", inner[:14] + [reaching("good-14", rng, extra=(10,) * 4)], None, None),
            ("nine toward one", inner[:14] + [reaching("good-14", rng, extra=(10, 10, 9, 10))],
             ["toward the frame's bottom left corner", COUNTS], "top left"),
            ("the outliers were the only views reaching out", inner + [reaching(f"bad-{n}", rng, (1.0,) * 4) for n in range(2)],
             ["were the only ones near the frame's top left corner, top right corner"], None),
        ]
        for name, views, refusal, not_named in cases:
            with self.subTest(name):
                frames, found = standing_in(views)
                with found, mock.patch.object(calibrate, "solve_views", solver_giving(bad_is_bad)):
                    if refusal is None:
                        lens = calibrate.solve(frames)
                        self.assertEqual(len(lens.views), len(views))
                        continue
                    with self.assertRaises(calibrate.Refused) as refused:
                        calibrate.solve(frames)
                for says in refusal:
                    self.assertIn(says, str(refused.exception))
                if not_named is not None:
                    self.assertNotIn(not_named, str(refused.exception))


def opencv_giving(rms=0.2, fx=1230.0, k1=-0.08, view_error=0.2):
    """OpenCV's solve as it can come back from a failed fit: the values given, the rest ordinary."""

    def calibrate_camera_extended(object_points, image_points, size, *rest, **options):
        matrix = np.array([[fx, 0, 800.0], [0, 1231.0, 650.0], [0, 0, 1]])
        distortion = np.array([[k1, 0.02, 0.0, 0.0, 0.0]])
        errors = np.full((len(object_points), 1), 0.2)
        errors[0] = view_error
        return rms, matrix, distortion, None, None, np.full((18, 1), 0.1), None, errors

    return calibrate_camera_extended


class FailedSolves(unittest.TestCase):
    """A solve that comes back with an error or a parameter that isn't a number is refused, not
    printed as a lens block that wouldn't parse."""

    def test_refusals(self):
        rng = np.random.default_rng(9)
        views = [flat_view(f"good-{n:02d}", EVERY_NINTH, rng) for n in range(16)]
        cases = [
            ("the error is NaN", opencv_giving(rms=float("nan")), "RMS error"),
            ("fx is NaN", opencv_giving(fx=float("nan")), "fx"),
            ("k1 is infinite", opencv_giving(k1=float("inf")), "distortion 1"),
            ("one view's error is NaN", opencv_giving(view_error=float("nan")), "good-00"),
        ]
        for name, opencv, says in cases:
            frames, found = standing_in(views)
            with self.subTest(name), found, mock.patch.object(calibrate.cv2, "calibrateCameraExtended", opencv):
                with self.assertRaises(calibrate.Refused) as refused:
                    calibrate.solve(frames)
                self.assertIn("not a number", str(refused.exception))
                self.assertIn(says, str(refused.exception))


class VisorWrittenFrames(unittest.TestCase):
    """A PGM written by Visor's own writer (testdata/visor-written.pgm, which Visor's test
    tests/pgm_for_calibrate.rs keeps matching what `visor record` writes) is read with its pixels."""

    def test_the_pixels_come_through(self):
        width, height = 37, 23
        expected = ((np.arange(width * height) + 10) % 256).astype(np.uint8).reshape(height, width)
        [(name, image)] = calibrate.read_frames([REPO / "testdata" / "visor-written.pgm"])
        self.assertEqual(name, "visor-written.pgm")
        self.assertEqual(image.dtype, np.uint8)
        np.testing.assert_array_equal(image, expected)


def visor_binary():
    given = os.environ.get("VISOR_BIN")
    if given:
        return Path(given)
    built = REPO / "target" / "release" / "visor"
    return built if built.exists() else None


def tag_texture(tag_id, cell=40):
    """The tag with its one-cell white quiet zone, 10 cells across."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    code = cv2.aruco.generateImageMarker(dictionary, tag_id, 8 * cell)
    out = np.full((10 * cell, 10 * cell), 255, np.uint8)
    out[cell : 9 * cell, cell : 9 * cell] = code
    return out.astype(np.float32), TAG_M / (8 * cell)


class ThroughVisor(unittest.TestCase):
    """The tool's lens block, pasted into Visor's configuration, solves a tag the true lens drew to its
    true place."""

    def test_a_tag_near_the_frame_edge_is_solved_at_its_place(self):
        binary = visor_binary()
        if binary is None:
            self.skipTest("no visor binary: build it (cargo build --release) or set VISOR_BIN")
        truth = LENSES[1]
        renderer = renderer_for(truth)
        rng = np.random.default_rng(3)
        frames = session(renderer, rng, everywhere(rng, 40))

        # A tag 3.5 m out near the bottom-right corner, where this lens bends most, turned 20 degrees.
        # The tag's frame (OpenCV's): x right and y up as printed, z out of its face, so facing the
        # camera is a half turn about x.
        R = rotation(180, 20, 4)
        t = placed_at(renderer, R, np.zeros(3), (0.88, 0.84), 3.5)
        texture, texel_m = tag_texture(7)
        half = texture.shape[0] * texel_m / 2
        # The texture runs down the page, the tag's y up: flip it so the printed top is +y.
        frame = renderer.draw(R, t, np.ascontiguousarray(texture[::-1]), texel_m, (-half, -half), background(rng))
        image = renderer.finish(frame, rng)

        (REPO / "target").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=REPO / "target") as scratch:
            scratch = Path(scratch)
            pgms = []
            for name, board, _, _ in frames:
                pgms.append(scratch / f"{name}.pgm")
                cv2.imwrite(str(pgms[-1]), board)
            block = scratch / "lens.toml"
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(calibrate.main(["solve", *map(str, pgms), "--out", str(block)]), 0)
            block_text = block.read_text()
            lens = tomllib.loads(block_text)["camera"]["lens"]
            config = scratch / "visor.toml"
            camera = (REPO / "deploy" / "visor.example.toml").read_text().split("[camera.lens]")[0]
            config.write_text(camera + block.read_text())
            tag = scratch / "tag.pgm"
            cv2.imwrite(str(tag), image)

            def solved(*how):
                out = subprocess.run(
                    [str(binary), "detect", *how, str(tag)], capture_output=True, text=True, check=True
                ).stdout
                found = re.search(r"tag\s+7\s+best at \(([-\d.]+), ([-\d.]+), ([-\d.]+)\) m", out)
                self.assertIsNotNone(found, out)
                return np.array([float(v) for v in found.groups()])

            calibrated = solved("--config", str(config), "--camera", "front-left")
            pinhole = ",".join(f"{lens[k]:.10g}" for k in ("fx", "fy", "cx", "cy"))
            without = solved(f"--lens={pinhole}")
        off = np.linalg.norm(calibrated - t)
        off_without = np.linalg.norm(without - t)
        print(
            f"\n  tag at {np.linalg.norm(t):.2f} m: {100 * off:.1f} cm off with the calibrated lens, "
            f"{100 * off_without:.1f} cm without its distortion",
            file=sys.stderr,
        )
        self.assertEqual((lens["width"], lens["height"]), (W, H))
        # The same lens as a robot's definition takes it.
        # This lens has a real k3, and the tool keeps it with no flag given.
        self.assertIn("k3 solved (needed)", block_text)
        java = calibrate.solve([(n, i) for n, i, _, _ in frames]).java()
        self.assertIn(f".width({W})", java)
        self.assertAlmostEqual(float(re.search(r"\.fx\(([\d.]+)\)", java).group(1)), lens["fx"], places=5)
        self.assertLess(off, 0.01 * np.linalg.norm(t), "within 1% of the distance")
        self.assertGreater(off_without, 5 * off, "the distortion matters there")


class PrintedBoard(unittest.TestCase):
    """The board `board` writes is the board `solve` looks for, at the size asked."""

    def test_the_written_board_is_found_whole(self):
        # The default board, and a bought one: 15 x 15 squares of 1.125 in with 0.875 in markers,
        # 112 markers, so ids past 100 and a 1000-marker dictionary.
        bought = ["--squares", "15x15", "--square-mm", "28.575", "--marker-mm", "22.225", "--dictionary", "4X4_1000"]
        cases = [
            ("the default board", [], BOARD, 300),
            ("a bought board", bought, calibrate.Board(15, 15, 0.028575, 0.022225, "4X4_1000"), 100),
        ]
        (REPO / "target").mkdir(exist_ok=True)
        for why, args, board, dpi in cases:
            with self.subTest(why), tempfile.TemporaryDirectory(dir=REPO / "target") as scratch:
                out = Path(scratch) / "board.png"
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(calibrate.main(["board", *args, "--out", str(out), "--dpi", str(dpi)]), 0)
                image = cv2.imread(str(out), cv2.IMREAD_GRAYSCALE)
                square_px = board.square_m / 0.0254 * dpi
                self.assertAlmostEqual(image.shape[1], (board.columns + 1) * square_px, delta=board.columns + 4)
                _, ids, _, _ = cv2.aruco.CharucoDetector(board.opencv()).detectBoard(image)
                self.assertEqual(len(ids), (board.columns - 1) * (board.rows - 1))


if __name__ == "__main__":
    unittest.main(verbosity=2)
