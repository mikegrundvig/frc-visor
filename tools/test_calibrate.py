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
    """40 frames reaching every part of the frame: the lens comes back."""

    def test_each_lens_is_recovered(self):
        for truth in LENSES:
            with self.subTest(truth.name):
                renderer = renderer_for(truth)
                self.assertLess(renderer.worst_ray_px(), 1e-3, "the renderer's rays")
                rng = np.random.default_rng(7)
                frames = session(renderer, rng, everywhere(rng, 40))
                lens = calibrate.solve([(name, image) for name, image, _, _ in frames])

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
                    f"\n  {truth.name}: {len(lens.views)} views, RMS {lens.rms_px:.3f} px; "
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
            lens = tomllib.loads(block.read_text())["camera"]["lens"]
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
        self.assertLess(off, 0.01 * np.linalg.norm(t), "within 1% of the distance")
        self.assertGreater(off_without, 5 * off, "the distortion matters there")


class PrintedBoard(unittest.TestCase):
    """The board `board` writes is the board `solve` looks for, at the size asked."""

    def test_the_written_board_is_found_whole(self):
        (REPO / "target").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=REPO / "target") as scratch:
            out = Path(scratch) / "board.png"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(calibrate.main(["board", "--out", str(out), "--dpi", "300"]), 0)
            image = cv2.imread(str(out), cv2.IMREAD_GRAYSCALE)
        square_px = BOARD.square_m / 0.0254 * 300
        self.assertAlmostEqual(image.shape[1], (BOARD.columns + 1) * square_px, delta=BOARD.columns + 4)
        _, ids, _, _ = cv2.aruco.CharucoDetector(BOARD.opencv()).detectBoard(image)
        self.assertEqual(len(ids), (BOARD.columns - 1) * (BOARD.rows - 1))


if __name__ == "__main__":
    unittest.main(verbosity=2)
