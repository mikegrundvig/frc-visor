# /// script
# requires-python = ">=3.12"
# dependencies = ["opencv-python-headless==5.0.0.93", "numpy==2.5.4"]
# ///
"""Tests for capture.py's choices of which frames to save, on boards rendered as test_calibrate.py
renders them, and a recording replayed from saved frames.

    uv run tools/test_capture.py
"""

import argparse
import contextlib
import io
import math
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

import calibrate
import capture
from test_calibrate import LENSES, H, REPO, W, everywhere, renderer_for, session

DIAGONAL = math.hypot(W, H)


def found(center=(800.0, 650.0), distance=1.0, tilt=10.0):
    """What found_board says of a board, for the checks that only read where it is."""
    return {"center": np.array(center), "distance": distance, "tilt": tilt}


def seen(ids, corners):
    """What found_board says of a board, for the checks that only read its corners."""
    return {"ids": np.array(ids), "corners": np.array(corners, float)}


class Sharpness(unittest.TestCase):
    """The focus bar's number falls as the board blurs, and an empty patch is 0."""

    def test_it_falls_with_blur(self):
        rng = np.random.default_rng(2)
        _, image, _, _ = session(renderer_for(LENSES[0]), rng, [(0.5, 0.5)])[0]
        values = [capture.sharpness(capture.middle(image))]
        for sigma in (0.8, 1.5, 3.0):
            values.append(capture.sharpness(capture.middle(cv2.GaussianBlur(image, (0, 0), sigma))))
        self.assertEqual(values, sorted(values, reverse=True), "sharpness by blur")
        self.assertGreater(values[0], 2 * values[-1])
        self.assertEqual(capture.sharpness(np.zeros((0, 0), np.uint8)), 0.0)


class IsNew(unittest.TestCase):
    """A board counts as new when it is somewhere else, at another distance, or tilted otherwise."""

    def test_rows(self):
        saved = [found()]
        cases = [
            ("the same place", found(), False),
            ("a little moved, nearer and turned", found((850, 690), 0.9, 18), False),
            ("elsewhere in the frame", found((800 + 0.15 * DIAGONAL, 650)), True),
            ("much nearer", found(distance=0.7), True),
            ("much further", found(distance=1.3), True),
            ("tilted further", found(tilt=25), True),
        ]
        for name, candidate, new in cases:
            with self.subTest(name):
                self.assertEqual(capture.is_new(candidate, saved, DIAGONAL), new)
        self.assertTrue(capture.is_new(found(), [], DIAGONAL), "nothing saved yet")


class RegionCounts(unittest.TestCase):
    """Corners are counted in the ninth of the frame they fall in, the last row and column included."""

    def test_rows(self):
        cases = [
            ("top left", (0, 0), (0, 0)),
            ("just right of the first third", (W / 3, 10), (0, 1)),
            ("bottom right pixel", (W - 1, H - 1), (2, 2)),
            ("middle", (W / 2, H / 2), (1, 1)),
            ("left, bottom third", (5, 2 * H / 3 + 1), (2, 0)),
        ]
        for name, corner, cell in cases:
            with self.subTest(name):
                counts = capture.region_counts([{"corners": np.array([corner])}], W, H)
                expected = np.zeros((3, 3), int)
                expected[cell] = 1
                np.testing.assert_array_equal(counts, expected)

    def test_it_agrees_with_the_solver(self):
        rng = np.random.default_rng(4)
        corners = rng.uniform((0, 0), (W, H), (400, 2))
        view = calibrate.View("v", corners, np.zeros((400, 3)))
        np.testing.assert_array_equal(capture.region_counts([{"corners": corners}], W, H), calibrate.coverage([view], W, H))
        # Toward the frame's corners, the live view's count and its "enough" are the solver's.
        limits = calibrate.Limits()
        cases = [
            ("spread over the frame", [corners]),
            ("kept in from the corners", [corners[np.hypot(corners[:, 0] / W - 0.5, corners[:, 1] / H - 0.5) < 0.45]]),
            ("nothing saved", []),
        ]
        for name, saved in cases:
            with self.subTest(name):
                views = [calibrate.View("v", c, np.zeros((len(c), 3))) for c in saved]
                toward = capture.reach_counts([{"corners": c} for c in saved], W, H)
                stacked = np.vstack(saved) if saved else np.zeros((0, 2))
                self.assertEqual(toward, calibrate.reach(stacked, (W, H), limits))
                self.assertEqual(calibrate.short_corners(toward, capture.LIMITS), calibrate.unreached(views, (W, H), limits))


class Speed(unittest.TestCase):
    """How fast the board moves: the median over the corners seen in both frames, or None."""

    def test_rows(self):
        ids = list(range(10))
        still = [(10.0 * i, 20.0) for i in ids]
        moved = [(x + 3, y + 4) for x, y in still]
        cases = [
            ("no earlier frame", None, seen(ids, still), 0.1, None),
            ("no time between them", seen(ids, still), seen(ids, still), 0.0, None),
            ("still", seen(ids, still), seen(ids, still), 0.1, 0.0),
            ("5 px in a tenth of a second", seen(ids, still), seen(ids, moved), 0.1, 50.0),
            ("only the shared corners count", seen(ids, still), seen(ids + [99], moved + [(900.0, 900.0)]), 0.1, 50.0),
            ("too few shared corners", seen(ids[:7], still[:7]), seen(ids[:7], moved[:7]), 0.1, None),
            ("other corners entirely", seen(ids, still), seen([i + 50 for i in ids], moved), 0.1, None),
        ]
        for name, before, after, seconds, expected in cases:
            with self.subTest(name):
                result = capture.speed(before, after, seconds)
                if expected is None:
                    self.assertIsNone(result)
                else:
                    self.assertAlmostEqual(result, expected)


class Scripted(capture.Camera):
    """A camera that gives one frame again and again and presses the keys given, one a frame. Its
    steady and due are the camera's own."""

    def __init__(self, keys):
        self.keys = list(keys)

    def key(self):
        return self.keys.pop(0)

    def show(self, image):
        pass


class Keys(unittest.TestCase):
    """The keys do what the view says, frame by frame, with the camera's rules for steady and due."""

    def test_a_session(self):
        rng = np.random.default_rng(8)
        _, image, _, _ = session(renderer_for(LENSES[0]), rng, [(0.5, 0.5)])[0]
        board = calibrate.Board().opencv()
        detector = cv2.aruco.CharucoDetector(board)
        lens = capture.guessed_lens(W, H, 80)
        # Each row: the key pressed on that frame, then frames saved, what's shown and the status after it.
        rows = [
            (-1, 0, capture.Shown.WHOLE, "focus first"),
            (ord(" "), 1, capture.Shown.WHOLE, "saved capture-000.pgm"),
            (ord(" "), 2, capture.Shown.WHOLE, "much like an earlier one"),
            (ord("u"), 1, capture.Shown.WHOLE, "dropped capture-001.pgm"),
            # Recording, and the board is still: a save is due at once, as --every is 0.
            (ord("s"), 2, capture.Shown.WHOLE, "saved capture-001.pgm"),
            (ord("s"), 2, capture.Shown.WHOLE, "paused"),
            (ord("f"), 2, capture.Shown.MIDDLE, "paused"),
            (ord("f"), 2, capture.Shown.WHOLE, "paused"),
        ]
        (REPO / "target").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=REPO / "target") as scratch:
            args = argparse.Namespace(out=Path(scratch), every=0.0, count=50, width=W, height=H, near=0.8)
            source = Scripted([key for key, _, _, _ in rows] + [ord("q")])
            state = capture.Session(capture.Mode.PAUSED, "focus first, then s to start")
            for n, (key, saved, shown, status) in enumerate(rows):
                with self.subTest(f"frame {n}, key {key}"):
                    capture.step(state, source, image, board, detector, lens, args, DIAGONAL)
                    self.assertEqual(len(state.saved), saved, "frames saved")
                    self.assertEqual(state.shown, shown)
                    self.assertIn(status, state.status)
            self.assertEqual(sorted(p.name for p in Path(scratch).glob("*.pgm")), ["capture-000.pgm", "capture-001.pgm"])
            with self.assertRaises(capture.Stop):
                capture.step(state, source, image, board, detector, lens, args, DIAGONAL)


class Replay(unittest.TestCase):
    """A recording replayed from saved frames saves each frame the board is in, and carries on past
    a frame in --out an interrupted run left half written."""

    def test_a_half_written_frame_is_skipped(self):
        rng = np.random.default_rng(6)
        frames = session(renderer_for(LENSES[0]), rng, everywhere(rng, 5))
        (REPO / "target").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=REPO / "target") as scratch:
            replay, out = Path(scratch) / "replay", Path(scratch) / "out"
            replay.mkdir()
            out.mkdir()
            for name, image, _, _ in frames:
                cv2.imwrite(str(replay / f"{name}.pgm"), image)
            cv2.imwrite(str(out / "capture-000.pgm"), frames[0][1])
            (out / "capture-001.pgm").write_bytes(f"P5\n{W} {H}\n255\n".encode() + bytes(1000))
            printed = io.StringIO()
            with contextlib.redirect_stdout(printed):
                capture.main(["unused", "--out", str(out), "--replay", str(replay)])
            self.assertIn("6 frames in", printed.getvalue(), "the earlier frame and the five replayed")
            self.assertIn("left out capture-001.pgm", printed.getvalue())
            self.assertEqual(len(list(out.glob("capture-*.pgm"))), 7)


if __name__ == "__main__":
    unittest.main(verbosity=2)
