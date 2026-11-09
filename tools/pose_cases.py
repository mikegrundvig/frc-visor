# /// script
# requires-python = ">=3.12"
# dependencies = ["opencv-python-headless==5.0.0.93", "numpy==2.5.4"]
# ///
"""Writes testdata/pose-cases.json: tag corners and what OpenCV makes of them, to check Visor's lens
undistortion and pose solve against.

Run from the repository root:

    uv run tools/pose_cases.py

Each case is a random lens (five or eight distortion coefficients, OpenCV's order), a tag seen from
0.5 to 10 m at up to 70 degrees, its corners projected through the lens (OpenCV's corner order),
with pixel noise in half the cases, and OpenCV's answers: the undistorted corners, and
solvePnPGeneric's two SOLVEPNP_IPPE_SQUARE poses, best first, with their reprojection errors.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np

TAG_M = 0.1651
W, H = 1600, 1304
OUT = Path(__file__).resolve().parent.parent / "testdata" / "pose-cases.json"


def square(size):
    half = size / 2
    return np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float64)


def random_case(rng):
    f = rng.uniform(800, 1400)
    K = np.array([[f, 0, W / 2 + rng.uniform(-40, 40)], [0, f * rng.uniform(0.98, 1.02), H / 2 + rng.uniform(-40, 40)], [0, 0, 1.0]])
    distortion = [rng.uniform(-0.3, 0.3), rng.uniform(-0.2, 0.2), rng.uniform(-0.003, 0.003), rng.uniform(-0.003, 0.003), rng.uniform(-0.05, 0.05)]
    if rng.random() < 0.5:
        distortion += [rng.uniform(-0.3, 0.3), rng.uniform(-0.2, 0.2), rng.uniform(-0.05, 0.05)]
    D = np.array(distortion)
    distance = rng.uniform(0.5, 10)
    turn = cv2.Rodrigues(np.array([rng.uniform(-1.2, 1.2), rng.uniform(-1.2, 1.2), rng.uniform(-math.pi, math.pi)]))[0]
    # Tags face the camera: OpenCV's tag frame has z out of the tag's face, so a tag seen from the
    # front has its z toward the camera. Start from a tag square to the camera and turn it.
    facing_camera = np.diag([1.0, -1.0, -1.0])
    rvec = cv2.Rodrigues(turn @ facing_camera)[0].ravel()
    tvec = np.array([rng.uniform(-0.4, 0.4) * distance, rng.uniform(-0.35, 0.35) * distance, distance])
    obj = square(TAG_M)
    corners, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    corners = corners.reshape(4, 2)
    if rng.random() < 0.5:
        corners = corners + rng.normal(0, 0.5, corners.shape)
    in_frame = (corners[:, 0] > 2).all() and (corners[:, 0] < W - 2).all() and (corners[:, 1] > 2).all() and (corners[:, 1] < H - 2).all()
    # Seen from the front (the tag's z points back toward the camera) and not edge-on.
    normal = cv2.Rodrigues(rvec)[0][:, 2]
    facing = -float(np.dot(normal, tvec / np.linalg.norm(tvec)))
    if not in_frame or facing < 0.35:
        return None
    ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(obj, corners.reshape(4, 1, 2), K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or len(rvecs) != 2:
        return None
    undistorted = cv2.undistortPoints(corners.reshape(4, 1, 2), K, D).reshape(4, 2)
    return {
        "fx": K[0, 0], "fy": K[1, 1], "cx": K[0, 2], "cy": K[1, 2],
        "distortion": D.tolist(),
        "corners": corners.ravel().tolist(),
        "undistorted": undistorted.ravel().tolist(),
        "poses": [
            {"rvec": r.ravel().tolist(), "tvec": t.ravel().tolist(), "error": float(e)}
            for r, t, e in zip(rvecs, tvecs, errors.ravel())
        ],
    }


def main():
    rng = np.random.default_rng(2026)
    cases = []
    while len(cases) < 300:
        case = random_case(rng)
        if case is not None:
            cases.append(case)
    OUT.write_text(json.dumps({"tag_size_m": TAG_M, "opencv": cv2.__version__, "cases": cases}) + "\n")
    print(len(cases), "cases,", OUT.stat().st_size // 1024, "KB")


if __name__ == "__main__":
    main()
