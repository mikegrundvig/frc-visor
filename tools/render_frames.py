# /// script
# requires-python = ">=3.12"
# dependencies = ["opencv-python-headless==5.0.0.93", "numpy==2.5.4"]
# ///
"""Renders the test frames in testdata/: ThriftyCam-sized images of AprilTags with known corners.

Run from the repository root:

    uv run tools/render_frames.py

Each scene writes testdata/<name>.png and testdata/<name>.json. The JSON's corners are in NAPA's
convention, the one Visor sends: OpenCV's corner order (the printed tag's top-left, top-right,
bottom-right, bottom-left) and pixel-centre coordinates (pixel (0, 0)'s centre is at 0, 0). Its pose
is the true one, as Visor solves it: OpenCV's rvec and tvec of the tag in the camera's frame, the
tag's own frame with x right and y up as printed and z out of its face (src/pose.rs).
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np

W, H = 1600, 1304  # ThriftyCam
FOCAL = math.hypot(W, H) / 2 / math.tan(math.radians(40))  # 80 degree diagonal lens
TAG_M = 0.1651  # an FRC tag's black square
SS = 3  # supersampling, so edges are area-averaged as a sensor's pixels are
K = np.array([[FOCAL, 0, W / 2], [0, FOCAL, H / 2], [0, 0, 1.0]])
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
OUT = Path(__file__).resolve().parent.parent / "testdata"

# name: the tags, each (id, distance m, yaw degrees, where its centre sits as a fraction of the frame)
SCENES = {
    "one-tag": [(7, 2.0, 30, (0.45, 0.5))],
    "two-tags": [(3, 3.0, 0, (0.3, 0.4)), (12, 8.0, 20, (0.7, 0.6))],
    "steep-far": [(21, 6.0, 55, (0.5, 0.5))],
    "empty": [],
}


def tag_image(tag_id, cell=24):
    """The tag with its one-cell white quiet zone, 10 cells across."""
    code = cv2.aruco.generateImageMarker(DICT, tag_id, 8 * cell)
    out = np.full((10 * cell, 10 * cell), 255, np.uint8)
    out[cell : 9 * cell, cell : 9 * cell] = code
    return out


def background(rng):
    """Low-frequency shading plus dark and light blocks, as a field wall and bumpers give."""
    base = cv2.resize(
        rng.integers(60, 200, (9, 11), dtype=np.uint8).astype(np.float32), (W, H), interpolation=cv2.INTER_CUBIC
    )
    for _ in range(12):
        x, y = int(rng.integers(0, W)), int(rng.integers(0, H))
        w, h = int(rng.integers(20, 300)), int(rng.integers(20, 300))
        cv2.rectangle(base, (x, y), (x + w, y + h), float(rng.integers(10, 245)), -1)
    return np.clip(base, 0, 255)


def project_tag(distance, yaw_deg, where):
    """The tag's black-square corners and its outer (quiet zone) corners, in edge coordinates, and
    its true pose."""
    half = TAG_M / 2
    margin = half * 10 / 8
    u, v = where
    # The centre's direction from the pixel it should land on.
    cx = (u * W - W / 2) / FOCAL * distance
    cy = (v * H - H / 2) / FOCAL * distance
    R, _ = cv2.Rodrigues(np.array([0.05, math.radians(yaw_deg), 0.1]))
    t = np.array([cx, cy, distance])

    def project(pts):
        cam = (R @ pts.T).T + t
        uv = (K @ cam.T).T
        return uv[:, :2] / uv[:, 2:3]

    square = np.array([[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]])
    outer = np.array([[-margin, -margin, 0], [margin, -margin, 0], [margin, margin, 0], [-margin, margin, 0]])
    # The tag is drawn in a frame with y down as printed, so its z points away from a camera that
    # sees it. Visor's tag frame has y up and z out of the face: the same turned half a turn about x.
    rvec, _ = cv2.Rodrigues(R @ np.diag([1.0, -1.0, -1.0]))
    pose = {"rvec": rvec.ravel().round(6).tolist(), "tvec": t.round(6).tolist()}
    return project(square), project(outer), pose


def render(rng, tags):
    frame = cv2.resize(background(rng), (W * SS, H * SS), interpolation=cv2.INTER_LINEAR)
    truth = []
    for tag_id, distance, yaw, where in tags:
        corners, outer, pose = project_tag(distance, yaw, where)
        img = tag_image(tag_id).astype(np.float32)
        size = img.shape[0]
        # warpPerspective works in pixel indices: shift by a half at both ends so the tag's edges land
        # exactly where the corners say.
        src = np.float32([[0, 0], [size, 0], [size, size], [0, size]]) - 0.5
        M = cv2.getPerspectiveTransform(src, np.float32(outer * SS - 0.5))
        big = cv2.warpPerspective(img, M, (W * SS, H * SS), flags=cv2.INTER_LINEAR)
        mask = cv2.warpPerspective(np.ones_like(img), M, (W * SS, H * SS), flags=cv2.INTER_LINEAR)
        black, white = 25.0, 210.0  # a tag under real light: black isn't 0 and white isn't 255
        shaded = black + (big / 255.0) * (white - black)
        frame = frame * (1 - mask) + shaded * mask
        # Edge coordinates (pixel (0, 0) spans 0 to 1) to pixel-centre coordinates.
        truth.append(
            {"id": tag_id, "distance_m": distance, "corners": (corners - 0.5).round(4).tolist(), "pose": pose}
        )
    frame = cv2.resize(frame, (W, H), interpolation=cv2.INTER_AREA)
    frame = frame + rng.normal(0, 1.5, frame.shape)
    return np.clip(frame, 0, 255).astype(np.uint8), truth


def main():
    rng = np.random.default_rng(1)
    OUT.mkdir(exist_ok=True)
    for name, tags in SCENES.items():
        image, truth = render(rng, tags)
        cv2.imwrite(str(OUT / f"{name}.png"), image, [cv2.IMWRITE_PNG_COMPRESSION, 9])
        (OUT / f"{name}.json").write_text(json.dumps({"width": W, "height": H, "tags": truth}, indent=2) + "\n")
        print(name, len(truth), "tags")


if __name__ == "__main__":
    main()
