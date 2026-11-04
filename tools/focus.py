# /// script
# requires-python = ">=3.12"
# dependencies = ["opencv-python==5.0.0.93", "numpy==2.5.4"]
# ///
"""Live view for focusing a camera's lens by hand, on a laptop with the camera plugged in.

    uv run tools/focus.py /dev/v4l/by-id/usb-...-video-index0 --width 1600 --height 1304

Point the camera at a tag or fine print at the distance to focus for (2 to 4 m for AprilTags), turn
the lens slowly, and watch the sharpness number: it peaks where the image is sharpest. Turn past
the peak and back, and stop at it. The window shows the whole frame, scaled to fit, beside the
middle of the frame at full size, where focus is judged. q quits, r resets the peak.

The sharpness is the variance of the Laplacian over the middle third of the frame: edges sharpen
as focus improves, so it rises and falls with focus for a scene that doesn't change. Its value only
compares one position of the lens against another, in the same light, at the same scene.
"""

import argparse
import sys

import cv2
import numpy as np


def sharpness(gray):
    """The Laplacian's variance over the middle third of the frame."""
    height, width = gray.shape
    middle = gray[height // 3 : 2 * height // 3, width // 3 : 2 * width // 3]
    return float(cv2.Laplacian(middle, cv2.CV_64F).var())


def gray_frame(frame):
    return frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def view(gray, score, peak, crop):
    """The frame scaled to 800 wide, beside the middle crop at full size, with the numbers."""
    height, width = gray.shape
    scale = 800 / width
    whole = cv2.resize(gray, (800, int(height * scale)), interpolation=cv2.INTER_AREA)
    top, left = (height - crop) // 2, (width - crop) // 2
    middle = gray[top : top + crop, left : left + crop]
    middle = cv2.resize(middle, (whole.shape[0], whole.shape[0]), interpolation=cv2.INTER_NEAREST)
    image = cv2.cvtColor(np.hstack([whole, middle]), cv2.COLOR_GRAY2BGR)
    x0, y0 = int(left * scale), int(top * scale)
    cv2.rectangle(image, (x0, y0), (int((left + crop) * scale), int((top + crop) * scale)), (0, 200, 255), 1)
    bar = int(600 * min(score / peak, 1.0)) if peak > 0 else 0
    cv2.rectangle(image, (10, 10), (10 + bar, 34), (0, 200, 0), -1)
    cv2.rectangle(image, (10, 10), (610, 34), (255, 255, 255), 1)
    text = f"sharpness {score:8.1f}   peak {peak:8.1f}   {width}x{height}   q quit, r reset"
    cv2.putText(image, text, (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
    return image


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("device", help="the camera's device, as `visor cameras` lists it")
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=1304)
    parser.add_argument("--crop", type=int, default=400, help="pixels of the middle shown at full size")
    parser.add_argument("--frames", type=int, default=0, help="stop after this many, with no window (a check)")
    args = parser.parse_args(argv)

    camera = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not camera.isOpened():
        sys.exit(f"Can't open {args.device}: is it plugged in, and not open in Visor or another program?")
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    peak, smoothed, count = 0.0, None, 0
    while True:
        ok, frame = camera.read()
        if not ok:
            sys.exit("The camera stopped sending frames.")
        gray = gray_frame(frame)
        score = sharpness(gray)
        # Smoothed over a few frames, so sensor noise doesn't make the number jump.
        smoothed = score if smoothed is None else 0.7 * smoothed + 0.3 * score
        peak = max(peak, smoothed)
        count += 1
        if args.frames:
            if count >= args.frames:
                height, width = gray.shape
                print(f"{width}x{height}, sharpness {smoothed:.1f}, peak {peak:.1f}")
                return
            continue
        cv2.imshow("focus", view(gray, smoothed, peak, min(args.crop, *gray.shape)))
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            return
        if key == ord("r"):
            peak = 0.0


if __name__ == "__main__":
    main()
