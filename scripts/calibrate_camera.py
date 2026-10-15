"""Calibrate a camera from chessboard images -> byeol YAML/JSON profile (requires OpenCV: pip install opencv-python-headless).

Example: python scripts/calibrate_camera.py --images "calib/*.jpg" --cols 9 --rows 6 --square-mm 30 --kind rgb \\
             --name my_cam --out configs/cameras/my_cam.yaml
cols/rows are the number of INNER corners (not squares). Capture 15-25 frames at different tilts and positions; the board
should cover different parts of the frame, including the edges. Warning signs: rms > 1 px or fewer than 10 frames used.
The profile is written in pixels mode (fx, fy, cx, cy + distortion) for the image resolution; for a different resolution
use CameraModel.resized().
"""
import argparse
import glob
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from byeol.calibrate import CalibrationError, calibrate_images
from byeol.camprofile import dump_camera


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", required=True, help="glob pattern for chessboard images")
    ap.add_argument("--cols", type=int, required=True)
    ap.add_argument("--rows", type=int, required=True)
    ap.add_argument("--square-mm", type=float, required=True)
    ap.add_argument("--kind", choices=("rgb", "lwir"), default="rgb")
    ap.add_argument("--name", default="calibrated")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    try:
        import cv2
    except ImportError:
        print("OpenCV is required: pip install opencv-python-headless", file=sys.stderr)
        return 2
    files = sorted(glob.glob(a.images))
    if not files:
        print(f"no files match pattern {a.images!r}", file=sys.stderr)
        return 2
    imgs = [cv2.imread(f, cv2.IMREAD_GRAYSCALE) for f in files]
    imgs = [i for i in imgs if i is not None]
    try:
        m, rms, used = calibrate_images(imgs, (a.cols, a.rows), a.square_mm / 1000.0, a.name)
    except CalibrationError as e:
        print(f"calibration error: {e}", file=sys.stderr)
        return 1
    from dataclasses import replace
    m = replace(m, kind=a.kind, notes=f"chessboard calibration {a.cols}x{a.rows}, frames {used}/{len(imgs)}, rms {rms:.3f} px")
    dump_camera(m, a.out, extra={"source": "own calibration", "license": "user-provided"})
    print(f"frames {used}/{len(imgs)}, reprojection rms {rms:.3f} px\nfx={m.fx:.2f} fy={m.fy:.2f} cx={m.cx:.2f} cy={m.cy:.2f}\n"
          f"k1={m.dist.k1:.4f} k2={m.dist.k2:.4f} p1={m.dist.p1:.5f} p2={m.dist.p2:.5f} k3={m.dist.k3:.4f}\nwritten {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
