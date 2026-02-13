#!/usr/bin/env python3
#
# Generate stereo_extrinsics.yaml (rotation R and translation T from left to right)
# from existing left.yaml and right.yaml when those files don't include R and T.
#
# For rectified stereo, translation is derived from the right camera's projection
# matrix: baseline = -P_right[0,3] / P_right[0,0], T = (baseline, 0, 0), R = I.
# Use: python3 generate_stereo_extrinsics.py left.yaml right.yaml [--output stereo_extrinsics.yaml]

import argparse
import re
import sys


def load_projection_matrix(path):
    """Load 3x4 projection matrix from a ROS-format calibration YAML."""
    with open(path) as f:
        text = f.read()
    # Find "projection_matrix:" section and collect "data:" block (may span lines)
    in_projection = False
    data_lines = []
    for line in text.splitlines():
        if line.strip().startswith("projection_matrix:"):
            in_projection = True
            continue
        if in_projection:
            if "data:" in line or (data_lines and re.search(r"[\d\.\-\+]", line)):
                data_lines.append(line)
                raw = " ".join(data_lines)
                numbers = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", raw)]
                if len(numbers) >= 12:
                    break
            elif line.strip() and not line.strip().startswith("rows") and not line.strip().startswith("cols"):
                in_projection = False
                data_lines = []
    if not data_lines:
        raise SystemExit("Could not find projection_matrix in %s" % path)
    raw = " ".join(data_lines)
    numbers = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", raw)]
    if len(numbers) < 12:
        raise SystemExit("projection_matrix in %s does not have 12 values (got %d)" % (path, len(numbers)))
    P = [numbers[0:4], numbers[4:8], numbers[8:12]]
    return P


def main():
    parser = argparse.ArgumentParser(
        description="Generate stereo_extrinsics.yaml from left.yaml and right.yaml"
    )
    parser.add_argument("left_yaml", help="Path to left camera YAML")
    parser.add_argument("right_yaml", help="Path to right camera YAML")
    parser.add_argument(
        "--output", "-o", default="stereo_extrinsics.yaml",
        help="Output path for stereo extrinsics (default: stereo_extrinsics.yaml)",
    )
    args = parser.parse_args()

    P_right = load_projection_matrix(args.right_yaml)
    fx = P_right[0][0]
    tx = P_right[0][3]
    # In rectified stereo: P_right has last column = [ -fx*baseline, 0, 0 ]
    baseline = -tx / fx if fx != 0 else 0.0

    # R = identity, T = (baseline, 0, 0) in left camera frame
    R = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
    T = [[baseline], [0.0], [0.0]]

    def format_mat(mat, precision=8):
        # Flatten to list of numbers (row-major)
        if isinstance(mat[0], (list, tuple)):
            flat = [x for row in mat for x in (row if isinstance(row, (list, tuple)) else [row])]
        else:
            flat = list(mat)
        return "[%s]" % ", ".join("%.*f" % (precision, x) for x in flat)

    yaml_content = """# Stereo extrinsics: rotation (R) and translation (T) from left to right camera
# Generated from right camera projection matrix (baseline = -Tx/fx)
rotation_matrix:
  rows: 3
  cols: 3
  data: %s
translation:
  rows: 3
  cols: 1
  data: %s
""" % (format_mat(R), format_mat(T))

    with open(args.output, "w") as f:
        f.write(yaml_content)
    print("Wrote %s (baseline = %.6f)" % (args.output, baseline))
    return 0


if __name__ == "__main__":
    sys.exit(main())
