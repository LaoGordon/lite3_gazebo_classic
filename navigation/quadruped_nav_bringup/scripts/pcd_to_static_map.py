#!/usr/bin/env python3
"""Convert a PCD point cloud into a 2D static occupancy map for Nav2.

The script intentionally keeps the first version simple:
- read PCD in ascii or binary form
- keep only xyz fields
- crop by height
- project to the XY plane
- emit ROS map_server compatible .pgm + .yaml
"""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np


PGM_FREE = 254
PGM_OCCUPIED = 0


class PCDFormatError(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd_path", type=Path, help="Input .pcd file path")
    parser.add_argument(
        "--output-prefix",
        type=Path,
        required=True,
        help="Output prefix without extension, e.g. maps/floor_1",
    )
    parser.add_argument("--z-min", type=float, default=-0.20, help="Minimum obstacle height")
    parser.add_argument("--z-max", type=float, default=0.60, help="Maximum obstacle height")
    parser.add_argument("--resolution", type=float, default=0.05, help="Map resolution in meters")
    parser.add_argument(
        "--padding",
        type=float,
        default=1.0,
        help="Extra padding around detected point bounds in meters",
    )
    parser.add_argument(
        "--min-points-per-cell",
        type=int,
        default=1,
        help="Minimum projected point count per cell to mark occupied",
    )
    parser.add_argument(
        "--frame-id",
        default="camera_init",
        help="Frame note written into the companion metadata file",
    )
    return parser.parse_args()


def read_pcd(path: Path) -> np.ndarray:
    header: Dict[str, str] = {}
    header_lines: List[str] = []
    with path.open("rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise PCDFormatError("Unexpected EOF before DATA line")
            decoded = line.decode("utf-8", errors="strict").strip()
            header_lines.append(decoded)
            if decoded.startswith("#") or not decoded:
                continue
            key, *rest = decoded.split()
            header[key.upper()] = " ".join(rest)
            if key.upper() == "DATA":
                data_type = " ".join(rest).lower()
                break
        data = f.read()

    fields = header.get("FIELDS", "").split()
    size = [int(v) for v in header.get("SIZE", "").split()]
    typ = header.get("TYPE", "").split()
    count = [int(v) for v in header.get("COUNT", "").split()] or [1] * len(fields)
    width = int(header.get("WIDTH", "0"))
    height = int(header.get("HEIGHT", "1"))
    points = int(header.get("POINTS", str(width * height)))

    if not fields or not size or not typ:
        raise PCDFormatError("Incomplete PCD header")
    if not (len(fields) == len(size) == len(typ) == len(count)):
        raise PCDFormatError("FIELDS/SIZE/TYPE/COUNT length mismatch")
    if any(c != 1 for c in count):
        raise PCDFormatError("COUNT > 1 is not supported in this first version")

    dtype_fields = []
    for field, s, t in zip(fields, size, typ):
        if t == "F" and s == 4:
            dtype_fields.append((field, np.float32))
        elif t == "F" and s == 8:
            dtype_fields.append((field, np.float64))
        elif t == "U" and s == 1:
            dtype_fields.append((field, np.uint8))
        elif t == "U" and s == 2:
            dtype_fields.append((field, np.uint16))
        elif t == "U" and s == 4:
            dtype_fields.append((field, np.uint32))
        elif t == "I" and s == 1:
            dtype_fields.append((field, np.int8))
        elif t == "I" and s == 2:
            dtype_fields.append((field, np.int16))
        elif t == "I" and s == 4:
            dtype_fields.append((field, np.int32))
        else:
            raise PCDFormatError(f"Unsupported field format: {field} size={s} type={t}")
    dtype = np.dtype(dtype_fields)

    if data_type == "ascii":
        rows = []
        text = data.decode("utf-8", errors="strict").strip().splitlines()
        for row in text:
            if not row.strip():
                continue
            parts = row.split()
            if len(parts) != len(fields):
                raise PCDFormatError("ASCII row width mismatch")
            rows.append(tuple(_cast_ascii_value(v, dt[1]) for v, dt in zip(parts, dtype_fields)))
        array = np.array(rows, dtype=dtype)
    elif data_type == "binary":
        expected = points * dtype.itemsize
        if len(data) < expected:
            raise PCDFormatError(f"Binary payload too short: {len(data)} < {expected}")
        array = np.frombuffer(data[:expected], dtype=dtype, count=points)
    else:
        raise PCDFormatError(f"Unsupported DATA mode: {data_type}")

    for axis in ("x", "y", "z"):
        if axis not in array.dtype.names:
            raise PCDFormatError(f"Missing required field '{axis}'")

    xyz = np.column_stack((array["x"], array["y"], array["z"]))
    return xyz.astype(np.float64, copy=False)


def _cast_ascii_value(value: str, dtype: np.dtype) -> object:
    if np.issubdtype(dtype, np.floating):
        return float(value)
    if np.issubdtype(dtype, np.integer):
        return int(value)
    raise PCDFormatError(f"Unsupported ASCII dtype cast: {dtype}")


def project_to_grid(
    xyz: np.ndarray,
    z_min: float,
    z_max: float,
    resolution: float,
    padding: float,
    min_points_per_cell: int,
) -> Tuple[np.ndarray, float, float]:
    finite_mask = np.isfinite(xyz).all(axis=1)
    z_mask = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
    filtered = xyz[finite_mask & z_mask]
    if filtered.size == 0:
        raise RuntimeError("No points remain after finite and height filtering")

    min_x = math.floor((filtered[:, 0].min() - padding) / resolution) * resolution
    min_y = math.floor((filtered[:, 1].min() - padding) / resolution) * resolution
    max_x = math.ceil((filtered[:, 0].max() + padding) / resolution) * resolution
    max_y = math.ceil((filtered[:, 1].max() + padding) / resolution) * resolution

    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))
    if width <= 0 or height <= 0:
        raise RuntimeError("Computed map dimensions are invalid")

    counts = np.zeros((height, width), dtype=np.uint16)
    ix = np.floor((filtered[:, 0] - min_x) / resolution).astype(np.int32)
    iy = np.floor((filtered[:, 1] - min_y) / resolution).astype(np.int32)

    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    ix = ix[valid]
    iy = iy[valid]
    np.add.at(counts, (iy, ix), 1)

    occupied = counts >= max(1, min_points_per_cell)
    image = np.full((height, width), PGM_FREE, dtype=np.uint8)
    image[occupied] = PGM_OCCUPIED
    # ROS map images treat the top row as max-y, so flip vertically before saving.
    image = np.flipud(image)
    return image, min_x, min_y


def write_pgm(path: Path, image: np.ndarray) -> None:
    height, width = image.shape
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    with path.open("wb") as f:
        f.write(header)
        f.write(image.tobytes())


def write_yaml(path: Path, image_path: Path, resolution: float, origin_x: float, origin_y: float) -> None:
    text = (
        f"image: {image_path.name}\n"
        f"mode: trinary\n"
        f"resolution: {resolution:.6f}\n"
        f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    path.write_text(text)


def write_metadata(path: Path, pcd_path: Path, frame_id: str, z_min: float, z_max: float, resolution: float, padding: float) -> None:
    text = (
        f"source_pcd: {pcd_path}\n"
        f"source_frame: {frame_id}\n"
        f"z_min: {z_min}\n"
        f"z_max: {z_max}\n"
        f"resolution: {resolution}\n"
        f"padding: {padding}\n"
    )
    path.write_text(text)


def main() -> None:
    args = parse_args()
    if args.resolution <= 0.0:
        raise SystemExit("--resolution must be positive")
    if args.z_min >= args.z_max:
        raise SystemExit("--z-min must be smaller than --z-max")

    xyz = read_pcd(args.pcd_path)
    image, origin_x, origin_y = project_to_grid(
        xyz,
        z_min=args.z_min,
        z_max=args.z_max,
        resolution=args.resolution,
        padding=args.padding,
        min_points_per_cell=args.min_points_per_cell,
    )

    output_prefix = args.output_prefix
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")
    meta_path = output_prefix.with_name(output_prefix.name + "_metadata.txt")

    write_pgm(pgm_path, image)
    write_yaml(yaml_path, pgm_path, args.resolution, origin_x, origin_y)
    write_metadata(meta_path, args.pcd_path, args.frame_id, args.z_min, args.z_max, args.resolution, args.padding)

    occupied_cells = int(np.count_nonzero(image == PGM_OCCUPIED))
    print(f"Saved map image: {pgm_path}")
    print(f"Saved map yaml:  {yaml_path}")
    print(f"Saved metadata:  {meta_path}")
    print(f"Map size: {image.shape[1]} x {image.shape[0]} cells")
    print(f"Occupied cells: {occupied_cells}")
    print(f"Origin: ({origin_x:.3f}, {origin_y:.3f})")


if __name__ == "__main__":
    main()
