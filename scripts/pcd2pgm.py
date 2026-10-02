#!/usr/bin/env python3

import argparse
import math
import sys
from pathlib import Path

import numpy as np


PCD_TYPES = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("I", 1): "<i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
    ("I", 8): "<i8",
    ("U", 1): "<u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
    ("U", 8): "<u8",
}


def read_pcd_header(path):
    metadata = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header has no DATA line")
            try:
                text = line.decode("ascii").strip()
            except UnicodeDecodeError as error:
                raise ValueError("PCD header is not valid ASCII") from error
            if not text or text.startswith("#"):
                continue
            key, *values = text.split()
            metadata[key.upper()] = values
            if key.upper() == "DATA":
                return metadata, stream.tell()


def make_pcd_dtype(metadata):
    fields = metadata.get("FIELDS")
    sizes = metadata.get("SIZE")
    types = metadata.get("TYPE")
    counts = metadata.get("COUNT", ["1"] * len(fields or []))
    if not fields or not sizes or not types:
        raise ValueError("PCD header must define FIELDS, SIZE, and TYPE")
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("PCD FIELDS, SIZE, TYPE, and COUNT lengths do not match")

    dtype_fields = []
    for field, size_text, type_name, count_text in zip(fields, sizes, types, counts):
        size = int(size_text)
        count = int(count_text)
        dtype_name = PCD_TYPES.get((type_name.upper(), size))
        if dtype_name is None:
            raise ValueError(f"Unsupported PCD field type: {type_name}{size}")
        if count == 1:
            dtype_fields.append((field, dtype_name))
        else:
            dtype_fields.append((field, dtype_name, (count,)))
    return np.dtype(dtype_fields)


def load_xyz(path):
    metadata, data_offset = read_pcd_header(path)
    fields = metadata.get("FIELDS", [])
    missing = {"x", "y", "z"} - set(fields)
    if missing:
        raise ValueError(f"PCD is missing required fields: {', '.join(sorted(missing))}")

    data_format = metadata["DATA"][0].lower()
    point_count = int(metadata.get("POINTS", metadata.get("WIDTH", ["0"]))[0])
    if point_count <= 0:
        raise ValueError("PCD contains no points")

    if data_format == "binary":
        cloud = np.fromfile(
            path, dtype=make_pcd_dtype(metadata), count=point_count, offset=data_offset
        )
        if cloud.size != point_count:
            raise ValueError(
                f"PCD declares {point_count} points but contains {cloud.size}"
            )
        return np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(
            np.float64, copy=False
        )

    if data_format == "ascii":
        column_offsets = {}
        offset = 0
        counts = [int(value) for value in metadata.get("COUNT", ["1"] * len(fields))]
        for field, count in zip(fields, counts):
            column_offsets[field] = offset
            offset += count
        columns = [column_offsets[axis] for axis in ("x", "y", "z")]
        with path.open("rb") as stream:
            stream.seek(data_offset)
            cloud = np.loadtxt(stream, usecols=columns)
        if cloud.ndim == 1:
            cloud = cloud.reshape(1, -1)
        return cloud

    if data_format == "binary_compressed":
        raise ValueError("binary_compressed PCD is not supported; use binary or ASCII PCD")
    raise ValueError(f"Unsupported PCD DATA format: {data_format}")


def rasterize(points, resolution, z_min, z_max, padding, min_points, inflation):
    finite = np.isfinite(points).all(axis=1)
    selected = points[finite & (points[:, 2] >= z_min) & (points[:, 2] <= z_max)]
    if selected.size == 0:
        raise ValueError(f"No finite points remain in Z range [{z_min}, {z_max}]")

    x_min = math.floor((selected[:, 0].min() - padding) / resolution) * resolution
    y_min = math.floor((selected[:, 1].min() - padding) / resolution) * resolution
    x_max = math.ceil((selected[:, 0].max() + padding) / resolution) * resolution
    y_max = math.ceil((selected[:, 1].max() + padding) / resolution) * resolution
    width = max(1, int(round((x_max - x_min) / resolution)) + 1)
    height = max(1, int(round((y_max - y_min) / resolution)) + 1)

    columns = np.floor((selected[:, 0] - x_min) / resolution).astype(np.int64)
    rows = np.floor((selected[:, 1] - y_min) / resolution).astype(np.int64)
    columns = np.clip(columns, 0, width - 1)
    rows = np.clip(rows, 0, height - 1)

    counts = np.zeros((height, width), dtype=np.uint32)
    np.add.at(counts, (rows, columns), 1)
    occupied = counts >= min_points
    if inflation > 0:
        occupied = inflate_obstacles(occupied, math.ceil(inflation / resolution))

    return occupied, (x_min, y_min), selected.shape[0]


def inflate_obstacles(occupied, radius_cells):
    source = occupied
    inflated = occupied.copy()
    for row_offset in range(-radius_cells, radius_cells + 1):
        for column_offset in range(-radius_cells, radius_cells + 1):
            if row_offset**2 + column_offset**2 > radius_cells**2:
                continue
            source_rows = slice(
                max(0, -row_offset), min(source.shape[0], source.shape[0] - row_offset)
            )
            source_columns = slice(
                max(0, -column_offset),
                min(source.shape[1], source.shape[1] - column_offset),
            )
            target_rows = slice(
                max(0, row_offset), min(source.shape[0], source.shape[0] + row_offset)
            )
            target_columns = slice(
                max(0, column_offset),
                min(source.shape[1], source.shape[1] + column_offset),
            )
            inflated[target_rows, target_columns] |= source[
                source_rows, source_columns
            ]
    return inflated


def write_map(output_prefix, occupied, origin, resolution, background):
    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")
    pgm_path.parent.mkdir(parents=True, exist_ok=True)

    background_value = 254 if background == "free" else 205
    image = np.full(occupied.shape, background_value, dtype=np.uint8)
    image[occupied] = 0
    image = np.flipud(image)

    with pgm_path.open("wb") as stream:
        stream.write(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii"))
        stream.write(image.tobytes())

    yaml_path.write_text(
        "\n".join(
            [
                f"image: {pgm_path.name}",
                "mode: trinary",
                f"resolution: {resolution:.9g}",
                f"origin: [{origin[0]:.9g}, {origin[1]:.9g}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return pgm_path, yaml_path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert a PCD point cloud into a Nav2 PGM/YAML occupancy map.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        default=Path("maps/office/map.pcd"),
        help="path to the input PCD point-cloud file",
    )
    parser.add_argument(
        "output",
        nargs="?",
        type=Path,
        default=Path("maps/office/map_2d"),
        help="output path prefix; .pgm and .yaml are added automatically",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.05,
        help="width and height of each occupancy-grid cell in meters",
    )
    parser.add_argument(
        "--z-min",
        type=float,
        default=0.1,
        help="minimum point height in meters to project as an obstacle",
    )
    parser.add_argument(
        "--z-max",
        type=float,
        default=2.0,
        help="maximum point height in meters to project as an obstacle",
    )
    parser.add_argument(
        "--padding",
        type=float,
        default=0.5,
        help="empty margin in meters added around the point-cloud bounds",
    )
    parser.add_argument(
        "--min-points",
        type=int,
        default=1,
        help="minimum number of projected points required to occupy a cell",
    )
    parser.add_argument(
        "--inflation",
        type=float,
        default=0.0,
        help="radius in meters used to expand occupied cells; normally handled by Nav2",
    )
    parser.add_argument(
        "--background",
        choices=("free", "unknown"),
        default="free",
        help="occupancy state assigned to cells without projected points",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.resolution <= 0:
        raise ValueError("--resolution must be positive")
    if args.z_min > args.z_max:
        raise ValueError("--z-min must not exceed --z-max")
    if args.padding < 0 or args.inflation < 0:
        raise ValueError("--padding and --inflation must not be negative")
    if args.min_points < 1:
        raise ValueError("--min-points must be at least 1")
    if not args.input.is_file():
        raise ValueError(f"Input PCD does not exist: {args.input}")

    points = load_xyz(args.input)
    occupied, origin, selected_count = rasterize(
        points,
        args.resolution,
        args.z_min,
        args.z_max,
        args.padding,
        args.min_points,
        args.inflation,
    )
    pgm_path, yaml_path = write_map(
        args.output, occupied, origin, args.resolution, args.background
    )
    print(
        f"Converted {selected_count}/{points.shape[0]} points into "
        f"{occupied.shape[1]}x{occupied.shape[0]} cells "
        f"({int(occupied.sum())} occupied)"
    )
    print(f"PGM:  {pgm_path}")
    print(f"YAML: {yaml_path}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as error:
        print(f"pcd2pgm: {error}", file=sys.stderr)
        sys.exit(1)
