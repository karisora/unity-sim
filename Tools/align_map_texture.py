#!/usr/bin/env python3
"""Add orthophoto UVs to Model3D_mesh2 and trim its invalid perimeter.

The affine transform was measured by registering a top-down rendering of the
OBJ vertex colors against orthophoto.tif.  The fit used 461 RANSAC inliers and
has a 1.7-pixel median residual at the source texture resolution.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter


# OBJ XY -> registration-render pixels used during feature matching.
OBJ_X_MIN = -18.5087376
OBJ_Y_MAX = 35.9261818
RENDER_PIXELS_PER_UNIT = 30.0

# Registration-render pixels -> orthophoto pixels at REGISTRATION_SCALE.
REGISTERED_AFFINE = np.array(
    [
        [1.01063207, 0.000628182187, 3.33727909],
        [-0.00147350739, 1.01076606, -12.8932475],
    ],
    dtype=np.float64,
)
REGISTRATION_SCALE = 1123.0 / 3703.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path, help="Untextured source OBJ")
    parser.add_argument("texture", type=Path, help="Aligned RGBA orthophoto")
    parser.add_argument("output", type=Path, help="Textured output OBJ")
    return parser.parse_args()


def texture_pixel(x: float, y: float) -> tuple[float, float]:
    render_pixel = np.array(
        [
            (x - OBJ_X_MIN) * RENDER_PIXELS_PER_UNIT,
            (OBJ_Y_MAX - y) * RENDER_PIXELS_PER_UNIT,
            1.0,
        ],
        dtype=np.float64,
    )
    pixel_x, pixel_y = REGISTERED_AFFINE @ render_pixel
    return pixel_x / REGISTRATION_SCALE, pixel_y / REGISTRATION_SCALE


def read_uvs_and_validity(
    source: Path, texture: Path
) -> tuple[list[tuple[float, float]], list[bool], int, int]:
    with Image.open(texture) as image:
        width, height = image.size
        if image.mode == "RGBA":
            alpha = image.getchannel("A")
        else:
            alpha = Image.new("L", image.size, 255)

        # Keep triangle edges two pixels inside the valid orthomosaic footprint.
        valid_alpha = np.asarray(alpha.filter(ImageFilter.MinFilter(5))) > 0

    uvs: list[tuple[float, float]] = []
    valid: list[bool] = [False]  # OBJ indices are one-based.
    with source.open("r", encoding="ascii") as stream:
        for line in stream:
            if not line.startswith("v "):
                continue
            fields = line.split()
            pixel_x, pixel_y = texture_pixel(float(fields[1]), float(fields[2]))
            u = pixel_x / (width - 1)
            v = 1.0 - pixel_y / (height - 1)
            uvs.append((u, v))

            image_x = round(pixel_x)
            image_y = round(pixel_y)
            inside = 0 <= image_x < width and 0 <= image_y < height
            valid.append(bool(valid_alpha[image_y, image_x]) if inside else False)

    total_faces = 0
    kept_faces = 0
    with source.open("r", encoding="ascii") as stream:
        for line in stream:
            if not line.startswith("f "):
                continue
            total_faces += 1
            vertex_ids = [int(item.split("/")[0]) for item in line.split()[1:]]
            kept_faces += all(valid[vertex_id] for vertex_id in vertex_ids)

    return uvs, valid, total_faces, kept_faces


def rewrite_obj(
    source: Path,
    output: Path,
    uvs: list[tuple[float, float]],
    valid: list[bool],
    total_faces: int,
    kept_faces: int,
) -> None:
    inserted_uvs = False
    with source.open("r", encoding="ascii") as input_stream, output.open(
        "w", encoding="ascii", newline="\n"
    ) as output_stream:
        for line in input_stream:
            stripped = line.rstrip("\r\n")
            if stripped.startswith("# Faces:"):
                output_stream.write(f"# Faces: {kept_faces}\n")
                continue
            if stripped == f"# {total_faces} faces, 0 coords texture":
                output_stream.write(
                    f"# {kept_faces} faces, {len(uvs)} coords texture\n"
                )
                continue
            if stripped.startswith("f "):
                if not inserted_uvs:
                    output_stream.write("# Texture coordinates generated from orthophoto.tif\n")
                    for u, v in uvs:
                        output_stream.write(f"vt {u:.9f} {v:.9f}\n")
                    output_stream.write("# Faces cropped to the valid orthophoto footprint\n")
                    inserted_uvs = True

                tokens = stripped.split()[1:]
                parsed = [token.split("/") for token in tokens]
                vertex_ids = [int(fields[0]) for fields in parsed]
                if not all(valid[vertex_id] for vertex_id in vertex_ids):
                    continue

                textured = []
                for fields in parsed:
                    vertex_id = fields[0]
                    normal_id = fields[-1] if len(fields) > 1 and fields[-1] else vertex_id
                    textured.append(f"{vertex_id}/{vertex_id}/{normal_id}")
                output_stream.write("f " + " ".join(textured) + "\n")
                continue

            output_stream.write(stripped + "\n")


def main() -> None:
    args = parse_args()
    uvs, valid, total_faces, kept_faces = read_uvs_and_validity(
        args.source, args.texture
    )
    rewrite_obj(
        args.source,
        args.output,
        uvs,
        valid,
        total_faces,
        kept_faces,
    )
    print(f"vertices/UVs: {len(uvs)}")
    print(f"faces: {total_faces} -> {kept_faces} ({total_faces - kept_faces} trimmed)")
    print(f"wrote: {args.output}")


if __name__ == "__main__":
    main()
