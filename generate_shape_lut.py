"""Generate surface-point LUT (shape_lut.hex) for all 10 3D shapes.

Reads the triangulated 3D meshes from platonic_solids.json, samples NUM_POINTS
evenly across each shape's surface (proportional to triangle area), and writes
each point (px, py, pz) and outward unit normal (nx, ny, nz) in signed 9-bit
Q2.7 fixed-point (1.0 = 128) packed as a 54-bit hex word per line.
"""

import json
import math
from pathlib import Path

NUM_POINTS = 2048
FRAC_BITS = 7
SCALE = 1 << FRAC_BITS  # 128 -> Q2.7 (signed 9-bit: -256 .. +255)

SHAPE_ORDER = [
    "tetrahedron",
    "octahedron",
    "cube",
    "icosahedron",
    "dodecahedron",
    "person",
    "nvidia_logo",
    "rocket",
    "jet",
    "star",
]


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def length(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def sample_triangle(v0, v1, v2, count):
    """Deterministically sample `count` evenly spaced points inside a triangle."""
    if count <= 0:
        return []
    if count == 1:
        return [(
            (v0[0] + v1[0] + v2[0]) / 3.0,
            (v0[1] + v1[1] + v2[1]) / 3.0,
            (v0[2] + v1[2] + v2[2]) / 3.0,
        )]

    # Find grid resolution n such that n*(n+1)/2 >= count
    n = 1
    while (n * (n + 1)) // 2 < count:
        n += 1

    grid = []
    for i in range(n):
        for j in range(n - i):
            # Cell-centered barycentric coordinates inside the triangle
            u = (i + 1.0 / 3.0) / n
            v = (j + 1.0 / 3.0) / n
            w = 1.0 - u - v
            p = (
                w * v0[0] + u * v1[0] + v * v2[0],
                w * v0[1] + u * v1[1] + v * v2[1],
                w * v0[2] + u * v1[2] + v * v2[2],
            )
            grid.append(p)

    # Pick exactly `count` evenly spread entries from the grid
    if len(grid) == count:
        return grid
    step = len(grid) / float(count)
    return [grid[int(k * step)] for k in range(count)]


def sample_shape(solid, num_points):
    verts = solid["vertices"]
    tris = solid["triangles"]

    # Compute each triangle's area and outward unit normal
    tri_info = []
    total_area = 0.0
    for i0, i1, i2 in tris:
        v0, v1, v2 = verts[i0], verts[i1], verts[i2]
        c = cross(sub(v1, v0), sub(v2, v0))
        norm = length(c)
        if norm == 0.0:
            continue
        area = 0.5 * norm
        normal = (c[0] / norm, c[1] / norm, c[2] / norm)
        tri_info.append((v0, v1, v2, normal, area))
        total_area += area

    # Allocate point counts proportional to triangle area (largest remainder)
    exact = [num_points * (t[4] / total_area) for t in tri_info]
    counts = [int(math.floor(x)) for x in exact]
    remainder = num_points - sum(counts)
    order = sorted(range(len(tri_info)), key=lambda idx: exact[idx] - counts[idx], reverse=True)
    for idx in order[:remainder]:
        counts[idx] += 1

    samples = []
    for (v0, v1, v2, normal, _), cnt in zip(tri_info, counts):
        for p in sample_triangle(v0, v1, v2, cnt):
            samples.append((p, normal))

    assert len(samples) == num_points
    return samples


def to_q2_7(val):
    q = int(round(val * SCALE))
    q = max(-256, min(255, q))
    return q & 0x1FF  # 9-bit two's complement


def pack_entry(p, n):
    word = 0
    for val in (p[0], p[1], p[2], n[0], n[1], n[2]):
        word = (word << 9) | to_q2_7(val)
    return "{:014X}".format(word)


def main():
    root = Path(__file__).resolve().parent
    data = json.loads((root / "platonic_solids.json").read_text(encoding="utf-8"))
    solids = data["solids"]

    lines = []
    for shape_id, name in enumerate(SHAPE_ORDER):
        solid = solids[name]
        assert solid["shape_id"] == shape_id
        samples = sample_shape(solid, NUM_POINTS)
        for p, n in samples:
            lines.append(pack_entry(p, n))
        print("Shape {:d} ({:<12s}): {:d} points".format(shape_id, name, len(samples)))

    out_path = root / "shape_lut.hex"
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("Wrote {} ({} total entries)".format(out_path.name, len(lines)))


if __name__ == "__main__":
    main()
