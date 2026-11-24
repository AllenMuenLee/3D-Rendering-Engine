"""Generate triangle LUT (shape_lut.hex) for the 5 Platonic solids + NVIDIA logo.

Each entry packs 3 vertices (v0, v1, v2) and 1 outward unit face normal (n)
as 12 signed 9-bit Q2.7 numbers (1.0 = 128) -> 108 bits = 27 hex digits.
Every shape occupies MAX_TRIS (256) consecutive slots in shape_lut.hex.
"""

import json
import math
from pathlib import Path

MAX_TRIS = 256
FRAC_BITS = 7
SCALE = 1 << FRAC_BITS  # 128 -> Q2.7 (signed 9-bit: -256 .. +255)

SHAPE_ORDER = [
    "tetrahedron",
    "octahedron",
    "cube",
    "icosahedron",
    "dodecahedron",
    "nvidia_logo",
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


def to_q2_7(val):
    q = int(round(val * SCALE))
    q = max(-256, min(255, q))
    return q & 0x1FF  # 9-bit two's complement


def pack_tri(v0, v1, v2, n):
    word = 0
    for vec in (v0, v1, v2, n):
        for val in vec:
            word = (word << 9) | to_q2_7(val)
    return "{:027X}".format(word)


def main():
    root = Path(__file__).resolve().parent
    data = json.loads((root / "platonic_solids.json").read_text(encoding="utf-8"))
    solids = data["solids"]

    zero_line = "0" * 27
    lines = []
    for shape_id, name in enumerate(SHAPE_ORDER):
        solid = solids[name]
        verts = solid["vertices"]
        tris = solid["triangles"]
        assert len(tris) <= MAX_TRIS

        shape_lines = []
        for i0, i1, i2 in tris:
            v0, v1, v2 = verts[i0], verts[i1], verts[i2]
            c = cross(sub(v1, v0), sub(v2, v0))
            norm = length(c)
            n = (c[0] / norm, c[1] / norm, c[2] / norm) if norm > 0.0 else (0.0, 0.0, 0.0)
            shape_lines.append(pack_tri(v0, v1, v2, n))

        while len(shape_lines) < MAX_TRIS:
            shape_lines.append(zero_line)

        lines.extend(shape_lines)
        print("Shape {:d} ({:<12s}): {:3d} triangles".format(shape_id, name, len(tris)))

    out_path = root / "shape_lut.hex"
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("Wrote {} ({} total entries)".format(out_path.name, len(lines)))


if __name__ == "__main__":
    main()
