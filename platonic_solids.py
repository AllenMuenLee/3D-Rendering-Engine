"""Calculate vertices, polygon faces, and triangle face groups for all 5 Platonic solids.

Provides:
1. A clean Python API (`SOLIDS`, `build_all_solids`, `get_solid`, `PlatonicSolid`)
   that can be imported directly into a 3D rendering engine.
2. Geometric & topological verification (`verify_solid`, `verify_all_solids`).
3. Exporters for hardcoded Verilog LUTs (`generate_verilog_lut`), JSON, and Wavefront OBJ.

All solids are centered at the origin (0, 0, 0), normalized to the unit sphere
(||v|| = 1.0), and oriented with counter-clockwise (CCW) outward-facing winding.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


Vec3 = tuple[float, float, float]
Tri = tuple[int, int, int]
Face = tuple[int, ...]

SHAPE_ORDER: tuple[str, ...] = (
    "tetrahedron",
    "octahedron",
    "cube",
    "icosahedron",
    "dodecahedron",
)


# ---------------------------------------------------------------------------
# Vector math helpers
# ---------------------------------------------------------------------------

def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a: Vec3) -> float:
    return math.sqrt(_dot(a, a))


def _normalize(a: Vec3) -> Vec3:
    length = _norm(a)
    if length == 0.0:
        raise ValueError("Cannot normalize zero vector")
    return (a[0] / length, a[1] / length, a[2] / length)


def _dist(a: Vec3, b: Vec3) -> float:
    return _norm(_sub(a, b))


def _triangle_normal(v0: Vec3, v1: Vec3, v2: Vec3) -> Vec3:
    return _normalize(_cross(_sub(v1, v0), _sub(v2, v0)))


def _face_centroid(vertices: list[Vec3], face: Iterable[int]) -> Vec3:
    pts = [vertices[i] for i in face]
    n = len(pts)
    return (
        sum(p[0] for p in pts) / n,
        sum(p[1] for p in pts) / n,
        sum(p[2] for p in pts) / n,
    )


def _orient_ccw_outward(vertices: list[Vec3], face: list[int]) -> Face:
    """Ensure polygon vertices are ordered CCW when viewed from outside the origin."""
    centroid = _face_centroid(vertices, face)
    v0, v1, v2 = vertices[face[0]], vertices[face[1]], vertices[face[2]]
    normal = _cross(_sub(v1, v0), _sub(v2, v0))
    if _dot(normal, centroid) < 0.0:
        face = [face[0], *reversed(face[1:])]
    return tuple(face)


def _sort_coplanar_ccw(vertices: list[Vec3], indices: list[int], outward_hint: Vec3) -> Face:
    """Order coplanar polygon vertices CCW around `outward_hint`."""
    n = _normalize(outward_hint)
    # Pick a stable reference axis not parallel to n
    ref = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = _normalize(_cross(n, ref))
    v = _cross(n, u)
    centroid = _face_centroid(vertices, indices)

    def angle(idx: int) -> float:
        rel = _sub(vertices[idx], centroid)
        return math.atan2(_dot(rel, v), _dot(rel, u))

    ordered = sorted(indices, key=angle)
    return _orient_ccw_outward(vertices, ordered)


def _triangulate_convex_face(face: Face) -> list[Tri]:
    """Fan-triangulate a convex CCW polygon into CCW triangles."""
    v0 = face[0]
    return [(v0, face[i], face[i + 1]) for i in range(1, len(face) - 1)]


# ---------------------------------------------------------------------------
# Fixed-point conversion
# ---------------------------------------------------------------------------

def float_to_fixed(value: float, width: int = 16, frac_bits: int = 14) -> int:
    """Convert a float to signed two's-complement fixed-point integer."""
    scale = 1 << frac_bits
    min_val = -(1 << (width - 1))
    max_val = (1 << (width - 1)) - 1
    q = int(round(value * scale))
    if q < min_val or q > max_val:
        raise ValueError(
            f"Value {value} (scaled={q}) overflows signed Q{width - frac_bits}.{frac_bits} "
            f"range [{min_val}, {max_val}]"
        )
    return q


def _format_verilog_signed(val: int, width: int) -> str:
    """Format an integer as a Verilog signed literal (e.g. -16'sd9459 or 16'sd9459)."""
    if val < 0:
        return f"-{width}'sd{-val}"
    return f"{width}'sd{val}"


def _quantize_vec3_list(
    vectors: Iterable[Vec3], width: int = 16, frac_bits: int = 14, scale: float = 1.0
) -> list[tuple[int, int, int]]:
    """Quantize a sequence of 3D vectors to signed fixed-point integers."""
    return [
        (
            float_to_fixed(v[0] * scale, width, frac_bits),
            float_to_fixed(v[1] * scale, width, frac_bits),
            float_to_fixed(v[2] * scale, width, frac_bits),
        )
        for v in vectors
    ]


# ---------------------------------------------------------------------------
# Core Data Model
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class PlatonicSolid:
    """Geometry and topology of a Platonic solid."""

    shape_id: int
    name: str
    vertices: tuple[Vec3, ...]
    faces: tuple[Face, ...]
    triangles: tuple[Tri, ...]
    triangle_face_groups: tuple[int, ...]
    face_groups: tuple[tuple[Tri, ...], ...]
    face_normals: tuple[Vec3, ...]
    triangle_normals: tuple[Vec3, ...]
    edges: tuple[tuple[int, int], ...]

    @property
    def num_vertices(self) -> int:
        return len(self.vertices)

    @property
    def num_faces(self) -> int:
        return len(self.faces)

    @property
    def num_triangles(self) -> int:
        return len(self.triangles)

    @property
    def num_edges(self) -> int:
        return len(self.edges)

    def scaled_vertices(self, radius: float = 1.0) -> list[Vec3]:
        """Return vertices scaled by `radius`."""
        return [_scale(v, radius) for v in self.vertices]

    def fixed_point_vertices(
        self, width: int = 16, frac_bits: int = 14, radius: float = 1.0
    ) -> list[tuple[int, int, int]]:
        """Return vertices quantized to signed fixed-point integers."""
        return _quantize_vec3_list(self.vertices, width=width, frac_bits=frac_bits, scale=radius)

    def fixed_point_face_normals(
        self, width: int = 16, frac_bits: int = 14
    ) -> list[tuple[int, int, int]]:
        """Return unit outward face normals quantized to signed fixed-point integers."""
        return _quantize_vec3_list(self.face_normals, width=width, frac_bits=frac_bits, scale=1.0)

    def to_dict(self, width: int = 16, frac_bits: int = 14, radius: float = 1.0) -> dict:
        """Serialize solid geometry for JSON or engine import."""
        scaled = self.scaled_vertices(radius)
        fp_verts = self.fixed_point_vertices(width, frac_bits, radius)
        fp_normals = self.fixed_point_face_normals(width, frac_bits)
        q_tag = f"q{width - frac_bits}_{frac_bits}"
        return {
            "shape_id": self.shape_id,
            "name": self.name,
            "counts": {
                "vertices": self.num_vertices,
                "edges": self.num_edges,
                "face_groups": self.num_faces,
                "triangles": self.num_triangles,
            },
            "vertices": [list(v) for v in scaled],
            f"vertices_{q_tag}": [list(v) for v in fp_verts],
            "faces": [list(f) for f in self.faces],
            "triangles": [list(t) for t in self.triangles],
            "triangle_face_groups": list(self.triangle_face_groups),
            "face_groups": [[list(t) for t in grp] for grp in self.face_groups],
            "face_normals": [list(n) for n in self.face_normals],
            f"face_normals_{q_tag}": [list(n) for n in fp_normals],
            "edges": [list(e) for e in self.edges],
        }

    def to_obj(self, radius: float = 1.0) -> str:
        """Export as Wavefront OBJ with per-polygon face groups."""
        lines = [f"# {self.name.title()} (unit sphere * {radius})", f"o {self.name}"]
        for x, y, z in self.scaled_vertices(radius):
            lines.append(f"v {x:.9f} {y:.9f} {z:.9f}")
        for nx, ny, nz in self.face_normals:
            lines.append(f"vn {nx:.9f} {ny:.9f} {nz:.9f}")
        for grp_idx, grp_tris in enumerate(self.face_groups):
            lines.append(f"g {self.name}_face_{grp_idx}")
            vn_idx = grp_idx + 1
            for i0, i1, i2 in grp_tris:
                lines.append(f"f {i0 + 1}//{vn_idx} {i1 + 1}//{vn_idx} {i2 + 1}//{vn_idx}")
        return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# Solid builders
# ---------------------------------------------------------------------------

def _build_solid_record(
    shape_id: int,
    name: str,
    raw_vertices: list[Vec3],
    raw_faces: list[Face],
) -> PlatonicSolid:
    vertices = tuple(_normalize(v) for v in raw_vertices)
    vert_list = list(vertices)

    faces = tuple(_orient_ccw_outward(vert_list, list(f)) for f in raw_faces)

    triangles: list[Tri] = []
    triangle_face_groups: list[int] = []
    face_groups: list[tuple[Tri, ...]] = []
    face_normals: list[Vec3] = []
    triangle_normals: list[Vec3] = []

    for grp_id, face in enumerate(faces):
        tris = _triangulate_convex_face(face)
        face_groups.append(tuple(tris))
        fn = _triangle_normal(vert_list[tris[0][0]], vert_list[tris[0][1]], vert_list[tris[0][2]])
        face_normals.append(fn)
        for tri in tris:
            triangles.append(tri)
            triangle_face_groups.append(grp_id)
            triangle_normals.append(
                _triangle_normal(vert_list[tri[0]], vert_list[tri[1]], vert_list[tri[2]])
            )

    edge_set: set[tuple[int, int]] = set()
    for face in faces:
        k = len(face)
        for i in range(k):
            a, b = face[i], face[(i + 1) % k]
            edge_set.add((min(a, b), max(a, b)))
    edges = tuple(sorted(edge_set))

    return PlatonicSolid(
        shape_id=shape_id,
        name=name,
        vertices=vertices,
        faces=faces,
        triangles=tuple(triangles),
        triangle_face_groups=tuple(triangle_face_groups),
        face_groups=tuple(face_groups),
        face_normals=tuple(face_normals),
        triangle_normals=tuple(triangle_normals),
        edges=edges,
    )


def _build_tetrahedron() -> PlatonicSolid:
    vertices: list[Vec3] = [
        (1.0, 1.0, 1.0),
        (1.0, -1.0, -1.0),
        (-1.0, 1.0, -1.0),
        (-1.0, -1.0, 1.0),
    ]
    faces: list[Face] = [
        (0, 1, 2),
        (0, 3, 1),
        (0, 2, 3),
        (1, 3, 2),
    ]
    return _build_solid_record(0, "tetrahedron", vertices, faces)


def _build_octahedron() -> PlatonicSolid:
    vertices: list[Vec3] = [
        (1.0, 0.0, 0.0),   # 0: +X
        (-1.0, 0.0, 0.0),  # 1: -X
        (0.0, 1.0, 0.0),   # 2: +Y
        (0.0, -1.0, 0.0),  # 3: -Y
        (0.0, 0.0, 1.0),   # 4: +Z
        (0.0, 0.0, -1.0),  # 5: -Z
    ]
    faces: list[Face] = [
        (0, 2, 4),
        (2, 1, 4),
        (1, 3, 4),
        (3, 0, 4),
        (2, 0, 5),
        (1, 2, 5),
        (3, 1, 5),
        (0, 3, 5),
    ]
    return _build_solid_record(1, "octahedron", vertices, faces)


def _build_cube() -> PlatonicSolid:
    vertices: list[Vec3] = [
        (-1.0, -1.0, -1.0),  # 0
        (1.0, -1.0, -1.0),   # 1
        (1.0, 1.0, -1.0),    # 2
        (-1.0, 1.0, -1.0),   # 3
        (-1.0, -1.0, 1.0),   # 4
        (1.0, -1.0, 1.0),    # 5
        (1.0, 1.0, 1.0),     # 6
        (-1.0, 1.0, 1.0),    # 7
    ]
    faces: list[Face] = [
        (4, 5, 6, 7),  # +Z
        (1, 0, 3, 2),  # -Z
        (1, 2, 6, 5),  # +X
        (0, 4, 7, 3),  # -X
        (3, 7, 6, 2),  # +Y
        (0, 1, 5, 4),  # -Y
    ]
    return _build_solid_record(2, "cube", vertices, faces)


def _icosahedron_raw_vertices() -> list[Vec3]:
    phi = (1.0 + math.sqrt(5.0)) / 2.0
    verts: list[Vec3] = []
    for s1 in (-1.0, 1.0):
        for s2 in (-1.0, 1.0):
            verts.append((0.0, s1, s2 * phi))
    for s1 in (-1.0, 1.0):
        for s2 in (-1.0, 1.0):
            verts.append((s1, s2 * phi, 0.0))
    for s1 in (-1.0, 1.0):
        for s2 in (-1.0, 1.0):
            verts.append((s2 * phi, 0.0, s1))
    return verts


def _build_icosahedron() -> PlatonicSolid:
    verts = _icosahedron_raw_vertices()
    n = len(verts)
    # Find exact edge length from minimum pairwise distance
    dists = [_dist(verts[i], verts[j]) for i in range(n) for j in range(i + 1, n)]
    edge_len = min(dists)
    tol = 1e-6

    adj = [
        {j for j in range(n) if i != j and abs(_dist(verts[i], verts[j]) - edge_len) < tol}
        for i in range(n)
    ]

    faces: list[Face] = []
    for i in range(n):
        for j in sorted(adj[i]):
            if j <= i:
                continue
            for k in sorted(adj[i] & adj[j]):
                if k <= j:
                    continue
                faces.append(_orient_ccw_outward(verts, [i, j, k]))

    return _build_solid_record(3, "icosahedron", verts, faces)


def _build_dodecahedron() -> PlatonicSolid:
    phi = (1.0 + math.sqrt(5.0)) / 2.0
    inv_phi = 1.0 / phi

    verts: list[Vec3] = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                verts.append((sx, sy, sz))
    for s1 in (-1.0, 1.0):
        for s2 in (-1.0, 1.0):
            verts.append((0.0, s1 * phi, s2 * inv_phi))
    for s1 in (-1.0, 1.0):
        for s2 in (-1.0, 1.0):
            verts.append((s1 * inv_phi, 0.0, s2 * phi))
    for s1 in (-1.0, 1.0):
        for s2 in (-1.0, 1.0):
            verts.append((s1 * phi, s2 * inv_phi, 0.0))

    # The 12 face normals of a dodecahedron are the 12 vertices of the dual icosahedron
    face_dirs = [_normalize(v) for v in _icosahedron_raw_vertices()]
    faces: list[Face] = []
    for direction in face_dirs:
        dots = [(_dot(v, direction), idx) for idx, v in enumerate(verts)]
        max_dot = max(d for d, _ in dots)
        face_verts = [idx for d, idx in dots if abs(d - max_dot) < 1e-6]
        if len(face_verts) != 5:
            raise RuntimeError(f"Expected 5 vertices per dodecahedron face, got {len(face_verts)}")
        faces.append(_sort_coplanar_ccw(verts, face_verts, direction))

    return _build_solid_record(4, "dodecahedron", verts, faces)


def build_all_solids() -> dict[str, PlatonicSolid]:
    """Construct and return all 5 Platonic solids keyed by lowercase name."""
    builders = {
        "tetrahedron": _build_tetrahedron,
        "octahedron": _build_octahedron,
        "cube": _build_cube,
        "icosahedron": _build_icosahedron,
        "dodecahedron": _build_dodecahedron,
    }
    return {name: builders[name]() for name in SHAPE_ORDER}


SOLIDS: dict[str, PlatonicSolid] = build_all_solids()


def get_solid(name: str) -> PlatonicSolid:
    """Fetch a Platonic solid by name (case-insensitive)."""
    key = name.strip().lower()
    if key not in SOLIDS:
        valid = ", ".join(SHAPE_ORDER)
        raise KeyError(f"Unknown Platonic solid {name!r}. Valid options: {valid}")
    return SOLIDS[key]


# ---------------------------------------------------------------------------
# Verification
# ---------------------------------------------------------------------------

EXPECTED_COUNTS: dict[str, tuple[int, int, int, int]] = {
    # name: (V, E, F_poly, T_triangles)
    "tetrahedron": (4, 6, 4, 4),
    "octahedron": (6, 12, 8, 8),
    "cube": (8, 12, 6, 12),
    "icosahedron": (12, 30, 20, 20),
    "dodecahedron": (20, 30, 12, 36),
}


def verify_solid(solid: PlatonicSolid, width: int = 16, frac_bits: int = 14) -> None:
    """Validate geometric, topological, winding, and fixed-point invariants."""
    exp_v, exp_e, exp_f, exp_t = EXPECTED_COUNTS[solid.name]
    assert solid.num_vertices == exp_v, f"{solid.name}: V={solid.num_vertices} != {exp_v}"
    assert solid.num_edges == exp_e, f"{solid.name}: E={solid.num_edges} != {exp_e}"
    assert solid.num_faces == exp_f, f"{solid.name}: F={solid.num_faces} != {exp_f}"
    assert solid.num_triangles == exp_t, f"{solid.name}: T={solid.num_triangles} != {exp_t}"
    assert solid.num_vertices - solid.num_edges + solid.num_faces == 2

    # Unit sphere check
    for idx, v in enumerate(solid.vertices):
        r = _norm(v)
        assert abs(r - 1.0) < 1e-12, f"{solid.name} v[{idx}] norm={r}"

    # Equal edge length check
    edge_lengths = [_dist(solid.vertices[a], solid.vertices[b]) for a, b in solid.edges]
    ref_len = edge_lengths[0]
    for el in edge_lengths:
        assert abs(el - ref_len) < 1e-11, f"{solid.name} unequal edge lengths: {el} vs {ref_len}"

    # Polygon face coplanarity & outward CCW winding check
    poly_half_edges: set[tuple[int, int]] = set()
    for f_idx, face in enumerate(solid.faces):
        fn = solid.face_normals[f_idx]
        centroid = _face_centroid(list(solid.vertices), face)
        assert _dot(fn, centroid) > 0.0, f"{solid.name} face {f_idx} normal points inward"
        d0 = _dot(solid.vertices[face[0]], fn)
        for vi in face:
            assert abs(_dot(solid.vertices[vi], fn) - d0) < 1e-11, (
                f"{solid.name} face {f_idx} is not coplanar"
            )
        k = len(face)
        for i in range(k):
            he = (face[i], face[(i + 1) % k])
            assert he not in poly_half_edges, f"{solid.name}: duplicate polygon half-edge {he}"
            poly_half_edges.add(he)

    for a, b in poly_half_edges:
        assert (b, a) in poly_half_edges, f"{solid.name}: unmatched polygon half-edge {(a, b)}"

    # Triangulated mesh manifold, outward CCW, and face-group alignment check
    tri_half_edges: set[tuple[int, int]] = set()
    for t_idx, tri in enumerate(solid.triangles):
        v0, v1, v2 = (solid.vertices[i] for i in tri)
        cross_vec = _cross(_sub(v1, v0), _sub(v2, v0))
        assert _norm(cross_vec) > 1e-9, f"{solid.name}: degenerate triangle {t_idx}"
        centroid = _face_centroid(list(solid.vertices), tri)
        assert _dot(cross_vec, centroid) > 0.0, f"{solid.name}: triangle {t_idx} points inward"

        grp_id = solid.triangle_face_groups[t_idx]
        assert _dot(solid.triangle_normals[t_idx], solid.face_normals[grp_id]) > 0.999999, (
            f"{solid.name}: triangle {t_idx} normal differs from face group {grp_id}"
        )

        for i in range(3):
            he = (tri[i], tri[(i + 1) % 3])
            assert he not in tri_half_edges, f"{solid.name}: duplicate tri half-edge {he}"
            tri_half_edges.add(he)

    for a, b in tri_half_edges:
        assert (b, a) in tri_half_edges, f"{solid.name}: unmatched tri half-edge {(a, b)}"

    # Face normal unit-length & fixed-point quantization range check
    for f_idx, fn in enumerate(solid.face_normals):
        assert abs(_norm(fn) - 1.0) < 1e-12, f"{solid.name} face_normal[{f_idx}] norm={_norm(fn)}"
    solid.fixed_point_vertices(width=width, frac_bits=frac_bits)
    solid.fixed_point_face_normals(width=width, frac_bits=frac_bits)


def verify_all_solids(width: int = 16, frac_bits: int = 14) -> None:
    """Run all geometric, topological, and fixed-point checks across all 5 solids."""
    for name in SHAPE_ORDER:
        verify_solid(SOLIDS[name], width=width, frac_bits=frac_bits)


# ---------------------------------------------------------------------------
# Verilog LUT generator
# ---------------------------------------------------------------------------

def generate_verilog_lut(
    width: int = 16,
    frac_bits: int = 14,
    radius: float = 1.0,
    registered: bool = False,
    module_name: str = "platonic_lut",
) -> str:
    """Generate a synthesizable Verilog-2001 LUT module for all 5 Platonic solids.

    Default format is signed Q2.14 (16-bit signed, 1.0 == 16384), unit-sphere radius.
    Compatible with `default_nettype none`, Quartus, ModelSim/Questa, and Icarus Verilog.
    """
    verify_all_solids(width=width, frac_bits=frac_bits)

    max_verts = max(s.num_vertices for s in SOLIDS.values())  # 20
    max_tris = max(s.num_triangles for s in SOLIDS.values())  # 36
    vert_addr_bits = (max_verts - 1).bit_length()  # 5
    tri_addr_bits = (max_tris - 1).bit_length()    # 6
    int_bits = width - frac_bits

    lines: list[str] = [
        "`default_nettype none",
        "",
        "// Auto-generated by platonic_solids.py",
        "// Platonic Solids Vertex & Triangle Face-Group LUT",
        "//",
        f"// Coordinate format : signed Q{int_bits}.{frac_bits} ({width}-bit two's complement)",
        f"// Unit scale        : 1.0 = {1 << frac_bits} (radius = {radius})",
        "// Winding           : Counter-clockwise (CCW), outward-facing normals",
        "//",
        "// shape_id encoding:",
    ]
    for solid in SOLIDS.values():
        lines.append(
            f"//   3'd{solid.shape_id} : {solid.name:<12s} "
            f"({solid.num_vertices:2d} vertices, {solid.num_triangles:2d} triangles, "
            f"{solid.num_faces:2d} face groups)"
        )

    port_list = [
        "    input  wire [2:0] shape_id,",
        f"    input  wire [{vert_addr_bits - 1}:0] vert_addr,",
        f"    input  wire [{vert_addr_bits - 1}:0] face_addr,",
        f"    input  wire [{tri_addr_bits - 1}:0] tri_addr,",
        f"    output reg  [{vert_addr_bits - 1}:0] num_verts,",
        f"    output reg  [{tri_addr_bits - 1}:0] num_tris,",
        f"    output reg  [{vert_addr_bits - 1}:0] num_face_groups,",
        f"    output reg  signed [{width - 1}:0] vx,",
        f"    output reg  signed [{width - 1}:0] vy,",
        f"    output reg  signed [{width - 1}:0] vz,",
        f"    output reg  signed [{width - 1}:0] nx,",
        f"    output reg  signed [{width - 1}:0] ny,",
        f"    output reg  signed [{width - 1}:0] nz,",
        f"    output reg  [{vert_addr_bits - 1}:0] tri_v0,",
        f"    output reg  [{vert_addr_bits - 1}:0] tri_v1,",
        f"    output reg  [{vert_addr_bits - 1}:0] tri_v2,",
        f"    output reg  [{vert_addr_bits - 1}:0] tri_face_group",
    ]
    if registered:
        port_list.insert(0, "    input  wire clk,")

    lines.extend([
        f"module {module_name} (",
        *port_list,
        ");",
        "",
        f"    localparam COORD_WIDTH = {width};",
        f"    localparam FRAC_BITS   = {frac_bits};",
        "",
        "    localparam [2:0] SHAPE_TETRAHEDRON  = 3'd0;",
        "    localparam [2:0] SHAPE_OCTAHEDRON   = 3'd1;",
        "    localparam [2:0] SHAPE_CUBE         = 3'd2;",
        "    localparam [2:0] SHAPE_ICOSAHEDRON  = 3'd3;",
        "    localparam [2:0] SHAPE_DODECAHEDRON = 3'd4;",
        "",
    ])

    always_hdr = "    always @(posedge clk) begin" if registered else "    always @(*) begin"

    # 1. Shape metadata LUT
    lines.extend([
        "    // -----------------------------------------------------------------------",
        "    // Shape metadata LUT",
        "    // -----------------------------------------------------------------------",
        always_hdr,
        "        case (shape_id)",
    ])
    for name in SHAPE_ORDER:
        s = SOLIDS[name]
        lines.extend([
            f"            SHAPE_{name.upper()}: begin",
            f"                num_verts       = {vert_addr_bits}'d{s.num_vertices};",
            f"                num_tris        = {tri_addr_bits}'d{s.num_triangles};",
            f"                num_face_groups = {vert_addr_bits}'d{s.num_faces};",
            "            end",
        ])
    lines.extend([
        "            default: begin",
        f"                num_verts       = {vert_addr_bits}'d0;",
        f"                num_tris        = {tri_addr_bits}'d0;",
        f"                num_face_groups = {vert_addr_bits}'d0;",
        "            end",
        "        endcase",
        "    end",
        "",
    ])

    # 2. Vertex coordinate LUT
    lines.extend([
        "    // -----------------------------------------------------------------------",
        f"    // Vertex coordinate LUT (signed Q{int_bits}.{frac_bits}, unit sphere * {radius})",
        "    // -----------------------------------------------------------------------",
        always_hdr,
        "        vx = {COORD_WIDTH{1'b0}};",
        "        vy = {COORD_WIDTH{1'b0}};",
        "        vz = {COORD_WIDTH{1'b0}};",
        "        case ({shape_id, vert_addr})",
    ])
    for name in SHAPE_ORDER:
        s = SOLIDS[name]
        fp_verts = s.fixed_point_vertices(width=width, frac_bits=frac_bits, radius=radius)
        lines.append(f"            // --- {name.title()} ({s.num_vertices} vertices) ---")
        for idx, ((fx, fy, fz), (qx, qy, qz)) in enumerate(
            zip(s.scaled_vertices(radius), fp_verts)
        ):
            sx = _format_verilog_signed(qx, width)
            sy = _format_verilog_signed(qy, width)
            sz = _format_verilog_signed(qz, width)
            lines.append(
                f"            {{SHAPE_{name.upper()}, {vert_addr_bits}'d{idx}}}: "
                f"begin vx = {sx}; vy = {sy}; vz = {sz}; end "
                f"// ({fx:+.6f}, {fy:+.6f}, {fz:+.6f})"
            )
    lines.extend([
        "            default: begin",
        "                vx = {COORD_WIDTH{1'b0}};",
        "                vy = {COORD_WIDTH{1'b0}};",
        "                vz = {COORD_WIDTH{1'b0}};",
        "            end",
        "        endcase",
        "    end",
        "",
    ])

    # 3. Face Normal LUT
    lines.extend([
        "    // -----------------------------------------------------------------------",
        f"    // Face-group outward normal LUT (unit vectors in signed Q{int_bits}.{frac_bits})",
        "    // -----------------------------------------------------------------------",
        always_hdr,
        "        nx = {COORD_WIDTH{1'b0}};",
        "        ny = {COORD_WIDTH{1'b0}};",
        "        nz = {COORD_WIDTH{1'b0}};",
        "        case ({shape_id, face_addr})",
    ])
    for name in SHAPE_ORDER:
        s = SOLIDS[name]
        fp_norms = s.fixed_point_face_normals(width=width, frac_bits=frac_bits)
        lines.append(f"            // --- {name.title()} ({s.num_faces} face groups) ---")
        for f_idx, ((fnx, fny, fnz), (qnx, qny, qnz)) in enumerate(
            zip(s.face_normals, fp_norms)
        ):
            sx = _format_verilog_signed(qnx, width)
            sy = _format_verilog_signed(qny, width)
            sz = _format_verilog_signed(qnz, width)
            lines.append(
                f"            {{SHAPE_{name.upper()}, {vert_addr_bits}'d{f_idx}}}: "
                f"begin nx = {sx}; ny = {sy}; nz = {sz}; end "
                f"// ({fnx:+.6f}, {fny:+.6f}, {fnz:+.6f})"
            )
    lines.extend([
        "            default: begin",
        "                nx = {COORD_WIDTH{1'b0}};",
        "                ny = {COORD_WIDTH{1'b0}};",
        "                nz = {COORD_WIDTH{1'b0}};",
        "            end",
        "        endcase",
        "    end",
        "",
    ])

    # 4. Triangle & Face Group LUT
    lines.extend([
        "    // -----------------------------------------------------------------------",
        "    // Triangle index & face-group LUT (CCW outward winding)",
        "    // -----------------------------------------------------------------------",
        always_hdr,
        f"        tri_v0         = {vert_addr_bits}'d0;",
        f"        tri_v1         = {vert_addr_bits}'d0;",
        f"        tri_v2         = {vert_addr_bits}'d0;",
        f"        tri_face_group = {vert_addr_bits}'d0;",
        "        case ({shape_id, tri_addr})",
    ])
    for name in SHAPE_ORDER:
        s = SOLIDS[name]
        lines.append(
            f"            // --- {name.title()} ({s.num_triangles} triangles, {s.num_faces} face groups) ---"
        )
        for t_idx, ((i0, i1, i2), grp) in enumerate(zip(s.triangles, s.triangle_face_groups)):
            lines.append(
                f"            {{SHAPE_{name.upper()}, {tri_addr_bits}'d{t_idx}}}: "
                f"begin "
                f"tri_v0 = {vert_addr_bits}'d{i0}; "
                f"tri_v1 = {vert_addr_bits}'d{i1}; "
                f"tri_v2 = {vert_addr_bits}'d{i2}; "
                f"tri_face_group = {vert_addr_bits}'d{grp}; "
                "end"
            )
    lines.extend([
        "            default: begin",
        f"                tri_v0         = {vert_addr_bits}'d0;",
        f"                tri_v1         = {vert_addr_bits}'d0;",
        f"                tri_v2         = {vert_addr_bits}'d0;",
        f"                tri_face_group = {vert_addr_bits}'d0;",
        "            end",
        "        endcase",
        "    end",
        "",
        "endmodule",
        "",
        "`default_nettype wire",
        "",
    ])

    return "\n".join(lines)


def generate_json(width: int = 16, frac_bits: int = 14, radius: float = 1.0) -> str:
    """Serialize all 5 Platonic solids to formatted JSON."""
    verify_all_solids(width=width, frac_bits=frac_bits)
    payload = {
        "fixed_point": {
            "width": width,
            "frac_bits": frac_bits,
            "unit_scale": 1 << frac_bits,
            "radius": radius,
        },
        "solids": {
            name: SOLIDS[name].to_dict(width=width, frac_bits=frac_bits, radius=radius)
            for name in SHAPE_ORDER
        },
    }
    return json.dumps(payload, indent=2) + "\n"


def format_summary() -> str:
    """Return a concise summary table of all 5 Platonic solids."""
    header = (
        f"{'ID':>2}  {'Solid':<13} {'Verts':>5} {'Edges':>5} "
        f"{'FaceGrps':>8} {'Tris':>5}  {'Tris/Grp':>8}"
    )
    sep = "-" * len(header)
    rows = [header, sep]
    for name in SHAPE_ORDER:
        s = SOLIDS[name]
        tris_per_grp = s.num_triangles // s.num_faces
        rows.append(
            f"{s.shape_id:>2}  {s.name.title():<13} {s.num_vertices:>5} {s.num_edges:>5} "
            f"{s.num_faces:>8} {s.num_triangles:>5}  {tris_per_grp:>8}"
        )
    return "\n".join(rows)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Calculate vertices and triangle face groups for all 5 Platonic solids "
                    "and export JSON / OBJ / Verilog LUT."
    )
    parser.add_argument(
        "--width", type=int, default=16, help="Fixed-point bit width for Verilog LUT (default: 16)"
    )
    parser.add_argument(
        "--frac-bits",
        type=int,
        default=14,
        help="Fixed-point fractional bits for Verilog LUT (default: 14, so 1.0 == 16384)",
    )
    parser.add_argument(
        "--radius", type=float, default=1.0, help="Sphere radius scale factor (default: 1.0)"
    )
    parser.add_argument(
        "--registered",
        action="store_true",
        help="Generate synchronous posedge-clk registered Verilog LUT instead of combinational",
    )
    parser.add_argument(
        "--out-verilog",
        type=Path,
        default=None,
        help="Output path for hardcoded Verilog LUT (.v)",
    )
    parser.add_argument(
        "--out-json",
        type=Path,
        default=None,
        help="Output path for JSON geometry data (.json)",
    )
    parser.add_argument(
        "--out-obj-dir",
        type=Path,
        default=None,
        help="Output directory for Wavefront .obj meshes",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Run full geometric, topological, and fixed-point verification",
    )

    args = parser.parse_args()

    verify_all_solids(width=args.width, frac_bits=args.frac_bits)
    if args.verify:
        print("Verification PASSED for all 5 Platonic solids.")

    print(format_summary())

    if args.out_verilog is not None:
        args.out_verilog.parent.mkdir(parents=True, exist_ok=True)
        verilog_src = generate_verilog_lut(
            width=args.width,
            frac_bits=args.frac_bits,
            radius=args.radius,
            registered=args.registered,
        )
        args.out_verilog.write_text(verilog_src, encoding="utf-8")
        print(f"Wrote Verilog LUT: {args.out_verilog}")

    if args.out_json is not None:
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        args.out_json.write_text(
            generate_json(width=args.width, frac_bits=args.frac_bits, radius=args.radius),
            encoding="utf-8",
        )
        print(f"Wrote JSON: {args.out_json}")

    if args.out_obj_dir is not None:
        args.out_obj_dir.mkdir(parents=True, exist_ok=True)
        for name in SHAPE_ORDER:
            obj_path = args.out_obj_dir / f"{name}.obj"
            obj_path.write_text(SOLIDS[name].to_obj(radius=args.radius), encoding="utf-8")
        print(f"Wrote 5 OBJ files to: {args.out_obj_dir}")


if __name__ == "__main__":
    main()
