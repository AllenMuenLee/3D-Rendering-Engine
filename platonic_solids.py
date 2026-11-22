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
from typing import Iterable, Tuple


Vec3 = Tuple[float, float, float]
Tri = Tuple[int, int, int]
Face = Tuple[int, ...]

SHAPE_ORDER: tuple[str, ...] = (
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
)

PLATONIC_NAMES: frozenset[str] = frozenset({
    "tetrahedron",
    "octahedron",
    "cube",
    "icosahedron",
    "dodecahedron",
})


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
        return len(self.face_groups)

    @property
    def num_face_groups(self) -> int:
        return len(self.face_groups)

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


class _MeshBuilder:
    """Watertight 3D figure mesh builder with grouped coplanar/shared normals.

    Coordinate convention (right-handed, matches platonic_renderer.v):
      +X : right on screen
      +Y : front (toward viewer)
      +Z : down on screen (-Z is UP)
    """

    def __init__(self) -> None:
        self.vertices: list[Vec3] = []
        self.faces: list[Face] = []
        self.triangles: list[Tri] = []
        self.triangle_face_groups: list[int] = []
        self.face_groups: list[list[Tri]] = []
        self.face_normals: list[Vec3] = []
        self.triangle_normals: list[Vec3] = []

    def add_vertices(self, pts: Iterable[Vec3]) -> int:
        base = len(self.vertices)
        self.vertices.extend(pts)
        return base

    def new_face_group(self, normal: Vec3) -> int:
        gid = len(self.face_groups)
        self.face_groups.append([])
        self.face_normals.append(_normalize(normal))
        return gid

    def add_tri(self, i0: int, i1: int, i2: int, grp_id: int | None = None) -> int:
        v0, v1, v2 = self.vertices[i0], self.vertices[i1], self.vertices[i2]
        tn = _triangle_normal(v0, v1, v2)
        if grp_id is None:
            grp_id = self.new_face_group(tn)
        else:
            if _dot(tn, self.face_normals[grp_id]) < 0.0:
                i1, i2 = i2, i1
                tn = _triangle_normal(self.vertices[i0], self.vertices[i1], self.vertices[i2])
        tri = (i0, i1, i2)
        self.triangles.append(tri)
        self.triangle_face_groups.append(grp_id)
        self.face_groups[grp_id].append(tri)
        self.triangle_normals.append(tn)
        return grp_id

    def add_quad(self, i0: int, i1: int, i2: int, i3: int, grp_id: int | None = None) -> int:
        self.faces.append((i0, i1, i2, i3))
        if grp_id is not None:
            self.add_tri(i0, i1, i2, grp_id)
            self.add_tri(i0, i2, i3, grp_id)
            return grp_id
        n0 = _triangle_normal(self.vertices[i0], self.vertices[i1], self.vertices[i2])
        n1 = _triangle_normal(self.vertices[i0], self.vertices[i2], self.vertices[i3])
        if _dot(n0, n1) > 0.9999:
            gid = self.add_tri(i0, i1, i2)
            self.add_tri(i0, i2, i3, gid)
            return gid
        gid = self.add_tri(i0, i1, i2)
        self.add_tri(i0, i2, i3)
        return gid

    def add_frustum_z(
        self,
        top_corners: list[Vec3],
        bot_corners: list[Vec3],
    ) -> None:
        """Add a closed 8-vertex hexahedron/frustum.

        top_corners (at smaller Z) and bot_corners (at larger Z) are each 4 points:
          0: (-X, -Y), 1: (+X, -Y), 2: (+X, +Y), 3: (-X, +Y)
        """
        b = self.add_vertices([*top_corners, *bot_corners])
        # Same outward-facing winding as _build_cube()
        quads = [
            (b + 4, b + 5, b + 6, b + 7),  # +Z (bottom)
            (b + 1, b + 0, b + 3, b + 2),  # -Z (top)
            (b + 1, b + 2, b + 6, b + 5),  # +X (right)
            (b + 0, b + 4, b + 7, b + 3),  # -X (left)
            (b + 3, b + 7, b + 6, b + 2),  # +Y (front)
            (b + 0, b + 1, b + 5, b + 4),  # -Y (back)
        ]
        for q in quads:
            self.add_quad(*q)

    def add_box(
        self,
        cx: float, cy: float, cz: float,
        hx: float, hy: float, hz: float,
    ) -> None:
        top = [
            (cx - hx, cy - hy, cz - hz),
            (cx + hx, cy - hy, cz - hz),
            (cx + hx, cy + hy, cz - hz),
            (cx - hx, cy + hy, cz - hz),
        ]
        bot = [
            (cx - hx, cy - hy, cz + hz),
            (cx + hx, cy - hy, cz + hz),
            (cx + hx, cy + hy, cz + hz),
            (cx - hx, cy + hy, cz + hz),
        ]
        self.add_frustum_z(top, bot)

    def add_oriented_beam(
        self,
        p0: Vec3,
        p1: Vec3,
        rx0: float,
        ry0: float,
        rx1: float,
        ry1: float,
    ) -> None:
        """Add a tapered 4-sided closed limb/beam from p0 to p1."""
        axis = _sub(p1, p0)
        az = _normalize(axis)
        ref = (0.0, 1.0, 0.0) if abs(az[1]) < 0.9 else (1.0, 0.0, 0.0)
        # Ensure (ax, ay, az) is right-handed like (X, Y, Z)
        ax = _normalize(_cross(ref, az))
        ay = _cross(az, ax)

        def ring(center: Vec3, rx: float, ry: float) -> list[Vec3]:
            return [
                _add(center, _add(_scale(ax, -rx), _scale(ay, -ry))),
                _add(center, _add(_scale(ax, +rx), _scale(ay, -ry))),
                _add(center, _add(_scale(ax, +rx), _scale(ay, +ry))),
                _add(center, _add(_scale(ax, -rx), _scale(ay, +ry))),
            ]

        self.add_frustum_z(ring(p0, rx0, ry0), ring(p1, rx1, ry1))

    def add_extruded_poly_x(
        self,
        profile_yz: list[tuple[float, float]],
        x_left: float,
        x_right: float,
    ) -> None:
        """Extrude a convex 2D polygon in (y, z) along X from x_left to x_right."""
        # Orient profile so signed area in (y, z) is positive -> (+X) outward on x_right
        area2 = sum(
            profile_yz[i][0] * profile_yz[(i + 1) % len(profile_yz)][1]
            - profile_yz[(i + 1) % len(profile_yz)][0] * profile_yz[i][1]
            for i in range(len(profile_yz))
        )
        pts = list(profile_yz) if area2 > 0.0 else list(reversed(profile_yz))
        n = len(pts)
        b = self.add_vertices(
            [(x_right, y, z) for y, z in pts] + [(x_left, y, z) for y, z in pts]
        )
        # Right cap (+X): (y1-y0, z1-z0) x (y2-y0, z2-z0) gives +X when area2 > 0
        g_right = self.new_face_group((1.0, 0.0, 0.0))
        self.faces.append(tuple(b + i for i in range(n)))
        for i in range(1, n - 1):
            self.add_tri(b, b + i, b + i + 1, g_right)
        # Left cap (-X): reversed winding
        g_left = self.new_face_group((-1.0, 0.0, 0.0))
        self.faces.append(tuple(b + n + i for i in reversed(range(n))))
        for i in range(1, n - 1):
            self.add_tri(b + n, b + n + i + 1, b + n + i, g_left)
        # Side walls
        for i in range(n):
            j = (i + 1) % n
            # Edge on right cap is (b+i -> b+j); side quad uses (b+j, b+i, b+n+i, b+n+j)
            self.add_quad(b + j, b + i, b + n + i, b + n + j)

    def add_lathe_z(
        self,
        sides: int,
        rings: list[tuple[float, float]],
        top_apex_z: float | None = None,
        bot_apex_z: float | None = None,
        angle_offset: float = 0.0,
        xy_scale: tuple[float, float] = (1.0, 1.0),
    ) -> None:
        """Build a closed watertight solid of revolution around the Z axis.

        rings: ordered list of (z, radius) from smallest Z (top) to largest Z (bottom).
        """
        sx, sy = xy_scale
        ring_bases: list[int] = []
        for z, r in rings:
            pts = []
            for k in range(sides):
                ang = angle_offset + 2.0 * math.pi * k / sides
                pts.append((r * sx * math.cos(ang), r * sy * math.sin(ang), z))
            ring_bases.append(self.add_vertices(pts))

        # Top cap (-Z)
        r0 = ring_bases[0]
        if top_apex_z is not None:
            apex_top = self.add_vertices([(0.0, 0.0, top_apex_z)])
            for k in range(sides):
                kn = (k + 1) % sides
                # Check: k=0 (+X), kn (+X,+Y); (kn - apex) x (k - apex) -> outward (+X,+Y,-Z)
                self.faces.append((apex_top, r0 + kn, r0 + k))
                self.add_tri(apex_top, r0 + kn, r0 + k)
        else:
            g_top = self.new_face_group((0.0, 0.0, -1.0))
            self.faces.append(tuple(r0 + k for k in reversed(range(sides))))
            for k in range(1, sides - 1):
                self.add_tri(r0, r0 + k + 1, r0 + k, g_top)

        # Side bands between consecutive rings
        for ri in range(len(rings) - 1):
            ra = ring_bases[ri]
            rb = ring_bases[ri + 1]
            for k in range(sides):
                kn = (k + 1) % sides
                # Outward normal points radially out: (ra+k, ra+kn, rb+kn, rb+k)
                # Check: (ra+kn - ra+k) ~ +Y, (rb+kn - ra+k) ~ +Y+Z => (+Y)x(+Z) = +X outward!
                self.add_quad(ra + k, ra + kn, rb + kn, rb + k)

        # Bottom cap (+Z)
        rl = ring_bases[-1]
        if bot_apex_z is not None:
            apex_bot = self.add_vertices([(0.0, 0.0, bot_apex_z)])
            for k in range(sides):
                kn = (k + 1) % sides
                self.faces.append((apex_bot, rl + k, rl + kn))
                self.add_tri(apex_bot, rl + k, rl + kn)
        else:
            g_bot = self.new_face_group((0.0, 0.0, 1.0))
            self.faces.append(tuple(rl + k for k in range(sides)))
            for k in range(1, sides - 1):
                self.add_tri(rl, rl + k, rl + k + 1, g_bot)

    def add_extruded_ribbon_y(
        self,
        centerline_xz: list[tuple[float, float]],
        half_widths: list[float],
        half_thickness_y: float,
    ) -> None:
        """Extrude a tapered 2D ribbon in the (x, z) plane along Y in [-t, +t].

        centerline_xz[0] and [-1] are sharp tapered tips (half_widths == 0).
        All intermediate points have positive half_widths.
        """
        m = len(centerline_xz) - 1
        assert m >= 2 and len(half_widths) == m + 1
        left_pts: list[tuple[float, float]] = []
        right_pts: list[tuple[float, float]] = []
        for i in range(1, m):
            x_prev, z_prev = centerline_xz[i - 1]
            x_cur, z_cur = centerline_xz[i]
            x_next, z_next = centerline_xz[i + 1]
            tx, tz = x_next - x_prev, z_next - z_prev
            tlen = math.hypot(tx, tz)
            # Unit normal to the left of travel in (x, z)
            nx2, nz2 = -tz / tlen, tx / tlen
            w = half_widths[i]
            left_pts.append((x_cur + w * nx2, z_cur + w * nz2))
            right_pts.append((x_cur - w * nx2, z_cur - w * nz2))

        # Construct CCW outline in (x, z) where +cross_y = (bz-az)*(cx-ax) - (bx-ax)*(cz-az) > 0
        # Walk: tip0 -> left[0..m-2] -> tipM -> right[m-2..0]
        ring_xz: list[tuple[float, float]] = [
            centerline_xz[0],
            *left_pts,
            centerline_xz[m],
            *reversed(right_pts),
        ]
        # Verify orientation using the first triangle (tip0, left[0], right[0])
        a, b_pt, c_pt = centerline_xz[0], left_pts[0], right_pts[0]
        cross_y = (b_pt[1] - a[1]) * (c_pt[0] - a[0]) - (b_pt[0] - a[0]) * (c_pt[1] - a[1])
        swap_lr = cross_y < 0.0
        if swap_lr:
            left_pts, right_pts = right_pts, left_pts
            ring_xz = [
                centerline_xz[0],
                *left_pts,
                centerline_xz[m],
                *reversed(right_pts),
            ]

        n_ring = len(ring_xz)  # 2 * m
        t = half_thickness_y
        # Front vertices: base + 0..n_ring-1 (y = +t)
        # Back vertices:  base + n_ring..2*n_ring-1 (y = -t)
        base = self.add_vertices(
            [(x, +t, z) for x, z in ring_xz] + [(x, -t, z) for x, z in ring_xz]
        )

        def f_idx(k: int) -> int:
            return base + k

        def b_idx(k: int) -> int:
            return base + n_ring + k

        # Index mapping on ring_xz:
        #   0          : tip0
        #   1 .. m-1   : left_pts[0 .. m-2]
        #   m          : tipM
        #   m+1 .. 2m-1: reversed(right_pts), so right_pts[i-1] is at index (2*m - i)
        def L(i: int) -> int:
            return i  # i in 1..m-1

        def R(i: int) -> int:
            return 2 * m - i  # i in 1..m-1

        g_front = self.new_face_group((0.0, 1.0, 0.0))
        g_back = self.new_face_group((0.0, -1.0, 0.0))

        # Build strip triangles on front (outward +Y) and back (outward -Y)
        strip_tris: list[tuple[int, int, int]] = []
        # Tip 0 triangle: (0, L(1), R(1))
        strip_tris.append((0, L(1), R(1)))
        # Middle quads: (L(i), L(i+1), R(i+1), R(i))
        for i in range(1, m - 1):
            strip_tris.append((L(i), L(i + 1), R(i + 1)))
            strip_tris.append((L(i), R(i + 1), R(i)))
        # Tip M triangle: (L(m-1), m, R(m-1))
        strip_tris.append((L(m - 1), m, R(m - 1)))

        for i0, i1, i2 in strip_tris:
            self.faces.append((f_idx(i0), f_idx(i1), f_idx(i2)))
            self.add_tri(f_idx(i0), f_idx(i1), f_idx(i2), g_front)
            self.faces.append((b_idx(i0), b_idx(i2), b_idx(i1)))
            self.add_tri(b_idx(i0), b_idx(i2), b_idx(i1), g_back)

        # Side walls around entire closed perimeter (k -> (k+1) % n_ring)
        for k in range(n_ring):
            kn = (k + 1) % n_ring
            # Front boundary has directed edge f_idx(k) -> f_idx(kn)
            # Side quad uses (f_idx(kn), f_idx(k), b_idx(k), b_idx(kn)) -> exact twin & outward!
            self.add_quad(f_idx(kn), f_idx(k), b_idx(k), b_idx(kn))

    def finalize(self, shape_id: int, name: str, max_radius: float = 0.96) -> PlatonicSolid:
        """Center/fit inside sphere of radius `max_radius` and build PlatonicSolid."""
        max_r = max(_norm(v) for v in self.vertices)
        scale = max_radius / max_r if max_r > max_radius else 1.0
        vertices = tuple(_scale(v, scale) for v in self.vertices)

        edge_set: set[tuple[int, int]] = set()
        for tri in self.triangles:
            for i in range(3):
                a, b = tri[i], tri[(i + 1) % 3]
                edge_set.add((min(a, b), max(a, b)))

        return PlatonicSolid(
            shape_id=shape_id,
            name=name,
            vertices=vertices,
            faces=tuple(self.faces),
            triangles=tuple(self.triangles),
            triangle_face_groups=tuple(self.triangle_face_groups),
            face_groups=tuple(tuple(g) for g in self.face_groups),
            face_normals=tuple(self.face_normals),
            triangle_normals=tuple(self.triangle_normals),
            edges=tuple(sorted(edge_set)),
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


def _build_person() -> PlatonicSolid:
    """Stylized low-poly 3D standing human figure (+X right, +Y front, -Z up)."""
    mb = _MeshBuilder()

    # 1. Head: 6-sided faceted head with tapered crown & chin (z = -0.92 .. -0.60)
    mb.add_lathe_z(
        sides=6,
        rings=[
            (-0.88, 0.125),
            (-0.74, 0.155),
            (-0.62, 0.115),
        ],
        top_apex_z=-0.94,
        bot_apex_z=-0.58,
        angle_offset=math.pi / 6.0,
        xy_scale=(0.92, 1.0),
    )

    # 2. Neck (z = -0.60 .. -0.50)
    mb.add_box(cx=0.0, cy=0.0, cz=-0.55, hx=0.055, hy=0.055, hz=0.055)

    # 3. Torso: athletic V-tapered chest to waist (z = -0.52 .. +0.06)
    mb.add_frustum_z(
        top_corners=[
            (-0.23, -0.11, -0.52),
            (+0.23, -0.11, -0.52),
            (+0.23, +0.12, -0.52),
            (-0.23, +0.12, -0.52),
        ],
        bot_corners=[
            (-0.145, -0.095, +0.06),
            (+0.145, -0.095, +0.06),
            (+0.145, +0.100, +0.06),
            (-0.145, +0.100, +0.06),
        ],
    )

    # 4. Arms: natural outward A-pose tucked cleanly into shoulder joints
    for sx in (-1.0, +1.0):
        mb.add_oriented_beam(
            p0=(sx * 0.205, 0.00, -0.47),
            p1=(sx * 0.50, +0.05, +0.04),
            rx0=0.068,
            ry0=0.068,
            rx1=0.048,
            ry1=0.048,
        )

    # 5. Legs: slight athletic stance + forward-pointing feet
    for sx in (-1.0, +1.0):
        mb.add_oriented_beam(
            p0=(sx * 0.095, 0.00, +0.04),
            p1=(sx * 0.145, -0.01, +0.84),
            rx0=0.068,
            ry0=0.078,
            rx1=0.052,
            ry1=0.062,
        )
        # Forward-pointing foot/boot (+Y)
        mb.add_frustum_z(
            top_corners=[
                (sx * 0.145 - 0.055, -0.075, +0.82),
                (sx * 0.145 + 0.055, -0.075, +0.82),
                (sx * 0.145 + 0.055, +0.145, +0.84),
                (sx * 0.145 - 0.055, +0.145, +0.84),
            ],
            bot_corners=[
                (sx * 0.145 - 0.060, -0.080, +0.93),
                (sx * 0.145 + 0.060, -0.080, +0.93),
                (sx * 0.145 + 0.060, +0.165, +0.93),
                (sx * 0.145 - 0.060, +0.165, +0.93),
            ],
        )

    return mb.finalize(5, "person", max_radius=0.96)


def _build_nvidia_logo() -> PlatonicSolid:
    """3D extruded iconic NVIDIA eye/spiral logo (+X right, +Y front, -Z up).

    Sculpted with sharp almond eye corners, bold sweeping outer upper & lower
    lids, and the signature inner spiral eye/pupil in sleek 3D relief depth.
    """
    mb = _MeshBuilder()
    t_y = 0.11

    # Helper: convert (x, u_up, half_width) list to extruded ribbon in (x, z=-u)
    def add_swoosh(pts_xu_w: list[tuple[float, float, float]]) -> None:
        mb.add_extruded_ribbon_y(
            centerline_xz=[(x, -u) for x, u, _ in pts_xu_w],
            half_widths=[w for _, _, w in pts_xu_w],
            half_thickness_y=t_y,
        )

    # 1. Upper-outer almond eye arch: razor-sharp left tip -> high bold brow -> sharp right corner
    add_swoosh([
        (-0.96, -0.02, 0.000),  # sharp left eye corner
        (-0.72, +0.26, 0.052),
        (-0.36, +0.47, 0.078),
        (+0.04, +0.52, 0.084),  # bold top brow peak
        (+0.44, +0.41, 0.072),
        (+0.76, +0.20, 0.048),
        (+0.96, -0.02, 0.000),  # sharp right eye corner
    ])

    # 2. Lower-outer sweeping eye jaw: right corner -> deep bottom curve -> curls up inside left
    add_swoosh([
        (+0.90, -0.06, 0.000),  # tucked right corner
        (+0.66, -0.28, 0.052),
        (+0.30, -0.45, 0.074),
        (-0.12, -0.49, 0.076),  # bold bottom arch
        (-0.52, -0.35, 0.064),
        (-0.74, -0.10, 0.052),
        (-0.58, +0.16, 0.052),  # curls inside left
        (-0.24, +0.29, 0.056),  # inner upper eyelid
        (+0.16, +0.25, 0.052),
        (+0.48, +0.08, 0.036),
        (+0.62, -0.03, 0.000),  # sharp inner-right eye corner
    ])

    # 3. Inner pupil spiral hook: sweeps from inner-right under pupil and curls into eye center
    add_swoosh([
        (+0.50, -0.07, 0.000),
        (+0.26, -0.21, 0.044),
        (-0.08, -0.23, 0.048),
        (-0.33, -0.09, 0.046),
        (-0.26, +0.07, 0.046),
        (-0.02, +0.11, 0.048),
        (+0.20, +0.03, 0.034),
        (+0.30, -0.03, 0.000),  # sharp central pupil tip
    ])

    return mb.finalize(6, "nvidia_logo", max_radius=0.96)


def _build_rocket() -> PlatonicSolid:
    """3D spacecraft rocket with pointed nose cone, fuselage, nozzle, and 4 swept fins."""
    mb = _MeshBuilder()

    # 1. Body of revolution (6-sided): nose cone -> fuselage -> bulkhead -> flared nozzle
    mb.add_lathe_z(
        sides=6,
        rings=[
            (-0.48, 0.23),
            (+0.46, 0.25),
            (+0.60, 0.17),
            (+0.86, 0.23),
        ],
        top_apex_z=-0.95,
        bot_apex_z=None,
        angle_offset=0.0,
    )

    # 2. Four swept aerodynamic tail fins at 0, 90, 180, 270 deg around Z
    for k in range(4):
        ang = k * (math.pi / 2.0)
        ca, sa = math.cos(ang), math.sin(ang)

        def rot_z(r: float, t: float, z: float) -> Vec3:
            return (r * ca - t * sa, r * sa + t * ca, z)

        # Swept fin frustum: root embedded slightly inside fuselage (r=0.19) to tip (r=0.56)
        r_in, r_out = 0.19, 0.56
        t_in, t_out = 0.032, 0.014
        z_top_in, z_bot_in = +0.06, +0.58
        z_top_out, z_bot_out = +0.44, +0.84

        # Build closed 8-vertex fin with exact outward winding
        # Inner edge (r=r_in): 4 corners; outer edge (r=r_out): 4 corners
        top_c = [
            rot_z(r_in, -t_in, z_top_in),
            rot_z(r_out, -t_out, z_top_out),
            rot_z(r_out, +t_out, z_top_out),
            rot_z(r_in, +t_in, z_top_in),
        ]
        bot_c = [
            rot_z(r_in, -t_in, z_bot_in),
            rot_z(r_out, -t_out, z_bot_out),
            rot_z(r_out, +t_out, z_bot_out),
            rot_z(r_in, +t_in, z_bot_in),
        ]
        mb.add_frustum_z(top_c, bot_c)

    return mb.finalize(7, "rocket", max_radius=0.96)


def _build_jet() -> PlatonicSolid:
    """3D swept-wing fighter jet (nose -Z, wings +/-X, dorsal canopy & tail fin +Y)."""
    mb = _MeshBuilder()

    # 1. Sleek diamond-section fuselage (4-sided lathe around Z)
    mb.add_lathe_z(
        sides=4,
        rings=[
            (-0.34, 0.14),
            (+0.50, 0.15),
            (+0.82, 0.09),
        ],
        top_apex_z=-0.94,
        bot_apex_z=None,
        angle_offset=0.0,
        xy_scale=(1.10, 0.78),
    )

    # 2. Raised cockpit canopy on dorsal (+Y) side of forward fuselage
    mb.add_frustum_z(
        top_corners=[
            (-0.045, +0.06, -0.44),
            (+0.045, +0.06, -0.44),
            (+0.045, +0.16, -0.34),
            (-0.045, +0.16, -0.34),
        ],
        bot_corners=[
            (-0.065, +0.06, -0.02),
            (+0.065, +0.06, -0.02),
            (+0.065, +0.18, -0.08),
            (-0.065, +0.18, -0.08),
        ],
    )

    # 3. Left & Right swept main delta wings
    for sx in (-1.0, +1.0):
        # Leading edge (smaller Z) -> trailing edge (larger Z)
        x_in, x_out = sx * 0.10, sx * 0.92
        x0, x1 = (x_in, x_out) if sx > 0 else (x_out, x_in)
        z_le_0, z_le_1 = (-0.22, +0.34) if sx > 0 else (+0.34, -0.22)
        z_te_0, z_te_1 = (+0.44, +0.52) if sx > 0 else (+0.52, +0.44)
        ty_0, ty_1 = (0.042, 0.014) if sx > 0 else (0.014, 0.042)
        mb.add_frustum_z(
            top_corners=[
                (x0, -ty_0, z_le_0),
                (x1, -ty_1, z_le_1),
                (x1, +ty_1, z_le_1),
                (x0, +ty_0, z_le_0),
            ],
            bot_corners=[
                (x0, -ty_0, z_te_0),
                (x1, -ty_1, z_te_1),
                (x1, +ty_1, z_te_1),
                (x0, +ty_0, z_te_0),
            ],
        )

    # 4. Left & Right horizontal tailplanes
    for sx in (-1.0, +1.0):
        x_in, x_out = sx * 0.08, sx * 0.44
        x0, x1 = (x_in, x_out) if sx > 0 else (x_out, x_in)
        z_le_0, z_le_1 = (+0.54, +0.72) if sx > 0 else (+0.72, +0.54)
        z_te_0, z_te_1 = (+0.78, +0.84) if sx > 0 else (+0.84, +0.78)
        ty_0, ty_1 = (0.026, 0.012) if sx > 0 else (0.012, 0.026)
        mb.add_frustum_z(
            top_corners=[
                (x0, -ty_0, z_le_0),
                (x1, -ty_1, z_le_1),
                (x1, +ty_1, z_le_1),
                (x0, +ty_0, z_le_0),
            ],
            bot_corners=[
                (x0, -ty_0, z_te_0),
                (x1, -ty_1, z_te_1),
                (x1, +ty_1, z_te_1),
                (x0, +ty_0, z_te_0),
            ],
        )

    # 5. Swept vertical tail fin on dorsal (+Y) side
    mb.add_frustum_z(
        top_corners=[
            (-0.024, +0.06, +0.34),
            (+0.024, +0.06, +0.34),
            (+0.012, +0.38, +0.62),
            (-0.012, +0.38, +0.62),
        ],
        bot_corners=[
            (-0.024, +0.06, +0.82),
            (+0.024, +0.06, +0.82),
            (+0.012, +0.38, +0.84),
            (-0.012, +0.38, +0.84),
        ],
    )

    return mb.finalize(8, "jet", max_radius=0.96)


def _build_star() -> PlatonicSolid:
    """3D faceted 5-pointed star (+X right, +Y front, -Z top tip)."""
    mb = _MeshBuilder()
    r_out = 0.96
    r_in = 0.40
    depth_y = 0.34

    # 10 perimeter points alternating outer tips and inner valleys in (X, Z) plane
    # Top tip at (0, 0, -r_out), stepping clockwise on screen (increasing angle from -Z toward +X)
    ring_pts: list[Vec3] = []
    for k in range(10):
        ang = k * (math.pi / 5.0)  # 0, 36, 72, ... deg
        r = r_out if (k % 2 == 0) else r_in
        x = r * math.sin(ang)
        z = -r * math.cos(ang)
        ring_pts.append((x, 0.0, z))

    b_ring = mb.add_vertices(ring_pts)
    b_front = mb.add_vertices([(0.0, +depth_y, 0.0)])
    b_back = mb.add_vertices([(0.0, -depth_y, 0.0)])

    for k in range(10):
        kn = (k + 1) % 10
        # Check front normal: (ring[k] - front) x (ring[kn] - front)
        # k=0: (0, -d, -R), kn=1: (+x, -d, -z) -> cross_y = (-R)*(+x) - 0 < 0,
        # so (front, ring[kn], ring[k]) has positive +Y outward normal!
        mb.faces.append((b_front, b_ring + kn, b_ring + k))
        mb.add_tri(b_front, b_ring + kn, b_ring + k)
        # Back face (-Y outward): reversed winding
        mb.faces.append((b_back, b_ring + k, b_ring + kn))
        mb.add_tri(b_back, b_ring + k, b_ring + kn)

    return mb.finalize(9, "star", max_radius=0.96)


def build_all_solids() -> dict[str, PlatonicSolid]:
    """Construct and return all 10 3D models (5 Platonic solids + 5 custom figures)."""
    builders = {
        "tetrahedron": _build_tetrahedron,
        "octahedron": _build_octahedron,
        "cube": _build_cube,
        "icosahedron": _build_icosahedron,
        "dodecahedron": _build_dodecahedron,
        "person": _build_person,
        "nvidia_logo": _build_nvidia_logo,
        "rocket": _build_rocket,
        "jet": _build_jet,
        "star": _build_star,
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
    is_platonic = solid.name in PLATONIC_NAMES

    if is_platonic:
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
    else:
        # Custom 3D figure checks: inside unit sphere, non-empty, LUT bit limits
        assert 0 < solid.num_vertices <= 255, f"{solid.name}: V={solid.num_vertices} out of 8-bit range"
        assert 0 < solid.num_face_groups <= 255, f"{solid.name}: F={solid.num_face_groups} out of 8-bit range"
        assert 0 < solid.num_triangles <= 255, f"{solid.name}: T={solid.num_triangles} out of 8-bit range"
        for idx, v in enumerate(solid.vertices):
            r = _norm(v)
            assert r <= 1.0 + 1e-9, f"{solid.name} v[{idx}] radius={r} exceeds unit sphere"

    # Triangulated mesh manifold, outward orientation, and face-group alignment check
    tri_half_edges: set[tuple[int, int]] = set()
    signed_vol_6 = 0.0
    for t_idx, tri in enumerate(solid.triangles):
        v0, v1, v2 = (solid.vertices[i] for i in tri)
        cross_vec = _cross(_sub(v1, v0), _sub(v2, v0))
        assert _norm(cross_vec) > 1e-7, f"{solid.name}: degenerate triangle {t_idx}"
        signed_vol_6 += _dot(v0, _cross(v1, v2))
        if is_platonic:
            centroid = _face_centroid(list(solid.vertices), tri)
            assert _dot(cross_vec, centroid) > 0.0, f"{solid.name}: triangle {t_idx} points inward"

        grp_id = solid.triangle_face_groups[t_idx]
        assert _dot(solid.triangle_normals[t_idx], solid.face_normals[grp_id]) > 0.999, (
            f"{solid.name}: triangle {t_idx} normal differs from face group {grp_id}"
        )

        for i in range(3):
            he = (tri[i], tri[(i + 1) % 3])
            assert he not in tri_half_edges, f"{solid.name}: duplicate tri half-edge {he}"
            tri_half_edges.add(he)

    for a, b in tri_half_edges:
        assert (b, a) in tri_half_edges, f"{solid.name}: unmatched tri half-edge {(a, b)}"
    assert signed_vol_6 > 0.0, f"{solid.name}: negative signed volume ({signed_vol_6}) -> inverted normals"

    # Face normal unit-length & fixed-point quantization range check
    for f_idx, fn in enumerate(solid.face_normals):
        assert abs(_norm(fn) - 1.0) < 1e-9, f"{solid.name} face_normal[{f_idx}] norm={_norm(fn)}"
    solid.fixed_point_vertices(width=width, frac_bits=frac_bits)
    solid.fixed_point_face_normals(width=width, frac_bits=frac_bits)


def verify_all_solids(width: int = 16, frac_bits: int = 14) -> None:
    """Run all geometric, topological, and fixed-point checks across all models."""
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
    """Generate a synthesizable Verilog-2001 LUT module for all 10 3D models.

    Default format is signed Q2.14 (16-bit signed, 1.0 == 16384), unit-sphere radius.
    Compatible with `default_nettype none`, Quartus, ModelSim/Questa, and Icarus Verilog.
    """
    verify_all_solids(width=width, frac_bits=frac_bits)

    shape_id_bits = 4   # supports up to 16 shapes (0..9 used)
    vert_addr_bits = 8  # supports up to 255 vertices per model
    face_addr_bits = 8  # supports up to 255 face groups per model
    tri_addr_bits = 8   # supports up to 255 triangles per model
    int_bits = width - frac_bits

    lines: list[str] = [
        "`default_nettype none",
        "",
        "// Auto-generated by platonic_solids.py",
        "// 3D Shapes & Custom Figures Vertex & Triangle Face-Group LUT",
        "//",
        f"// Coordinate format : signed Q{int_bits}.{frac_bits} ({width}-bit two's complement)",
        f"// Unit scale        : 1.0 = {1 << frac_bits} (radius = {radius})",
        "// Winding           : Counter-clockwise (CCW), outward-facing normals",
        "//",
        "// shape_id encoding (SW[9:5] Platonic solids, SW[4:0] custom 3D figures):",
    ]
    for solid in SOLIDS.values():
        lines.append(
            f"//   {shape_id_bits}'d{solid.shape_id:<2d}: {solid.name:<12s} "
            f"({solid.num_vertices:3d} vertices, {solid.num_triangles:3d} triangles, "
            f"{solid.num_face_groups:3d} face groups)"
        )

    port_list = [
        f"    input  wire [{shape_id_bits - 1}:0] shape_id,",
        f"    input  wire [{vert_addr_bits - 1}:0] vert_addr,",
        f"    input  wire [{face_addr_bits - 1}:0] face_addr,",
        f"    input  wire [{tri_addr_bits - 1}:0] tri_addr,",
        f"    output reg  [{vert_addr_bits - 1}:0] num_verts,",
        f"    output reg  [{tri_addr_bits - 1}:0] num_tris,",
        f"    output reg  [{face_addr_bits - 1}:0] num_face_groups,",
        f"    output reg  signed [{width - 1}:0] vx,",
        f"    output reg  signed [{width - 1}:0] vy,",
        f"    output reg  signed [{width - 1}:0] vz,",
        f"    output reg  signed [{width - 1}:0] nx,",
        f"    output reg  signed [{width - 1}:0] ny,",
        f"    output reg  signed [{width - 1}:0] nz,",
        f"    output reg  [{vert_addr_bits - 1}:0] tri_v0,",
        f"    output reg  [{vert_addr_bits - 1}:0] tri_v1,",
        f"    output reg  [{vert_addr_bits - 1}:0] tri_v2,",
        f"    output reg  [{face_addr_bits - 1}:0] tri_face_group",
    ]
    if registered:
        port_list.insert(0, "    input  wire clk,")

    shape_params = [
        f"    localparam [{shape_id_bits - 1}:0] SHAPE_{name.upper():<12s} = {shape_id_bits}'d{SOLIDS[name].shape_id};"
        for name in SHAPE_ORDER
    ]

    lines.extend([
        f"module {module_name} (",
        *port_list,
        ");",
        "",
        f"    localparam COORD_WIDTH = {width};",
        f"    localparam FRAC_BITS   = {frac_bits};",
        "",
        *shape_params,
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
            f"                num_face_groups = {face_addr_bits}'d{s.num_face_groups};",
            "            end",
        ])
    lines.extend([
        "            default: begin",
        f"                num_verts       = {vert_addr_bits}'d0;",
        f"                num_tris        = {tri_addr_bits}'d0;",
        f"                num_face_groups = {face_addr_bits}'d0;",
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
        lines.append(f"            // --- {name.replace('_', ' ').title()} ({s.num_vertices} vertices) ---")
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
        lines.append(f"            // --- {name.replace('_', ' ').title()} ({s.num_face_groups} face groups) ---")
        for f_idx, ((fnx, fny, fnz), (qnx, qny, qnz)) in enumerate(
            zip(s.face_normals, fp_norms)
        ):
            sx = _format_verilog_signed(qnx, width)
            sy = _format_verilog_signed(qny, width)
            sz = _format_verilog_signed(qnz, width)
            lines.append(
                f"            {{SHAPE_{name.upper()}, {face_addr_bits}'d{f_idx}}}: "
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
        f"        tri_face_group = {face_addr_bits}'d0;",
        "        case ({shape_id, tri_addr})",
    ])
    for name in SHAPE_ORDER:
        s = SOLIDS[name]
        lines.append(
            f"            // --- {name.replace('_', ' ').title()} "
            f"({s.num_triangles} triangles, {s.num_face_groups} face groups) ---"
        )
        for t_idx, ((i0, i1, i2), grp) in enumerate(zip(s.triangles, s.triangle_face_groups)):
            lines.append(
                f"            {{SHAPE_{name.upper()}, {tri_addr_bits}'d{t_idx}}}: "
                f"begin "
                f"tri_v0 = {vert_addr_bits}'d{i0}; "
                f"tri_v1 = {vert_addr_bits}'d{i1}; "
                f"tri_v2 = {vert_addr_bits}'d{i2}; "
                f"tri_face_group = {face_addr_bits}'d{grp}; "
                "end"
            )
    lines.extend([
        "            default: begin",
        f"                tri_v0         = {vert_addr_bits}'d0;",
        f"                tri_v1         = {vert_addr_bits}'d0;",
        f"                tri_v2         = {vert_addr_bits}'d0;",
        f"                tri_face_group = {face_addr_bits}'d0;",
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
    """Return a concise summary table of all 10 3D models."""
    header = (
        f"{'ID':>2}  {'Model':<13} {'Verts':>5} {'Edges':>5} "
        f"{'FaceGrps':>8} {'Tris':>5}  {'AvgT/G':>6}"
    )
    sep = "-" * len(header)
    rows = [header, sep]
    for name in SHAPE_ORDER:
        s = SOLIDS[name]
        avg_tg = s.num_triangles / s.num_face_groups
        rows.append(
            f"{s.shape_id:>2}  {name:<13} {s.num_vertices:>5} {s.num_edges:>5} "
            f"{s.num_face_groups:>8} {s.num_triangles:>5}  {avg_tg:>6.2f}"
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
        print("Verification PASSED for all 10 3D models (5 Platonic solids + 5 custom figures).")

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
