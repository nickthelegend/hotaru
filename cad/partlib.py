"""partlib — pure-python CAD kernel for HOTARU (animated servo desk lamp).

No OpenSCAD, no CadQuery, no OCC. Everything is built from 2D shapely
profiles extruded/lofted into closed triangle shells. There is deliberately
NO 3D CSG: each printable part is a union of individually-watertight shells
(overlapping or coplanar shells are merged by the slicer). 2D booleans
(shapely) are allowed and encouraged.

Units: mm. X right, Y back, Z up. See SPEC.md for the dimensional contract.

The two rules that keep this honest:

  1. Build each closed shell in its own `Mesh(weld=True)` so its walls and
     caps stitch into one manifold, then merge shells with `+=`. Merging
     never welds across meshes -- that is how shells stay separate.
  2. Where shells stack, overlap them by OVL (0.2mm) in Z or radially so
     the slicer fuses them. To keep an opening's edges EXACT, use the
     band-split trick: the band carrying the opening is stretched OVL into
     its solid neighbours (its cut runs the band's full height), while the
     solid neighbour ends exactly at the opening edge -- the neighbour's
     cap face IS the opening's top/bottom face.

Typical use:
    m = Mesh()
    m += prism(rounded_rect(60, 40, 4), 0.0, 2.4)
    m += prism(ring2d(rounded_rect(60,40,4), rounded_rect(55.2,35.2,3)), 2.4, 20)
    report = validate(m)          # every connected shell closed & manifold
    stl_write("base.stl", m)
    glb_write("base.glb", [("base", m, "#AEB4BC")])
"""
from __future__ import annotations

import json
import math
import struct

import numpy as np
import shapely
from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

EPS = 1e-9

OVL = 0.2          # standard overlap between stacked shells (slicer fuses)

# ------------------------------------------------------------ 2D shapes ----

def rounded_rect(w, h, r, seg=10):
    """Axis-aligned rounded rectangle centered at origin (CCW)."""
    r = min(r, w / 2 - 1e-6, h / 2 - 1e-6)
    if r <= 0:
        return box(-w / 2, -h / 2, w / 2, h / 2)
    cx, cy = w / 2 - r, h / 2 - r
    corners = [((cx, -cy), -90), ((cx, cy), 0), ((-cx, cy), 90), ((-cx, -cy), 180)]
    pts = []
    for (ox, oy), a0 in corners:
        for t in np.linspace(math.radians(a0), math.radians(a0 + 90), seg + 1):
            pts.append((ox + r * math.cos(t), oy + r * math.sin(t)))
    return Polygon(pts)


def circle(d, seg=64):
    """Circle of diameter d centered at origin (CCW)."""
    t = np.linspace(0, 2 * math.pi, seg, endpoint=False)
    return Polygon(np.column_stack([d / 2 * np.cos(t), d / 2 * np.sin(t)]))


def ring2d(outer, inner):
    """outer minus inner -- convenience for wall/annulus profiles."""
    return outer.difference(inner)


def slot(length, width, seg=12):
    """Obround / capsule of overall `length` in X and `width` in Y, centered.

    The workhorse for cable channels, zip-tie slots and adjustment slots.
    """
    r = width / 2.0
    c = max(length / 2.0 - r, 0.0)
    return LineString([(-c, 0), (c, 0)]).buffer(r, quad_segs=seg)


def ngon(across_flats, n=6, seg_rot=0.0):
    """Regular n-gon sized ACROSS FLATS (nut pockets, hex bores)."""
    r = across_flats / 2.0 / math.cos(math.pi / n)
    a = np.linspace(0, 2 * math.pi, n, endpoint=False) + math.radians(seg_rot)
    return Polygon(np.column_stack([r * np.cos(a), r * np.sin(a)]))


def teardrop(d, seg=64, angle=90.0):
    """PLA-friendly horizontal hole: circle + a 45-deg roof so the top of the
    bore self-supports instead of drooping. `angle` points the roof apex
    (default +Y). Use for bores whose axis lies in the print plane."""
    c = circle(d, seg)
    r = d / 2.0
    apex = r * math.sqrt(2.0)
    roof = Polygon([(-r, 0), (r, 0), (0, apex)])
    return unary_union([c, affinity.rotate(roof, angle - 90.0, origin=(0, 0))])


def chamfer_pair(profile, delta):
    """(profile, profile shrunk by `delta`) -- the two rings of a 45-deg
    chamfer/lead-in, ready for loft_solid()."""
    return profile, profile.buffer(-delta, join_style=2)


def resample_ring(poly, n, start_angle=0.0):
    """Resample a polygon's exterior to exactly n points by arclength,
    starting near the boundary point in direction start_angle from centroid.
    Use to give two profiles matching vertex counts before loft_solid()."""
    ring = orient(poly, 1.0).exterior
    L = ring.length
    cx, cy = poly.centroid.x, poly.centroid.y
    probe = LineString([(cx, cy), (cx + 1e4 * math.cos(start_angle),
                                   cy + 1e4 * math.sin(start_angle))])
    hit = ring.intersection(probe)
    d0 = ring.project(list(hit.geoms)[0] if hit.geom_type.startswith("Multi") else
                      (hit if hit.geom_type == "Point" else Point(ring.coords[0])))
    pts = [ring.interpolate((d0 + L * i / n) % L) for i in range(n)]
    return Polygon([(p.x, p.y) for p in pts])


def stroke(coords, w):
    """A line of coords thickened to width w (round caps) -- 2D rib/channel."""
    return LineString(coords).buffer(w / 2.0, quad_segs=8)


def smooth(g, r):
    """Fillet outer corners and de-spike a profile by r (dilate then erode)."""
    return g.buffer(r, quad_segs=8).buffer(-r, quad_segs=8)

# ----------------------------------------------------------------- mesh ----

class Mesh:
    """Triangle soup with optional coordinate welding.

    Mesh(weld=True): the add_* builders reuse a vertex index for identical
    (rounded) coordinates, so walls and caps of ONE shell stitch into a
    closed manifold. Merging meshes with `+=` never welds across meshes --
    that is how separate shells stay separate (the slicer unions them).
    Build each closed shell in its own welded Mesh, then merge.
    """

    def __init__(self, weld=False):
        self.V: list = []
        self.F: list = []
        self._weld = {} if weld else None

    # -- low-level builders (compose these for custom shells) --

    def _pt(self, x, y, z):
        if self._weld is None:
            self.V.append((x, y, z))
            return len(self.V) - 1
        key = (round(x, 6), round(y, 6), round(z, 6))
        i = self._weld.get(key)
        if i is None:
            self.V.append((x, y, z))
            i = self._weld[key] = len(self.V) - 1
        return i

    def add_ring_wall(self, pts, z0, z1):
        """Vertical wall around a 2D ring. CCW ring -> outward normals,
        CW ring (a hole) -> normals face into the hole (out of the solid)."""
        n = len(pts)
        b = [self._pt(x, y, z0) for x, y in pts]
        t = [self._pt(x, y, z1) for x, y in pts]
        for i in range(n):
            j = (i + 1) % n
            self.F.append((b[i], b[j], t[j]))
            self.F.append((b[i], t[j], t[i]))

    def add_loft_wall(self, pts_a, z0, pts_b, z1):
        """Wall between two index-aligned CCW rings at different heights."""
        if len(pts_a) != len(pts_b):
            raise ValueError("loft rings must have equal point counts "
                             f"({len(pts_a)} vs {len(pts_b)}); use resample_ring()")
        n = len(pts_a)
        b = [self._pt(x, y, z0) for x, y in pts_a]
        t = [self._pt(x, y, z1) for x, y in pts_b]
        for i in range(n):
            j = (i + 1) % n
            self.F.append((b[i], b[j], t[j]))
            self.F.append((b[i], t[j], t[i]))

    def add_cap(self, geom, z, up):
        """Flat triangulated face of a (Multi)Polygon at height z.
        up=True -> normal +Z, else -Z."""
        for poly in _polys(geom):
            poly = orient(poly, 1.0)
            tris = shapely.constrained_delaunay_triangles(poly)
            local = {}
            for ring in [poly.exterior, *poly.interiors]:
                for x, y in list(ring.coords)[:-1]:
                    local[(round(x, 6), round(y, 6))] = self._pt(x, y, z)
            for tri in tris.geoms:
                cs = [(round(x, 6), round(y, 6)) for x, y in tri.exterior.coords[:-1]]
                if not all(c in local for c in cs):
                    raise RuntimeError("CDT introduced a vertex not on the input rings")
                a, b, c = (local[c] for c in cs)
                area = ((cs[1][0] - cs[0][0]) * (cs[2][1] - cs[0][1])
                        - (cs[2][0] - cs[0][0]) * (cs[1][1] - cs[0][1]))
                if abs(area) < 1e-9:
                    continue
                if (area > 0) != up:
                    a, c = c, a
                self.F.append((a, b, c))

    # -- transforms / merge --

    def _np(self):
        return np.asarray(self.V, dtype=np.float64), np.asarray(self.F, dtype=np.int64)

    def translate(self, dx=0.0, dy=0.0, dz=0.0):
        self.V = [(x + dx, y + dy, z + dz) for x, y, z in self.V]
        return self

    def rotate_z(self, deg, about=(0.0, 0.0)):
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        ox, oy = about
        self.V = [((x - ox) * c - (y - oy) * s + ox,
                   (x - ox) * s + (y - oy) * c + oy, z) for x, y, z in self.V]
        return self

    def rotate_x(self, deg, about=(0.0, 0.0)):
        """Rotate about the X axis through (y,z)=about. Proper rotation, so
        triangle winding (and therefore normals) stay valid."""
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        oy, oz = about
        self.V = [(x, (y - oy) * c - (z - oz) * s + oy,
                   (y - oy) * s + (z - oz) * c + oz) for x, y, z in self.V]
        return self

    def rotate_y(self, deg, about=(0.0, 0.0)):
        """Rotate about the Y axis through (x,z)=about."""
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        ox, oz = about
        self.V = [((x - ox) * c + (z - oz) * s + ox, y,
                   -(x - ox) * s + (z - oz) * c + oz) for x, y, z in self.V]
        return self

    def bounds(self):
        """(xmin, ymin, zmin, xmax, ymax, zmax)."""
        V, _ = self._np()
        return (*V.min(axis=0), *V.max(axis=0))

    def __iadd__(self, other):
        base = len(self.V)
        self.V.extend(other.V)
        self.F.extend((a + base, b + base, c + base) for a, b, c in other.F)
        return self

    def copy(self):
        m = Mesh()
        m.V = list(self.V)
        m.F = list(self.F)
        return m


def _polys(geom):
    if isinstance(geom, Polygon):
        return [] if geom.is_empty else [geom]
    if isinstance(geom, MultiPolygon):
        return [p for p in geom.geoms if not p.is_empty]
    if hasattr(geom, "geoms"):
        out = []
        for g in geom.geoms:
            out.extend(_polys(g))
        return out
    raise TypeError(f"expected polygonal geometry, got {geom.geom_type}")


def _rings(poly):
    """(exterior CCW, holes CW) as open point lists, consecutive dups removed."""
    poly = orient(poly, 1.0)
    def clean(ring):
        pts = [(x, y) for x, y in list(ring.coords)[:-1]]
        out = [p for i, p in enumerate(pts)
               if abs(p[0] - pts[i - 1][0]) > EPS or abs(p[1] - pts[i - 1][1]) > EPS]
        return out
    return clean(poly.exterior), [clean(r) for r in poly.interiors]


def prism(geom, z0, z1):
    """Closed extrusion of a (Multi)Polygon (holes supported)."""
    out = Mesh()
    for poly in _polys(geom):
        m = Mesh(weld=True)
        ext, holes = _rings(poly)
        m.add_ring_wall(ext, z0, z1)
        for h in holes:
            m.add_ring_wall(h, z0, z1)
        m.add_cap(poly, z1, up=True)
        m.add_cap(poly, z0, up=False)
        out += m
    return out


def loft_solid(poly_a, z0, poly_b, z1):
    """Closed solid between two hole-free profiles with equal vertex counts."""
    ea, ha = _rings(poly_a)
    eb, hb = _rings(poly_b)
    if ha or hb:
        raise ValueError("loft_solid: profiles must not have holes")
    m = Mesh(weld=True)
    m.add_loft_wall(ea, z0, eb, z1)
    m.add_cap(poly_b, z1, up=True)
    m.add_cap(poly_a, z0, up=False)
    return m


# ------------------------------------------------------------- threads ----
# A single-start helical thread. At a fixed z, walking around theta is the
# same as walking ALONG the thread axially (the helix advances one pitch per
# revolution), so the cross-section at any height is just the axial thread
# profile wrapped into polar form. That makes a thread expressible as a
# revolve() profile_fn and needs no new machinery.
#
# Symmetric trapezoid, not a sharp V: 20% crest, 30% flank, 20% root, 30%
# flank. The DEPTH is tied to the flank width, because that ratio IS the
# flank angle -- a flank that drops `depth` of radius over `RAMP * pitch` of
# axial run sits at atan(depth / (RAMP*pitch)) off vertical, and past 45 the
# printer is laying it on air.
#
# Getting this wrong is easy and silent: the first version here used the
# textbook depth of 0.54 * pitch with a 20% flank, which is 69.7 degrees.
# It printed on paper and the overhang audit caught it. depth defaults to
# RAMP * pitch now, i.e. exactly 45, and a caller asking for more gets a
# shallower angle only by widening the flank.
THREAD_RAMP = 0.30          # flank width as a fraction of one revolution


def thread_profile_fn(major_d, pitch, depth, seg=64, clearance=0.0, phase=0.0):
    """profile_fn(z) for revolve(): one single-start trapezoidal thread."""
    r_maj = major_d / 2.0 + clearance
    r_min = r_maj - depth
    R = THREAD_RAMP
    c0 = 0.5 - R / 2.0                 # crest ends
    c1 = c0 + R                        # falling flank ends
    c2 = 1.0 - R                       # root ends

    def r_at(u):
        u = u % 1.0
        if u < c0:                         # crest
            return r_maj
        if u < c1:                         # falling flank
            return r_maj - depth * (u - c0) / R
        if u < c2:                         # root
            return r_min
        return r_min + depth * (u - c2) / R    # rising flank

    def fn(z):
        pts = []
        for i in range(seg):
            th = 2.0 * math.pi * i / seg
            r = r_at(th / (2.0 * math.pi) - z / pitch + phase)
            pts.append((r * math.cos(th), r * math.sin(th)))
        return Polygon(pts)

    return fn


def thread(major_d, pitch, z0, z1, depth=None, seg=64, per_turn=24,
           clearance=0.0):
    """Solid external thread between z0 and z1. Use `clearance` NEGATIVE on a
    screw (or positive on the matching bore) to set the running fit."""
    depth = THREAD_RAMP * pitch if depth is None else depth   # 45 deg flank
    steps = max(8, int(per_turn * (z1 - z0) / pitch))
    return revolve(thread_profile_fn(major_d, pitch, depth, seg, clearance),
                   z0, z1, steps, seg=seg)


def threaded_bore(outer_d, major_d, pitch, z0, z1, depth=None, seg=64,
                  per_turn=24, clearance=0.0):
    """A plain cylinder of `outer_d` with a THREADED hole up the middle, as
    one watertight annular shell.

    Built rather than subtracted: the kernel does no 3D CSG (cad/README.md),
    so the outer wall, the helical inner wall and the two annular caps are
    each generated directly and stitched. The inner ring is wound backwards
    so its normals face into the bore.
    """
    depth = THREAD_RAMP * pitch if depth is None else depth   # 45 deg flank
    steps = max(8, int(per_turn * (z1 - z0) / pitch))
    fn = thread_profile_fn(major_d, pitch, depth, seg, clearance)
    zs = np.linspace(z0, z1, steps + 1)
    outer = [resample_ring(circle(outer_d, seg), seg) for _ in zs]
    inner = [resample_ring(fn(z), seg) for z in zs]

    m = Mesh(weld=True)
    for i in range(steps):
        oa, _ = _rings(outer[i])
        ob, _ = _rings(outer[i + 1])
        m.add_loft_wall(oa, zs[i], ob, zs[i + 1])
        ia, _ = _rings(inner[i])
        ib, _ = _rings(inner[i + 1])
        m.add_loft_wall(ia[::-1], zs[i], ib[::-1], zs[i + 1])
    m.add_cap(Polygon(_rings(outer[-1])[0], [_rings(inner[-1])[0]]), zs[-1], up=True)
    m.add_cap(Polygon(_rings(outer[0])[0], [_rings(inner[0])[0]]), zs[0], up=False)
    return m


def revolve(profile_fn, z0, z1, steps, seg=64):
    """Stack of thin lofted bands between z0 and z1, where profile_fn(z)
    returns the (hole-free) cross-section at height z. Every band is resampled
    to `seg` points so the walls stitch. Use for domes, cones, lamp shades."""
    zs = np.linspace(z0, z1, steps + 1)
    rings = [resample_ring(profile_fn(z), seg) for z in zs]
    m = Mesh(weld=True)
    for i in range(steps):
        ea, _ = _rings(rings[i])
        eb, _ = _rings(rings[i + 1])
        m.add_loft_wall(ea, zs[i], eb, zs[i + 1])
    m.add_cap(rings[-1], zs[-1], up=True)
    m.add_cap(rings[0], zs[0], up=False)
    return m

# ----------------------------------------------------------- validation ----

def validate(mesh):
    """Split into connected shells; each must be closed, edge-manifold,
    consistently wound, with positive volume. Returns a report dict."""
    V, F = mesh._np()
    report = {"vertices": len(V), "triangles": len(F), "shells": 0,
              "watertight": True, "problems": []}
    if len(F) == 0:
        report["watertight"] = False
        report["problems"].append("empty mesh")
        return report

    edges = {}
    for fi, (a, b, c) in enumerate(F):
        if a == b or b == c or a == c:
            report["problems"].append(f"degenerate face {fi}")
            continue
        for e in ((a, b), (b, c), (c, a)):
            edges.setdefault(e, 0)
            edges[e] += 1

    parent = list(range(len(F)))
    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    def union(x, y):
        rx, ry = find(x), find(y)
        if rx != ry:
            parent[rx] = ry
    by_edge = {}
    for fi, (a, b, c) in enumerate(F):
        for e in ((a, b), (b, c), (c, a)):
            k = (min(e), max(e))
            if k in by_edge:
                union(fi, by_edge[k])
            else:
                by_edge[k] = fi

    for (a, b), n in edges.items():
        if n != 1 or edges.get((b, a), 0) != 1:
            report["watertight"] = False
            report["problems"].append(
                f"edge {a}->{b} count {n}, reverse {edges.get((b, a), 0)}")
            if len(report["problems"]) > 12:
                report["problems"].append("... (truncated)")
                return report

    shells = {}
    for fi in range(len(F)):
        shells.setdefault(find(fi), []).append(fi)
    report["shells"] = len(shells)
    vol_total = 0.0
    for faces in shells.values():
        vol = 0.0
        for fi in faces:
            a, b, c = F[fi]
            vol += np.dot(V[a], np.cross(V[b], V[c])) / 6.0
        if vol <= 0:
            report["watertight"] = False
            report["problems"].append(f"shell volume {vol:.3f} <= 0 (inverted?)")
        vol_total += vol
    report["volume_mm3"] = round(float(vol_total), 2)
    return report


def fits_build_plate(mesh, x=180.0, y=180.0, z=180.0):
    """Bambu A1 mini check. Returns (ok, (dx,dy,dz))."""
    x0, y0, z0, x1, y1, z1 = mesh.bounds()
    d = (x1 - x0, y1 - y0, z1 - z0)
    return (d[0] <= x and d[1] <= y and d[2] <= z), d

# -------------------------------------------------------------- exports ----

def _explode(mesh):
    """Per-face vertices + flat normals (for GLB display)."""
    V, F = mesh._np()
    tri = V[F]                                   # (m,3,3)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln == 0] = 1.0
    n = n / ln
    pos = tri.reshape(-1, 3).astype(np.float32)
    nrm = np.repeat(n, 3, axis=0).astype(np.float32)
    return pos, nrm


def stl_write(path, mesh):
    V, F = mesh._np()
    tri = V[F]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln == 0] = 1.0
    n = (n / ln).astype(np.float32)
    with open(path, "wb") as fh:
        # EXACTLY 80 bytes, whatever the label says. This was written as
        # a literal string plus a hand-counted 68 nulls, so renaming the
        # project from "aibo" to "hotaru" pushed the header to 82 and
        # every binary STL written after it was unreadable -- Bambu
        # Studio said "the file does not contain any geometry data",
        # because the triangle count was being read two bytes late.
        # ljust does the counting now, so the label can be anything.
        header = b"hotaru partlib"[:80].ljust(80, b"\0")
        assert len(header) == 80, len(header)
        fh.write(header)
        fh.write(struct.pack("<I", len(F)))
        tri32 = tri.astype(np.float32)
        for i in range(len(F)):
            fh.write(n[i].tobytes())
            fh.write(tri32[i].tobytes())
            fh.write(b"\0\0")


def _srgb_to_linear(c):
    c = c / 255.0
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def glb_write(path, items):
    """items: list of (name, Mesh, '#RRGGBB') -> one node+material each."""
    bin_chunk = b""
    views, accessors, meshes, nodes, materials = [], [], [], [], []

    def add_view(blob, target):
        nonlocal bin_chunk
        views.append({"buffer": 0, "byteOffset": len(bin_chunk),
                      "byteLength": len(blob), "target": target})
        bin_chunk += blob + b"\0" * (-len(blob) % 4)
        return len(views) - 1

    for i, (name, mesh, hexcol) in enumerate(items):
        pos, nrm = _explode(mesh)
        idx = np.arange(len(pos), dtype=np.uint32)
        vp = add_view(pos.tobytes(), 34962)
        vn = add_view(nrm.tobytes(), 34962)
        vi = add_view(idx.tobytes(), 34963)
        accessors += [
            {"bufferView": vp, "componentType": 5126, "count": len(pos), "type": "VEC3",
             "min": [float(x) for x in pos.min(axis=0)],
             "max": [float(x) for x in pos.max(axis=0)]},
            {"bufferView": vn, "componentType": 5126, "count": len(pos), "type": "VEC3"},
            {"bufferView": vi, "componentType": 5125, "count": len(idx), "type": "SCALAR"},
        ]
        rgb = [_srgb_to_linear(int(hexcol[j:j + 2], 16)) for j in (1, 3, 5)]
        materials.append({"name": f"{name}-mat", "pbrMetallicRoughness": {
            "baseColorFactor": [*rgb, 1.0], "metallicFactor": 0.05,
            "roughnessFactor": 0.55}})
        meshes.append({"name": name, "primitives": [{
            "attributes": {"POSITION": 3 * i, "NORMAL": 3 * i + 1},
            "indices": 3 * i + 2, "material": i}]})
        nodes.append({"mesh": i, "name": name})

    gltf = {"asset": {"version": "2.0", "generator": "hotaru partlib"},
            "scene": 0, "scenes": [{"nodes": list(range(len(nodes)))}],
            "nodes": nodes, "meshes": meshes, "materials": materials,
            "bufferViews": views, "accessors": accessors,
            "buffers": [{"byteLength": len(bin_chunk)}]}
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * (-len(js) % 4)
    total = 12 + 8 + len(js) + 8 + len(bin_chunk)
    with open(path, "wb") as fh:
        fh.write(struct.pack("<III", 0x46546C67, 2, total))
        fh.write(struct.pack("<II", len(js), 0x4E4F534A) + js)
        fh.write(struct.pack("<II", len(bin_chunk), 0x004E4942) + bin_chunk)

# ------------------------------------------------------------ smoke test ----

if __name__ == "__main__":
    import sys
    ok = True

    def chk(label, mesh, shells=None):
        global ok
        r = validate(mesh)
        good = r["watertight"] and (shells is None or r["shells"] == shells)
        ok &= good
        print(f"  {label:22s} shells={r['shells']:2d} tris={r['triangles']:6d} "
              f"vol={r.get('volume_mm3', 0):9.2f} watertight={r['watertight']} "
              f"{r['problems'][:2]}")

    print("partlib smoke:")
    chk("prism + hole", prism(ring2d(rounded_rect(40, 30, 6), circle(10)), 0, 5), 1)
    chk("loft", loft_solid(rounded_rect(18.2, 18.2, 2.5), 0,
                           rounded_rect(16.4, 16.4, 2.2), 7.5), 1)
    chk("slot channel", prism(slot(30, 4), 0, 3), 1)
    chk("hex nut pocket", prism(ring2d(circle(12), ngon(5.5)), 0, 4), 1)
    chk("teardrop bore", prism(ring2d(rounded_rect(20, 20, 2), teardrop(6)), 0, 10), 1)
    chk("dome (revolve)", revolve(lambda z: circle(2 * math.sqrt(max(25**2 - z**2, 1e-4))),
                                  0, 24, steps=16), 1)

    # band-split: a wall with an exact-edged window, 3 shells that fuse
    w = ring2d(rounded_rect(50, 50, 4), rounded_rect(45, 45, 3))
    win = box(-10, -30, 10, 30)
    m = Mesh()
    m += prism(w, 0, 10)                                  # solid below
    m += prism(w.difference(win), 10 - OVL, 20 + OVL)     # windowed band
    m += prism(w, 20, 30)                                 # solid above
    chk("band-split window", m)

    fit, dims = fits_build_plate(prism(rounded_rect(170, 170, 5), 0, 100))
    print(f"  build-plate check       {tuple(round(d,1) for d in dims)} fits={fit}")
    ok &= fit

    print("SMOKE", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


def banded(profile, z0, z1, openings):
    """Extrude `profile` from z0..z1 while cutting wall openings with exact
    edges -- the band-split trick, mechanised.

    openings: [(geom, oz0, oz1), ...]. Each band that carries an opening is
    stretched OVL into its solid neighbours (so the shells fuse), while the
    solid neighbours stop EXACTLY at the opening edge -- their cap faces are
    the opening's floor and ceiling. Bands at z0/z1 are not stretched past
    the part's own extent.

    The stretch is masked by the neighbour's own profile. Where two openings
    ABUT in z -- a pin relief whose ceiling is a pocket's floor -- an
    unmasked stretch would lay the lower band's solid across the upper
    opening's floor and plug it. Intersecting with the neighbour keeps the
    stretch to where both bands are solid, which is the only place fusing
    was ever needed.
    """
    marks = {z0, z1}
    for _g, a, b in openings:
        if z0 < a < z1:
            marks.add(a)
        if z0 < b < z1:
            marks.add(b)
    zs = sorted(marks)
    bands = [(a, b) for a, b in zip(zs[:-1], zs[1:]) if b - a >= 1e-6]
    cuts = []
    for a, b in bands:
        act = [g for g, oa, ob in openings if oa <= a + 1e-6 and ob >= b - 1e-6]
        cuts.append(profile.difference(unary_union(act)) if act else profile)

    m = Mesh()
    for i, (a, b) in enumerate(bands):
        cut = cuts[i]
        if cut.is_empty:
            continue
        if cut is profile:                       # solid band: never stretched
            m += prism(profile, a, b)
            continue
        m += prism(cut, a, b)
        if a > z0:                               # down into the band below
            ov = cut.intersection(cuts[i - 1])
            if not ov.is_empty:
                m += prism(ov, a - OVL, min(a + OVL, b))
        if b < z1:                               # up into the band above
            ov = cut.intersection(cuts[i + 1])
            if not ov.is_empty:
                m += prism(ov, max(b - OVL, a), b + OVL)
    return m


def revolve_shell(z0, z1, od_fn, wall, steps=24, seg=128):
    """A hollow surface of revolution as ONE watertight shell.

    od_fn(z) -> outer diameter. Outer and inner loft walls plus two ring caps,
    so a curved silhouette comes out smooth instead of stepped. Use for domed
    shoulders and cone shades; if od_fn only ever decreases with z the result
    is self-supporting by construction.
    """
    zs = np.linspace(z0, z1, steps + 1)
    outer = [circle(od_fn(z), seg) for z in zs]
    inner = [circle(od_fn(z) - 2 * wall, seg) for z in zs]
    m = Mesh(weld=True)
    for i in range(steps):
        m.add_loft_wall(_rings(outer[i])[0], zs[i], _rings(outer[i + 1])[0], zs[i + 1])
        m.add_loft_wall(_rings(inner[i])[0][::-1], zs[i],
                        _rings(inner[i + 1])[0][::-1], zs[i + 1])
    m.add_cap(ring2d(outer[-1], inner[-1]), zs[-1], up=True)
    m.add_cap(ring2d(outer[0], inner[0]), zs[0], up=False)
    return m
