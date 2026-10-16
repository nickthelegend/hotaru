"""h3_overhang.py -- will this part print WITHOUT supports? Checked, not hoped.

The v3 parts were published as "no supports" and the tub printed with a
forest of tree supports: its slew ring jutted 16.5 mm straight in from the
wall. Nothing had measured overhangs. This does, on the print-oriented STLs:

  * every downward-facing triangle steeper than MAX_ANGLE from vertical,
    not on the bed, is overhang;
  * overhang triangles are merged into regions (projected onto the bed);
  * a region is fine if it is NARROW: its widest inscribed circle is at most
    BRIDGE_OK across -- nut-slot ceilings, small holes, counterbores, short
    bridges, which a printer spans cleanly. Anything wider is reported.

    python sw/h3_overhang.py            # all parts, exit 1 if any fail
    python sw/h3_overhang.py h3-tub
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
from shapely.ops import polylabel
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3_stl as S

MAX_ANGLE = 45.0       # overhang allowed, degrees from vertical
BRIDGE_OK = 10.0       # mm: widest inscribed circle an overhang region may have
BED_Z = 0.25           # first layer


def regions(tris):
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    n = np.cross(e1, e2)
    ln = np.linalg.norm(n, axis=1)
    ok = ln > 1e-9
    nz = np.where(ok, n[:, 2] / np.maximum(ln, 1e-12), 0.0)
    # a face tilted more than MAX_ANGLE from vertical, facing down
    lim = -math.sin(math.radians(90.0 - MAX_ANGLE)) + 1e-6
    zmin = tris[:, :, 2].min(axis=1)
    bad = ok & (nz < lim) & (zmin > BED_Z)
    polys = [Polygon(t[:, :2]).buffer(0.05) for t in tris[bad]
             if Polygon(t[:, :2]).area > 1e-6]
    if not polys:
        return []
    merged = unary_union(polys)
    out = []
    for g in getattr(merged, "geoms", [merged]):
        if g.area < 0.5:
            continue
        c = polylabel(g, 0.1)
        w = 2 * g.boundary.distance(c)
        out.append(dict(area=g.area, width=w, at=(c.x, c.y)))
    return out


def check(name):
    t = S.read_stl(os.path.join(S.DST, name + ".stl"))
    rs = regions(t)
    return rs, [r for r in rs if r["width"] > BRIDGE_OK]


def main(names):
    any_fail = False
    for n in names:
        rs, fails = check(n)
        if fails:
            any_fail = True
            worst = max(fails, key=lambda r: r["width"])
            print(f"NEEDS SUPPORT  {n:18s} {len(fails)} region(s); worst {worst['width']:.1f} mm "
                  f"across, {worst['area']:.0f} mm2 at ({worst['at'][0]:.0f}, {worst['at'][1]:.0f})")
        else:
            small = max((r["width"] for r in rs), default=0)
            print(f"ok             {n:18s} {len(rs)} small overhang(s), widest {small:.1f} mm")
    return any_fail


if __name__ == "__main__":
    names = sys.argv[1:] or list(S.PLAN)
    sys.exit(1 if main(names) else 0)
