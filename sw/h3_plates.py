"""h3_plates.py -- HOTARU 3 print plates for a Bambu P1S (256 x 256 x 256).

    python sw/h3_plates.py        # -> exports/v3/plates/

Takes the print-oriented STLs from h3_stl.py, lays each plate out on the bed
and writes, per plate:
  plate-N-<name>.3mf   separate objects, already placed -- open it in Bambu
                       Studio / OrcaSlicer and slice
  plate-N-<name>.stl   the same plate as one mesh, for other slicers
  plate-N-<name>.svg   top-down preview of the layout

Layout: each part's footprint (convex hull of its bed-contact projection)
is placed greedily, largest first, lowest-then-leftmost position that keeps
a GAP to every other part and MARGIN to the bed edge, trying 0 and 90 deg.
The finished layout is centred on the bed.
"""
from __future__ import annotations

import html
import os
import sys
import zipfile

import numpy as np
from shapely import affinity
from shapely.geometry import MultiPoint, box
from shapely.prepared import prep

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3_stl as S

BED = 256.0
MARGIN = 5.0
GAP = 6.0
STEP = 2.0

# Grouped by what prints together well -- and by filament: plate 2 is every
# part the model colours dark, so it can go on in a second colour.
PLATES = [
    ("tub", "light", [("h3-tub", 1)]),
    ("lipring-and-pins", "dark", [("h3-lipring", 1), ("h3-ledclamp", 1),
                                  ("h3-pin-shoulder", 2), ("h3-pin-elbow", 1),
                                  ("h3-pin-head", 1)]),
    ("turntable", "light", [("h3-disc", 1), ("h3-tower", 1)]),
    ("arm", "light", [("h3-link1", 1), ("h3-link1-cover", 1), ("h3-link2", 1),
                      ("h3-link2-cover", 1)]),
    ("head-and-shade", "light (+ accent)", [("h3-shade", 1), ("h3-head", 1),
                                            ("h3-head-cover", 1), ("h3-pinion", 1),
                                            ("h3-keycap", 1)]),
]
ACCENT = {"h3-pinion", "h3-keycap"}


def load(name):
    return S.read_stl(os.path.join(S.DST, name + ".stl"))


def footprint(tris):
    pts = tris.reshape(-1, 3)[:, :2]
    return MultiPoint([tuple(p) for p in np.unique(np.round(pts, 2), axis=0)]).convex_hull


def place(items):
    """items: [(name, tris)] -> [(name, tris placed in bed coords, hull)]"""
    order = sorted(items, key=lambda it: -footprint(it[1]).area)
    placed, hulls = [], []
    bed_in = box(MARGIN, MARGIN, BED - MARGIN, BED - MARGIN)
    for name, tris in order:
        best = None
        for rot in (0, 90):
            t = tris.copy()
            if rot:
                t = t @ np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float).T
            h = footprint(t)
            x0, y0, x1, y1 = h.bounds
            w, d = x1 - x0, y1 - y0
            if w > BED - 2 * MARGIN or d > BED - 2 * MARGIN:
                continue
            others = prep(hulls[0].union_all() if False else
                          __import__("shapely").union_all([g.buffer(GAP) for g in hulls])) \
                if hulls else None
            y = MARGIN
            found = None
            while y + d <= BED - MARGIN + 1e-6 and found is None:
                x = MARGIN
                while x + w <= BED - MARGIN + 1e-6:
                    cand = affinity.translate(h, x - x0, y - y0)
                    if bed_in.contains(cand) and (others is None or not others.intersects(cand)):
                        found = (x - x0, y - y0, cand)
                        break
                    x += STEP
                y += STEP
            if found and (best is None or (found[1], found[0]) < (best[1], best[0])):
                best = (found[0], found[1], found[2], t)
        if best is None:
            raise RuntimeError(f"{name} does not fit on this plate")
        dx, dy, cand, t = best
        placed.append((name, t + np.array([dx, dy, 0.0]), cand))
        hulls.append(cand)
    # centre the whole layout on the bed
    allb = np.array([h.bounds for _n, _t, h in placed])
    cx = (BED - (allb[:, 2].max() + allb[:, 0].min())) / 2
    cy = (BED - (allb[:, 3].max() + allb[:, 1].min())) / 2
    return [(n, t + np.array([cx, cy, 0.0]), affinity.translate(h, cx, cy))
            for n, t, h in placed]


def indexed(tris):
    v = tris.reshape(-1, 3)
    uniq, inv = np.unique(np.round(v, 4), axis=0, return_inverse=True)
    return uniq, inv.reshape(-1, 3)


def write_3mf(path, title, objs):
    res, build = [], []
    for i, (name, tris) in enumerate(objs, start=1):
        V, F = indexed(tris)
        vs = "".join(f'<vertex x="{x:.4f}" y="{y:.4f}" z="{z:.4f}"/>' for x, y, z in V)
        fs = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in F)
        res.append(f'<object id="{i}" type="model" name="{html.escape(name)}"><mesh>'
                   f'<vertices>{vs}</vertices><triangles>{fs}</triangles></mesh></object>')
        build.append(f'<item objectid="{i}"/>')
    model = ('<?xml version="1.0" encoding="UTF-8"?>'
             '<model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             f'<metadata name="Title">{html.escape(title)}</metadata>'
             '<metadata name="Designer">HOTARU 3</metadata>'
             f'<resources>{"".join(res)}</resources><build>{"".join(build)}</build></model>')
    ct = ('<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/'
          'package/2006/content-types"><Default Extension="rels" ContentType="application/'
          'vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" '
          'ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.'
            'openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" '
            'Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>'
            '</Relationships>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", ct)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", model)


def write_svg(path, title, placed):
    s = 2.5
    W = int(BED * s)
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{W + 34}" '
           f'viewBox="0 0 {W} {W + 34}" font-family="sans-serif">',
           f'<rect width="{W}" height="{W + 34}" fill="#f4f3ef"/>',
           f'<text x="10" y="23" font-size="17" fill="#222">{html.escape(title)}</text>',
           f'<rect x="0.5" y="34.5" width="{W - 1}" height="{W - 1}" fill="#fff" stroke="#999"/>']
    for name, _t, h in placed:
        pts = " ".join(f"{x * s:.1f},{34 + (BED - y) * s:.1f}" for x, y in h.exterior.coords)
        fill = "#2C6BED" if name in ACCENT else ("#3a3d42" if "dark" in title.lower() else "#d9d6cc")
        out.append(f'<polygon points="{pts}" fill="{fill}" fill-opacity="0.85" stroke="#333"/>')
        c = h.centroid
        tc = "#fff" if fill != "#d9d6cc" else "#222"
        out.append(f'<text x="{c.x * s:.1f}" y="{34 + (BED - c.y) * s:.1f}" font-size="12" '
                   f'text-anchor="middle" fill="{tc}">{html.escape(name.replace("h3-", ""))}</text>')
    out.append("</svg>")
    open(path, "w", encoding="utf-8").write("\n".join(out))


def main():
    dst = os.path.join(os.path.dirname(S.DST), "plates")
    os.makedirs(dst, exist_ok=True)
    total = {}
    lines = []
    for k, (pname, colour, parts) in enumerate(PLATES, start=1):
        items = []
        for name, qty in parts:
            t = load(name)
            for q in range(qty):
                items.append((name if qty == 1 else f"{name}-{q + 1}", t))
                total[name] = total.get(name, 0) + 1
        placed = place(items)
        stem = f"plate-{k}-{pname}"
        title = f"HOTARU 3 - plate {k}: {pname} ({colour})"
        write_3mf(os.path.join(dst, stem + ".3mf"), title, [(n, t) for n, t, _h in placed])
        S.write_stl(os.path.join(dst, stem + ".stl"), np.concatenate([t for _n, t, _h in placed]),
                    title)
        write_svg(os.path.join(dst, stem + ".svg"), title, placed)
        allv = np.concatenate([t.reshape(-1, 3) for _n, t, _h in placed])
        lo, hi = allv.min(0), allv.max(0)
        assert lo[0] >= MARGIN - 0.01 and lo[1] >= MARGIN - 0.01 and hi[0] <= BED - MARGIN + 0.01 \
            and hi[1] <= BED - MARGIN + 0.01 and hi[2] <= BED and abs(lo[2]) < 1e-3
        for i, (_a, _b, ha) in enumerate(placed):
            for _c, _d, hb in placed[i + 1:]:
                assert ha.distance(hb) >= GAP - 0.01, (stem, _a, _c)
        lines.append((k, pname, colour, [n for n, _t, _h in placed],
                      (hi[0] - lo[0], hi[1] - lo[1], hi[2])))
        print(f"plate {k} {pname:18s} {colour:17s} {len(placed)} parts, "
              f"{hi[0] - lo[0]:.0f} x {hi[1] - lo[1]:.0f} x {hi[2]:.0f} mm  "
              f"({', '.join(n.replace('h3-', '') for n, _t, _h in placed)})")
    want = {n: q for _p, _c, parts in PLATES for n, q in parts}
    assert set(want) == set(S.PLAN), "every printed part on exactly one plate"
    assert all(total[n] == S.PLAN[n][1] for n in S.PLAN), "quantities match the BOM"
    zp = os.path.join(dst, "hotaru3-plates.zip")
    with zipfile.ZipFile(zp, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(os.listdir(dst)):
            if f.endswith((".3mf", ".stl")):
                z.write(os.path.join(dst, f), f)
    print("zip:", zp)
    return lines


if __name__ == "__main__":
    main()
