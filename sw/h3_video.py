"""h3_video.py -- assets for videos/hotaru3-assembly, straight from the CAD.

    python sw/h3_video.py stl    # SolidWorks: every part file as built -> build/stl/
    python sw/h3_video.py glb    # -> assets/hotaru3.glb + assets/steps.json

The meshes are the SolidWorks parts (the tub and tower WITH their hand
cuts), placed with h3_assembly.LAYOUT and h3_hardware.ROWS. The build order
and every approach path are h3_insert.STEPS -- the ones SolidWorks checked
for interference -- plus the bolts and nuts, which go in along their own
axes / pocket mouths. Nothing in the video moves along a path the CAD has
not been asked about, except the transfers between the bench and the lamp,
which happen in free air.

World frame: SolidWorks mm, Y up. The GLB is Z up: (x, y, z) -> (x, -z, y).
"""
from __future__ import annotations

import json
import math
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3 as H
import h3_hardware as W
import h3_stl as S

ROOT = os.path.dirname(HERE)
PARTS = os.path.join(ROOT, "solidworks", "parts")
VID = os.path.join(ROOT, "videos", "hotaru3-assembly")
STLDIR = os.path.join(VID, "build", "stl")
ASSETS = os.path.join(VID, "assets")


def layout():
    import h3_assembly as A                     # pulls in pywin32; only needed here
    return A.LAYOUT


# ---------------------------------------------------------------- stl ---
def export_stls():
    import swlib as L
    sw = L.SW()
    sw.close_all()
    names = sorted({f for _i, f, _g, _m in layout()} | {p for _i, p, *_r in W.ROWS})
    os.makedirs(STLDIR, exist_ok=True)
    for n in names:
        r = sw.app.OpenDoc6(os.path.join(PARTS, n + ".SLDPRT"), 1, 1, "", 0, 0)
        doc = r[0] if isinstance(r, tuple) else r
        p = L.Part(sw, doc, n)
        p.stl(os.path.join(STLDIR, n + ".stl"))
        sw.app.CloseDoc(n + ".SLDPRT")
        print("stl", n, flush=True)


# ------------------------------------------------------------- meshes ---
def gl(v):
    v = np.asarray(v, float)
    return np.stack([v[..., 0], -v[..., 2], v[..., 1]], axis=-1)


def weld(tris):
    v = tris.reshape(-1, 3)
    key = np.round(v, 4)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    return uniq.astype(np.float32), inv.reshape(-1).astype(np.uint32)


def box(lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    c = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return np.array([[c[a], c[b], c[d]] for a, b, d in f])


def chips():
    """The material the hand cuts take away, so the video can cut it."""
    out = {}
    z0, z1 = H.SPK_Z0, H.SPK_Z0 + H.SPK_L
    y0, y1 = H.TUB_FLOOR + 10.0, H.TUB_FLOOR + 24.0
    xa, xb, xc, xd = H.SPK_X - 3.0, H.SPK_X - 0.3, H.SPK_X + H.SPK_T + 0.3, H.SPK_X + H.SPK_T + 3.0
    for nm, (r0, r1, g0, g1) in {"chip-rail-front": (z1 - 1.0, z1 + 2.0, z1 - 1.1, z1 + 0.4),
                                 "chip-rail-back": (z0 - 2.0, z0 + 1.0, z0 - 0.4, z0 + 1.1)}.items():
        solid_z = (g1, r1) if g0 <= r0 + 0.2 else (r0, g0)        # the rail behind its groove
        parts = [box((xa, y0, r0), (xb, y1, r1)), box((xc, y0, r0), (xd, y1, r1)),
                 box((xb, y0, solid_z[0]), (xc, y1, solid_z[1]))]
        out[nm] = np.concatenate(parts)
    out["chip-mic-wall"] = box((H.MIC_X - 1.1, H.MIC_Y - 8.0, -10.0), (H.MIC_X + 1.1, H.MIC_Y + 9.0, -7.8))
    tab_under = H.PAN_CASE_TOP + H.SV_TAB_Z0
    out["chip-pillar-groove"] = box((31.1, 13.0, -3.2), (33.6, tab_under, 3.2))
    return out


COLOR = {
    "h3-tub": "#E9E5DC", "h3-lipring": "#3A3D44", "h3-disc": "#E9E5DC", "h3-tower": "#E9E5DC",
    "h3-tower-keeper": "#E9E5DC", "h3-pinion": "#3F7BEF", "h3-link1": "#F1EEE7",
    "h3-link1-cover": "#E4E0D7", "h3-link2": "#F1EEE7", "h3-link2-cover": "#E4E0D7",
    "h3-head": "#F1EEE7", "h3-head-cover": "#E4E0D7", "h3-shade": "#F1EEE7",
    "h3-ledclamp": "#3A3D44", "h3-keycap": "#3F7BEF", "h3-pin-shoulder": "#4A4E57",
    "h3-pin-elbow": "#4A4E57", "h3-pin-head": "#4A4E57",
    "MG996R": "#2E3036", "Horn": "#55595F", "HornTrim": "#55595F", "PCA9685": "#2850A8",
    "ESP32-S3": "#16704A", "MAX98357A": "#6B3796", "Speaker2040": "#26272A",
    "INMP441": "#6B3796", "Cap2200uF": "#27508F", "CherryMX": "#D9D9D9",
    "WS2812-ring": "#FFC85C", "M3x8": "#B9BEC6", "M3x10": "#B9BEC6", "M3x12": "#B9BEC6",
    "M3x16": "#B9BEC6", "M3-nut": "#9AA0A9", "chip": "#E9E5DC",
}


def rgb(h):
    h = h.lstrip("#")
    return [int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]


def instances():
    """name -> (part, world tris in SolidWorks frame, group)"""
    cache = {}

    def part(p):
        if p not in cache:
            cache[p] = S.read_stl(os.path.join(STLDIR, p + ".stl"))
        return cache[p]

    out = {}
    for inst, f, g, M in layout():
        t = part(f)
        out[inst] = (f, t @ M[:3, :3].T + M[:3, 3], g)
    for i, p, g, M, _step, _into in W.ROWS:
        if i in out:                            # a bolt named like a part replaces it
            raise SystemExit(f"hardware id {i!r} is also a part instance")
        out[i] = (p, part(p) @ M[:3, :3].T + M[:3, 3], g)
    for nm, t in chips().items():
        out[nm] = ("chip", t, "base")
    return out


def write_glb(path, nodes):
    """nodes: [(name, verts float32 (n,3) GL, idx uint32, rgb)]"""
    bin_ = bytearray()
    views, accs, meshes, mats, gnodes = [], [], [], [], []

    def add(data, target):
        while len(bin_) % 4:
            bin_.append(0)
        views.append({"buffer": 0, "byteOffset": len(bin_), "byteLength": len(data),
                      "target": target})
        bin_.extend(data)
        return len(views) - 1

    for k, (name, v, idx, col) in enumerate(nodes):
        vv = add(v.astype("<f4").tobytes(), 34962)
        accs.append({"bufferView": vv, "componentType": 5126, "count": len(v), "type": "VEC3",
                     "min": v.min(0).tolist(), "max": v.max(0).tolist()})
        small = len(v) < 65536
        ii = add(idx.astype("<u2" if small else "<u4").tobytes(), 34963)
        accs.append({"bufferView": ii, "componentType": 5123 if small else 5125,
                     "count": len(idx), "type": "SCALAR"})
        mats.append({"pbrMetallicRoughness": {"baseColorFactor": col + [1.0]}})
        meshes.append({"primitives": [{"attributes": {"POSITION": 2 * k}, "indices": 2 * k + 1,
                                       "material": k}]})
        gnodes.append({"name": name, "mesh": k})
    doc = {"asset": {"version": "2.0", "generator": "hotaru h3_video.py"},
           "scene": 0, "scenes": [{"nodes": list(range(len(gnodes)))}], "nodes": gnodes,
           "meshes": meshes, "materials": mats, "accessors": accs, "bufferViews": views,
           "buffers": [{"byteLength": len(bin_)}]}
    js = json.dumps(doc, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    while len(bin_) % 4:
        bin_.append(0)
    out = struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(bin_))
    out += struct.pack("<II", len(js), 0x4E4F534A) + js
    out += struct.pack("<II", len(bin_), 0x004E4942) + bytes(bin_)
    open(path, "wb").write(out)
    return len(out)


def stamp(path, dur, end):
    """The root's length is read before any script runs, so the HTML has to
    carry the numbers steps.json arrived at."""
    import re
    s = open(path, encoding="utf-8").read()
    for el in ("root", "bg", "scene", "hud", "dipwrap"):
        s = re.sub(rf'(id="{el}"[^>]*?data-duration=")[^"]*"', rf'\g<1>{dur:g}"', s, flags=re.S)
    s = re.sub(r'(id="outro"[^>]*?data-start=")[^"]*"', rf'\g<1>{end:g}"', s, flags=re.S)
    s = re.sub(r'(id="outro"[^>]*?data-duration=")[^"]*"', rf'\g<1>{dur - end:g}"', s, flags=re.S)
    c = round(dur - 168.0, 3)
    s = re.sub(r'(id="bgm-c"[^>]*?data-duration=")[^"]*"', rf'\g<1>{c:g}"', s, flags=re.S)
    q = r'(?:"|&quot;)'                  # Studio rewrites the attribute entity-encoded
    s, n = re.subn(rf'(id="bgm-c".*?\{{{q}t{q}:2,{q}v{q}:1\}},\{{{q}t{q}:)[^,]*(,{q}v{q}:1\}},\{{{q}t{q}:)'
                   rf'[^,]*(,{q}v{q}:0\}})', rf'\g<1>{c - 3:g}\g<2>{c:g}\g<3>', s, count=1, flags=re.S)
    if not n:
        print("stamp: could not find bgm-c's fade-out; check it by hand")
    open(path, "w", encoding="utf-8", newline="\n").write(s)


if __name__ == "__main__":
    if sys.argv[1:] == ["stl"]:
        export_stls()
    elif sys.argv[1:] == ["glb"]:
        import h3_video_steps as VS
        inst = instances()
        nodes = []
        boxes = {}
        for name, (p, t, g) in inst.items():
            v, idx = weld(gl(t))
            nodes.append((name, v, idx, rgb(COLOR[p])))
            boxes[name] = [v.min(0).tolist(), v.max(0).tolist()]
        n = write_glb(os.path.join(ASSETS, "hotaru3.glb"), nodes)
        print(f"hotaru3.glb  {len(nodes)} nodes  {n / 1e6:.2f} MB")
        steps = VS.build(inst, boxes)
        moved = {m["n"] for m in steps["moves"]}
        missing = sorted(set(inst) - moved)
        if missing:
            raise SystemExit(f"never placed in the video: {missing}")
        json.dump(steps, open(os.path.join(ASSETS, "steps.json"), "w"), separators=(",", ":"))
        print(f"steps.json  {len(steps['steps'])} steps  {steps['duration']:.1f} s")
        stamp(os.path.join(VID, "index.html"), steps["duration"], steps["body_end"])
    else:
        print(__doc__)
