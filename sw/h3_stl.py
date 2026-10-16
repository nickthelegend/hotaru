"""h3_stl.py -- turn the SolidWorks STLs into print-ready files.

SolidWorks exports each part where it sits in the lamp, Y up. Slicers are Z
up and want a flat face on the bed. Each part gets the orientation it was
designed to print in (the reason is next to it), is dropped onto z = 0 and
centred, and lands in exports/v3/print/.

    python sw/h3_stl.py
"""
from __future__ import annotations

import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "exports", "v3")
DST = os.path.join(SRC, "print")
BED = 256.0          # Bambu P1S

# rows = where world X, Y, Z go. Every map below is a proper rotation.
Y_UP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])      # world Y -> Z
Y_DOWN = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])    # world Y -> -Z
X_UP = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])      # world X -> Z
X_DOWN = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])    # world X -> -Z

PLAN = {
    # part               orientation   qty  why
    "h3-tub":          (Y_UP,   1, "floor on the bed; a 45 deg cone carries the slew ring, no supports"),
    "h3-lipring":      (Y_UP,   1, "flat underside down; the lip's 45 deg underside needs no support"),
    "h3-disc":         (Y_UP,   1, "underside down; the horn pocket and nut pockets bridge over"),
    "h3-tower":        (Y_UP,   1, "base plate down; cheeks, bulkhead and back wall rise straight"),
    "h3-pinion":       (X_DOWN, 1, "hub face down, horn pocket facing up"),
    "h3-link1":        (X_DOWN, 1, "gear plate down, box walls and ledges rise from it"),
    "h3-link1-cover":  (X_UP,   1, "outer face down, nut pockets facing up"),
    "h3-link2":        (X_DOWN, 1, "hub plate down (flat 6 mm, no boss), walls rise"),
    "h3-link2-cover":  (X_UP,   1, "outer face down"),
    "h3-head":         (X_UP,   1, "-X face down: tail and walls share it; the +X wall bridges 20 mm"),
    "h3-head-cover":   (X_DOWN, 1, "inner face down, tail pad on top"),
    "h3-shade":        (Y_DOWN, 1, "mouth down; yoke arms grow off the cone wall"),
    "h3-ledclamp":     (Y_UP,   1, "flat"),
    "h3-keycap":       (Y_DOWN, 1, "top face down, stem socket up"),
    "h3-pin-shoulder": (X_DOWN, 2, "flange down"),
    "h3-pin-elbow":    (X_DOWN, 1, "flange down"),
    "h3-pin-head":     (X_DOWN, 1, "flange down"),
}


def read_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:5].lower() == b"solid" and b"facet" in data[:400]:
        raise ValueError(f"{path}: ASCII STL, expected binary")
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([
        ("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].astype(np.float64)


def write_stl(path, tris, name):
    n = len(tris)
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    nrm = np.cross(e1, e2)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.where(ln > 0, nrm / np.maximum(ln, 1e-12), 0)
    rec = np.zeros(n, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"], rec["v"] = nrm, tris
    with open(path, "wb") as f:
        f.write(name.encode()[:80].ljust(80, b" "))
        f.write(struct.pack("<I", n))
        f.write(rec.tobytes())


def signed_volume(t):
    return float(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6.0)


def main():
    os.makedirs(DST, exist_ok=True)
    rows = []
    for name, (R, qty, why) in PLAN.items():
        src = os.path.join(SRC, name + ".stl")
        if not os.path.exists(src):
            src = os.path.join(SRC, name + ".STL")
        t = read_stl(src)
        v0 = signed_volume(t)
        t = t @ R.T
        lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
        t = t - np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
        v1 = signed_volume(t)
        assert abs(v1 - v0) < 1e-6 * max(1.0, abs(v0)), "rotation must keep volume and winding"
        size = hi - lo
        fits = size[0] <= BED and size[1] <= BED and size[2] <= BED
        write_stl(os.path.join(DST, name + ".stl"), t, f"HOTARU 3 {name} x{qty}")
        rows.append((name, qty, size, fits, abs(v1) / 1000, why))
    print(f"{'part':18s} qty  {'size on the bed (mm)':24s} cm3   why")
    for name, qty, size, fits, vol, why in rows:
        print(f"{name:18s} {qty:>3d}  {size[0]:6.1f} x {size[1]:6.1f} x {size[2]:5.1f}"
              f"{'' if fits else ' TOO BIG'}  {vol:6.1f}  {why}")
    if not all(r[3] for r in rows):
        sys.exit(1)


if __name__ == "__main__":
    main()
