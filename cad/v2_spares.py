"""v2_spares.py -- a plate of nothing but printed thumb-bolts.

The heads are 11 mm discs only 2.6 mm tall sitting flat on the build plate,
and the neck above them is 5.4 mm. That is a lever: the head grips the
plate, and prising it off snaps it at the neck. So you want spares, and you
want them by the dozen rather than reprinting the 22-bolt tray that lives
on plate 4 next to the cone.

The bolt itself comes from v2_parts.v2_bolt_one -- the same geometry the
robot's own tray prints, not a second copy of the recipe.

    .venv/bin/python cad/v2_spares.py            # fill the plate
    .venv/bin/python cad/v2_spares.py --count 60
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import partlib as pl
import v2_parts as V

EXP = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "exports", "v2")

# The exporter's BED - 12 rule is about whether a plate FITS. It does not
# model the P1S's exclusion zone -- the corner the printer reserves for
# purging -- so a 235 mm square passed every check here and then Bambu
# Studio refused it: "Object_1 is too close to exclusion area, there may be
# collisions when printing." A tray is the one part big enough in BOTH axes
# to reach that corner, so it gets the tighter number.
USABLE = 200.0
PITCH_X, PITCH_Y = V.BOLT_PITCH


def grid_capacity():
    """How many columns and rows of bolts fit the usable bed."""
    cols = int((USABLE - V.PB_HEAD) // PITCH_X) + 1
    rows = int((USABLE - V.PB_HEAD) // PITCH_Y) + 1
    return cols, rows


def tray(count):
    cols, rows = grid_capacity()
    cap = cols * rows
    if count > cap:
        raise SystemExit(f"{count} will not fit: the bed holds {cap} "
                         f"({cols} x {rows}) at a {PITCH_X} x {PITCH_Y} pitch")
    m = pl.Mesh()
    for i in range(count):
        q = V.v2_bolt_one()
        q.translate(dx=(i % cols) * PITCH_X, dy=(i // cols) * PITCH_Y)
        m += q
    return m


def main():
    cols, rows = grid_capacity()
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=cols * rows,
                    help=f"how many bolts (default: fill the plate, {cols*rows})")
    ap.add_argument("--out", default=os.path.join(EXP, "v2-bolts-spares.stl"))
    a = ap.parse_args()

    print("HOTARU 2.0 -- printed thumb-bolt spares")
    print("=" * 58)
    print(f"  bed {V.BED:.0f}, usable {USABLE:.0f} at a "
          f"{PITCH_X:.0f} x {PITCH_Y:.0f} pitch -> {cols} x {rows} = "
          f"{cols*rows} bolts max")

    m = tray(a.count)
    b = m.bounds()
    w, d, h = b[3] - b[0], b[4] - b[1], b[5] - b[2]
    print(f"  building {a.count} bolts ... {w:.1f} x {d:.1f} x {h:.1f} mm, "
          f"{len(m.F)} triangles")
    if w > USABLE or d > USABLE:
        raise SystemExit(f"tray {w:.1f} x {d:.1f} exceeds the usable bed")

    # A tray is SUPPOSED to be N loose solids -- that is the one place the
    # one-piece rule is inverted, so state the count rather than assume it.
    import v2_onepiece as OP
    pieces, _sizes = OP.islands(m)   # returns (count, sizes)
    print(f"  {pieces} separate solids (a tray of {a.count})")
    if pieces != a.count:
        raise SystemExit(f"expected {a.count} loose bolts, got {pieces} -- "
                         "two of them are fused, so the pitch is too tight")

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    pl.stl_write(a.out, m)
    print(f"\nwrote {os.path.relpath(a.out)}  "
          f"{os.path.getsize(a.out)/1e6:.1f} MB")
    tmf = os.path.splitext(a.out)[0] + ".3mf"
    pl.tmf_write(tmf, m, name=f"HOTARU thumb-bolts x{a.count}")
    print(f"wrote {os.path.relpath(tmf)}  "
          f"{os.path.getsize(tmf)/1e6:.1f} MB")
    print("no supports: every bolt sits head-down flat on the plate")
    return 0


if __name__ == "__main__":
    sys.exit(main())
