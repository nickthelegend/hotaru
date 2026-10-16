"""h3_hardware.py -- every M3 bolt and nut in HOTARU 3, and where it goes.

No heat-set inserts anywhere. A bolt either runs into a hex nut sitting in
a printed pocket, or cuts its own thread into a 2.6 mm pilot hole in solid
PLA (pan servo -> tub pillars, lip ring -> tub), or into a servo's tapped
output shaft (pinion, link2 hub, shade hub).

Positions come from the same numbers the parts are cut from (h3.py, and the
pocket depths in hotaru3.py), in SolidWorks world mm, Y up, neutral pose.
Each row belongs to the joint group it moves with, so h3_assembly's group
transforms pose it like the part it holds.

    python sw/h3_hardware.py      # print the list and the totals
"""
from __future__ import annotations

import math
import os
import sys
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3 as H

X, Y, Z = np.eye(3)
HALF_NUT = H.M3_NUT_H / 2


def _bolt_frame(p, axis):
    """Local +Z (head underside -> tip) onto `axis`, origin at the head's
    underside."""
    z = np.asarray(axis, float)
    ref = Y if abs(z @ Y) < 0.9 else X
    x = np.cross(ref, z)
    x /= np.linalg.norm(x)
    M = np.eye(4)
    M[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    M[:3, 3] = p
    return M


def _nut_frame(p, plane, rot=0.0):
    """A nut lying in a sketch plane whose (u, v) axes are `plane`, turned
    like a pocket drawn with Sketch.hexagon(..., rot) or a nut_slot."""
    U, V = plane
    a = math.radians(rot)
    x = math.cos(a) * U + math.sin(a) * V
    y = -math.sin(a) * U + math.cos(a) * V
    M = np.eye(4)
    M[:3, :3] = np.column_stack([x, y, np.cross(U, V)])
    M[:3, 3] = p
    return M


WXZ = (X, Z)          # "wXZ" sketches: u = world x, v = world z
WYZ = (Y, Z)          # "wYZ" and every link frame: u = world y, v = world z

# (id, part, group, local->world, step key, threads into)
ROWS = []
ENTRY = {}      # nut id -> unit vector it slides in FROM (pocket mouth / slot)


def bolt(i, length, group, p, axis, step, into):
    ROWS.append((i, f"M3x{length}", group, _bolt_frame(p, axis), step, into))


def nut(i, group, p, plane, step, rot=0.0, entry=None):
    ROWS.append((i, "M3-nut", group, _nut_frame(p, plane, rot), step, None))
    ENTRY[i] = np.asarray(entry, float)


def _build():
    ROWS.clear()
    YP, ZP, YE, YH = H.Y_P, H.Z_P, H.Y_E, H.Y_H
    pocket = H.NUT_POCKET_H

    # pan servo tabs -> pilot holes in the tub pillars (no nuts)
    y = H.PAN_CASE_TOP + H.SV_TAB_Z1
    for k, (hx, hy) in enumerate(H.SV_HOLES):
        bolt(f"pan-tab-{k}", 10, "base", (hx, y, -hy), -Y, "pan-servo", "h3-tub")

    # lip ring -> pilot holes in the tub's cone
    for k, a in enumerate(H.RING_BOLT_ANG):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        bolt(f"ring-{k}", 12, "base", (H.RING_BOLT_R * c, H.Y_D, H.RING_BOLT_R * s), -Y,
             "lipring", "h3-tub")

    # tower base plate -> disc; nuts in the disc's underside pockets
    for k, (x, z) in enumerate(H.TOWER_BOLTS):
        bolt(f"tower-{k}", 12, "turret", (x, H.TOWER_Y1, z), -Y, "tower", "nut")
        nut(f"tower-nut-{k}", "turret", (x, H.Y_T + pocket - HALF_NUT, z), WXZ, "tower-nuts",
            entry=-Y)

    # shoulder servo tabs -> bulkhead (lower pair) and the keeper (upper pair)
    bx1 = H.SH_BULK_X[1]
    for k, (hx, hy) in enumerate(H.SV_HOLES):
        y, z = YP - hx, H.SH_SHAFT_Z - hy
        upper = y > H.Y_P
        step = "keeper-bolts" if upper else "shoulder-tabs"
        bolt(f"sh-tab-{k}", 10, "turret", (H.SH_CASE_TOP + H.SV_TAB_Z0, y, z), X, step, "nut")
        nut(f"sh-tab-nut-{k}", "turret", (bx1 - pocket + HALF_NUT, y, z), WYZ,
            "keeper-nuts" if upper else "shoulder-nuts", entry=X)

    # pinion -> the shoulder servo's tapped shaft
    bolt("pinion-bolt", 10, "pinion", (H.PINION_X[1] - 2.5, YP, H.SH_SHAFT_Z), -X,
         "shoulder-bench", "MG996R")

    # shoulder pins: A on the gear side, nut in link1; B through the cover
    xa = H.L1A[1] - 1.0 + H.SHOULDER_PIN + 0.5
    bolt("pin-sh-A", 16, "link1", (xa, YP, ZP), -X, "shoulder-pins", "nut")
    nut("pin-sh-A-nut", "link1", (H.GEAR_X[0] + pocket - HALF_NUT, YP, ZP), WYZ, "link1-nuts",
        entry=-X)
    xb = H.L1B[0] + 1.0 - H.SHOULDER_PIN - 0.5
    bolt("pin-sh-B", 16, "link1", (xb, YP, ZP), X, "shoulder-pins", "nut")
    nut("pin-sh-B-nut", "link1", (H.L1B[1] - pocket + HALF_NUT, YP, ZP), WYZ, "link1-cover-nuts",
        entry=X)

    # link1 cover -> nuts slid into link1's wall slots
    for k, (s, t) in enumerate((s, sg * 14.0) for s in (20.0, 55.0, 100.0) for sg in (-1, 1)):
        bolt(f"l1-cover-{k}", 10, "link1", (H.L1B[0] + H.M3_HEAD_H, YP + s, ZP + t), X,
             "link1-cover", "nut")
        nut(f"l1-cover-nut-{k}", "link1", (-18.4 + HALF_NUT, YP + s, ZP + t), WYZ, "link1-nuts",
            entry=-math.copysign(1, t) * Z)

    # elbow servo tabs -> nuts in link1's ledge slots
    lx0 = H.EL_CASE_TOP + H.SV_TAB_Z1
    for k, (s, t) in enumerate((s, t) for s in (H.L1 - 35.1, H.L1 + 14.4) for t in (-5.0, 5.0)):
        bolt(f"el-tab-{k}", 10, "link1", (H.EL_CASE_TOP + H.SV_TAB_Z0, YP + s, ZP + t), X,
             "elbow-servo", "nut")
        nut(f"el-tab-nut-{k}", "link1", (lx0 + 3.5 + HALF_NUT, YP + s, ZP + t), WYZ,
            "link1-nuts", rot=30.0, entry=(-1 if s < H.L1 else 1) * Y)

    # elbow pin (fixed in link1), nut in the link1 cover's far pocket
    bolt("pin-elbow-bolt", 12, "link1", (H.L1B[0] + 1.0 - H.ELBOW_PIN - 0.5, YE, ZP), X,
         "elbow-pin", "nut")
    nut("pin-elbow-nut", "link1", (H.L1B[1] - pocket + HALF_NUT, YE, ZP), WYZ, "link1-cover-nuts",
        entry=X)

    # link2 hub -> the elbow servo's tapped shaft
    bolt("l2-hub", 10, "link2", (H.L2A[1] + H.L2A_BOSS - 2.5, YE, ZP), -X, "link2", "MG996R")

    # link2 cover -> nuts in link2's wall slots
    for k, (s, t) in enumerate((s, sg * 14.0) for s in (42.0, 58.0) for sg in (-1, 1)):
        bolt(f"l2-cover-{k}", 10, "link2", (H.L2B[0] + H.M3_HEAD_H, YE + s, ZP + t), X,
             "link2-cover", "nut")
        nut(f"l2-cover-nut-{k}", "link2", (-24.9 + HALF_NUT, YE + s, ZP + t), WYZ, "link2-nuts",
            entry=-math.copysign(1, t) * Z)

    # head block <-> link2: through both plates into the head's tail block
    for k, t in enumerate((-7.0, 7.0)):
        y, z = YE + H.L2 - 18.0, ZP + t
        bolt(f"head-mount-D-{k}", 16, "link2", (H.L2A[1] - H.M3_HEAD_H, y, z), -X,
             "head-mount-D", "nut")
        nut(f"head-mount-D-nut-{k}", "link2", (21.6 - HALF_NUT, y, z), WYZ, "head-nuts",
            entry=math.copysign(1, t) * Z)
        bolt(f"head-mount-I-{k}", 16, "link2", (H.L2B[0] + H.M3_HEAD_H, y, z), X,
             "link2-cover", "nut")
        nut(f"head-mount-I-nut-{k}", "link2", (-17.5 + HALF_NUT, y, z), WYZ, "head-nuts",
            entry=math.copysign(1, t) * Z)

    # head servo tabs -> nuts in the head's ledge slots
    lx0 = H.HD_CASE_TOP + H.SV_TAB_Z1
    for k, (s, t) in enumerate((s, t) for s in (14.4, -35.1) for t in (-5.0, 5.0)):
        bolt(f"hd-tab-{k}", 10, "link2", (H.HD_CASE_TOP + H.SV_TAB_Z0, YH + s, ZP + t), X,
             "head-servo", "nut")
        nut(f"hd-tab-nut-{k}", "link2", (lx0 + 3.5 + HALF_NUT, YH + s, ZP + t), WYZ,
            "head-nuts", rot=30.0, entry=(-1 if s > 0 else 1) * Y)

    # head cover, top bolt
    bolt("head-cover-bolt", 10, "link2", (H.NOSE_B[0] + H.M3_HEAD_H, YH + 19.5, ZP), X,
         "head-cover", "nut")
    nut("head-cover-nut", "link2", (-16.4 + HALF_NUT, YH + 19.5, ZP), WYZ, "head-nuts", rot=30.0,
        entry=Y)

    # head pin, nut in the head cover's inner pocket
    bolt("pin-head-bolt", 12, "link2", (H.NOSE_B[0] + 1.0 - H.HEAD_PIN - 0.5, YH, ZP), X,
         "head-pin", "nut")
    nut("pin-head-nut", "link2", (H.NOSE_B[1] - pocket + HALF_NUT, YH, ZP), WYZ, "head-cover-nut",
        entry=X)

    # shade hub -> the head servo's tapped shaft
    bolt("shade-hub", 8, "shade", (H.YOKE_A[1] - 2.5, YH, ZP), -X, "shade-hub", "MG996R")
    return ROWS


_build()


def totals():
    return Counter(part for _i, part, *_r in ROWS)


def check():
    """Put every bolt and nut into the SolidWorks assembly at the neutral
    pose and run interference detection. A bolt in a clearance hole, a head
    on its face and a nut in its pocket touch nothing. The only overlaps
    allowed are a thread cutting into its pilot hole or a servo's tap."""
    import json
    import h3_assembly as A
    import swlib as L
    sw = L.SW()
    sw.close_all()
    rows = list(A.LAYOUT) + [(i, p, g, M) for i, p, g, M, _s, _into in ROWS]
    ids = [r[0] for r in rows]
    assert len(ids) == len(set(ids)), "a bolt named like a part would replace it"
    lamp = A.Lamp(sw, rows)
    lamp.pose(A.POSES["neutral"])
    inst_of = {c.Name2: inst for inst, (c, *_r) in lamp.comps.items()}
    into = {i: t for i, _p, _g, _M, _s, t in ROWS}
    ok, bad = [], []
    for names, vol in lamp.interferences():
        insts = [inst_of.get(n, n) for n in names]
        parts = [n.rsplit("-", 1)[0] for n in names]
        hw = [i for i in insts if i in into]
        other = [p for i, p in zip(insts, parts) if i not in into]
        if len(hw) == 1 and len(other) == 1 and into[hw[0]] == other[0]:
            ok.append((hw[0], other[0], round(vol, 2)))
        else:
            bad.append((insts, round(vol, 2)))
    for r in ok:
        print("thread  ", *r)
    for r in bad:
        print("PROBLEM ", *r)
    json.dump(dict(threads=ok, problems=bad),
              open(os.path.join(A.ROOT, "solidworks", "hardware_report.json"), "w"), indent=1)
    return not bad


if __name__ == "__main__":
    if "--check" in sys.argv:
        sys.exit(0 if check() else 1)
    for i, part, g, M, step, into in ROWS:
        p = M[:3, 3]
        print(f"{i:22s} {part:7s} {g:7s} ({p[0]:7.2f} {p[1]:7.2f} {p[2]:7.2f})  "
              f"axis {np.round(M[:3, 2], 3)}  {step:16s} {into or ''}")
    for part, n in sorted(totals().items()):
        print(f"{part:8s} x{n}")
