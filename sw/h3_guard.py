"""h3_guard.py -- the pose guard the firmware runs, in Python, so SolidWorks
can check it.

Two rules decide whether a commanded pose is allowed:
  torque     the shoulder SERVO (after the 1.8:1 gear) may not be asked to
             hold more than a third of MG996R stall -- the level a hobby
             servo holds all day without cooking;
  collision  no part of link2, the head or the shade may enter the space
             the tower and the base occupy.

Group masses and centres of mass are SolidWorks' (see h3_assembly.masses),
printed into hotaru_motion.ino by `python sw/h3_guard.py --emit`.

    python sw/h3_guard.py --validate 40   # random poses: guard vs SolidWorks
"""
from __future__ import annotations

import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3 as H

G = 9.81
STALL = 9.4 * 0.0980665                 # N.m, MG996R @ 4.8 V
TAU_MAX = STALL / 3

# group mass (g) and centre of mass (s along the link from its joint, mm);
# t is 0 for all three by symmetry. Filled in from SolidWorks by --emit.
GROUPS = {"link1": (168.2, 61.82), "link2": (171.9, 75.56), "shade": (51.5, 40.24)}

# keep-out, in the arm plane (y up, z along the arm's forward direction).
# Margins are 1 mm: SolidWorks, not this model, is the final word, and the
# validation run below is what says whether 1 mm is enough.
M = 1.0
TOWER_ZONE = dict(y_max=H.Y_P + 22.0, z_min=-60.0, z_max=32.0)
BASE_ZONE = dict(y_max=H.Y_D, r_max=H.TUB_R)      # lip ring top, tub radius
POD = dict(y_max=H.MX_TOP + 6.2 + 0.8 + 6.0, r=17.0, z=H.MX_POD_Z)   # keycap top
L2_HALF = H.LINK_W / 2 + 1.0
NOSE_R = 26.0


def _pt(y0, z0, d, a):
    return (y0 + d * math.cos(math.radians(a)), z0 + d * math.sin(math.radians(a)))


def shoulder_torque(sh, el, hd):
    """Gravity torque at the shoulder JOINT, N.m (sign: + pulls forward)."""
    a1, a2 = sh, sh + el
    a3 = a2 + hd
    m1, s1 = GROUPS["link1"]
    m2, s2 = GROUPS["link2"]
    m3, s3 = GROUPS["shade"]
    s = math.sin
    r = math.radians
    ze = H.L1 * s(r(a1))
    zh = ze + (H.L2 + H.HEAD_OFF) * s(r(a2))
    mz = m1 * s1 * s(r(a1)) + m2 * (ze + s2 * s(r(a2))) + m3 * (zh + s3 * s(r(a3)))
    return mz * 1e-3 * 1e-3 * G


def sections(sh, el, hd):
    """Cross-sections covering link2, the head and the shade: (centre y,
    centre z, axis angle, half-width). A section is a segment across the
    part's axis in the arm plane -- its two ends are the part's lowest /
    widest points in y and z, which is all the keep-out test needs."""
    a1, a2 = sh, sh + el
    a3 = a2 + hd
    ey, ez = _pt(H.Y_P, H.Z_P, H.L1, a1)
    out = []
    for k in range(0, 6):                           # link2 body
        d = 36.0 + (H.L2 - 36.0) * k / 5
        out.append((*_pt(ey, ez, d, a2), a2, L2_HALF))
    hy, hz = _pt(ey, ez, H.L2 + H.HEAD_OFF, a2)
    out.append((hy, hz, a2, NOSE_R))
    out.append((hy, hz, a2 + 90.0, NOSE_R))
    rb, rm = H.SHADE_BACK_D / 2, H.SHADE_MOUTH_D / 2
    for k in range(0, 9):                           # shade, back plate to mouth
        d = H.SHADE_BACK + H.SHADE_LEN * k / 8
        out.append((*_pt(hy, hz, d, a3), a3, rb + (rm - rb) * k / 8 + 0.5))
    return out


def clearance(pan, sh, el, hd):
    """Smallest margin (mm) between any section end and the keep-outs;
    negative means inside one."""
    worst = 1e9
    t, b, p = TOWER_ZONE, BASE_ZONE, POD
    cp = math.cos(math.radians(pan))
    for y, z, a, hw in sections(sh, el, hd):
        ny, nz = -math.sin(math.radians(a)), math.cos(math.radians(a))
        for k in (-1.0, 0.0, 1.0):
            py, pz = y + k * hw * ny, z + k * hw * nz
            # tower: a box in the arm plane (the turret pans with the arm)
            d_t = max(py - t["y_max"], t["z_min"] - pz, pz - t["z_max"])
            # base: a cylinder about the pan axis
            d_b = max(py - b["y_max"], abs(pz) - b["r_max"])
            worst = min(worst, d_t, d_b)
        # the button pod is FIXED to the base while the arm pans: distance
        # from this section's centre to the pod, in the horizontal plane
        dxz = math.sqrt(max(z * z - 2 * z * p["z"] * cp + p["z"] ** 2, 0.0))
        worst = min(worst, max(y - hw - p["y_max"], dxz - hw - p["r"]))
    return worst


def collides(pan, sh, el, hd):
    return clearance(pan, sh, el, hd) < M


def allowed(pose):
    pan, sh, el, hd = pose
    lim = {"pan": (-90, 90), "shoulder": (-30, 70), "elbow": (-120, 120), "head": (-75, 75)}
    for v, (lo, hi) in zip(pose, lim.values()):
        if not lo <= v <= hi:
            return False, "limit"
    if abs(shoulder_torque(sh, el, hd)) / H.GEAR_RATIO > TAU_MAX:
        return False, "torque"
    if collides(pan, sh, el, hd):
        return False, "collision"
    return True, "ok"


# ----------------------------------------------------------------- tools ---
def emit():
    import h3_assembly as A
    ms = A.masses()
    piv = {"link1": H.Y_P, "link2": H.Y_E, "shade": H.Y_H}
    out = {}
    for grp in piv:
        m = sy = 0.0
        for _k, (mm, com, gg, M) in ms.items():
            if gg == grp:
                p = M @ com
                m += mm
                sy += mm * p[1]
        out[grp] = (round(m, 1), round(sy / m - piv[grp], 2))
    print(json.dumps(out))
    return out


def validate(n, seed=3, record=False):
    """Random poses in the joint limits; guard verdict against SolidWorks'
    interference detection. A pose the guard ALLOWS that SolidWorks finds
    interfering is a guard failure; the reverse is only conservatism."""
    import h3_assembly as A
    import swlib as L
    rnd = random.Random(seed)
    GROUPS.update(emit())              # this build's SolidWorks masses
    sw = L.SW()
    sw.close_all()
    sw.to_front()
    if record:
        sw.start_frames(os.path.join(A.REC, "frames"))
    rows = []
    try:
        lamp = A.Lamp(sw)
        for i in range(n):
            pose = (rnd.uniform(-90, 90), rnd.uniform(-30, 70), rnd.uniform(-120, 120),
                    rnd.uniform(-75, 75))
            pose = tuple(round(v) for v in pose)
            ok, why = allowed(pose)
            lamp.pose(pose)
            ints = lamp.interferences()
            sw.frame()
            rows.append(dict(pose=pose, guard=why, sw_clear=not ints,
                             hits=[[a, round(v, 1)] for a, v in ints]))
            tag = ("PASS" if (not ok or not ints) else "GUARD MISSED IT")
            print(f"{i:3d} {str(pose):22s} guard={why:9s} solidworks="
                  f"{'clear' if not ints else str(len(ints)) + ' hits'}  {tag}", flush=True)
    finally:
        pass
    missed = [r for r in rows if r["guard"] == "ok" and not r["sw_clear"]]
    allowed_n = sum(r["guard"] == "ok" for r in rows)
    print(f"\n{allowed_n}/{n} allowed by the guard, all {allowed_n - len(missed)} of those "
          f"clear in SolidWorks; {len(missed)} missed")
    json.dump(rows, open(os.path.join(A.ROOT, "solidworks", "guard_report.json"), "w"), indent=1)
    return missed


if __name__ == "__main__":
    if "--emit" in sys.argv:
        emit()
    elif "--validate" in sys.argv:
        k = sys.argv.index("--validate")
        n = int(sys.argv[k + 1]) if len(sys.argv) > k + 1 else 30
        sys.exit(1 if validate(n) else 0)
    else:
        for name, pose in (("rest", (0, 15, 105, 70)), ("reach", (0, 60, 40, 30)),
                           ("full-reach", (0, 70, 0, 0)), ("old fold-back", (0, 30, -120, -75))):
            print(f"{name:14s} {pose}  {allowed(pose)}  "
                  f"servo {abs(shoulder_torque(*pose[1:])) / H.GEAR_RATIO:.3f} N.m")
