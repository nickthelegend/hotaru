"""h3_pan_sweep.py -- does the base turn through its whole range without
hitting anything? Checked by SolidWorks' own Interference Detection.

    python sw/h3_pan_sweep.py              # everything below
    python sw/h3_pan_sweep.py --time       # one pose, timed (smoke test)

The 12 named poses in h3_assembly only visit five pan angles. Everything
that turns with the pan (disc, tower, pinion, the whole arm) moves against
parts that do not: the tub, the lip ring, and the Cherry MX pod and keycap
standing on the lip ring's rear edge. The pod is the one fixed thing above
the turntable, so it is what a panning arm can hit.

  1. sweep     for a set of arm configurations, pan -90..+90 in 5 deg steps,
               SolidWorks interference at every step, next to the firmware
               guard's verdict. Pass: the guard never allows a pose that
               SolidWorks finds interfering, and every preset pose is clear
               at every pan angle.
  2. routes    moves that pan an arm folded back low from one side to the
               other. The straight joint-space move (what the firmware did
               before plan()) is sampled and checked in SolidWorks; then the
               route plan() picks instead, sampled the same way. Pass: every
               sample on every planned route is clear.
  3. presets   every move the button and the demo cycle make, planned and
               sampled the same way.

Writes solidworks/pan_report.json.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3 as H
import h3_assembly as A
import h3_guard as G
import swlib as L

REPORT = os.path.join(A.ROOT, "solidworks", "pan_report.json")
PANS = list(range(-90, 91, 5))

# the firmware's POSES[], in button / demo order
PRESETS = [("neutral", (0, 0, 0, 0)), ("rest", (0, 15, 105, 70)),
           ("reach", (0, 60, 40, 30)), ("look", (0, 10, 30, -70)),
           ("lean", (0, -30, 60, -30)), ("curious", (35, 45, 75, -20)),
           ("shy", (-40, -20, 110, 60))]


def arm_configs():
    """(label, (sh, el, hd)) to sweep the pan with: every preset's arm, the
    arm configurations the guard allows at SOME pan angles and refuses at
    others (the pod decides -- SolidWorks must agree where), and those it
    allows at every pan angle with the least room to spare."""
    out = [(f"preset {n}", p[1:]) for n, p in PRESETS]
    out += [("fold", (0, 120, 75)), ("fold-back", (30, -100, 0))]
    flip, tight = [], []
    for sh in range(-30, 71, 10):
        for el in range(-120, 121, 10):
            for hd in range(-75, 76, 15):
                if abs(G.shoulder_torque(sh, el, hd)) / H.GEAR_RATIO > G.TAU_MAX:
                    continue
                ok = [not G.collides(p, sh, el, hd) for p in PANS]
                if any(ok) and not all(ok):
                    flip.append((sum(ok), (sh, el, hd)))
                elif all(ok):
                    tight.append((min(G.clearance(p, sh, el, hd) for p in PANS), (sh, el, hd)))
    flip.sort()
    # spread across the list: fewest allowed pans (arm deepest over the pod)
    # to most (just grazing it)
    pick = sorted({round(i * (len(flip) - 1) / 7) for i in range(8)})
    out += [("pod-boundary", flip[i][1]) for i in pick]
    tight.sort()
    out += [("tightest", a) for _c, a in tight[:6]]
    return out


def check(lamp, pose):
    lamp.pose(pose, redraw=False)
    lamp.doc.EditRebuild3()
    ints = lamp.interferences()
    # the pan servo's horn sits in the disc's pocket and the servo body in
    # the tub by design; anything else touching is a hit
    return [[a, round(v, 2)] for a, v in ints]


def sweep(lamp, rows):
    for label, arm in arm_configs():
        for pan in PANS:
            pose = (pan,) + tuple(arm)
            ok, why = G.allowed(pose)
            hits = check(lamp, pose)
            r = dict(kind="sweep", label=label, pose=pose, guard=why,
                     clearance_model=round(G.clearance(*pose), 1), hits=hits)
            rows.append(r)
            bad = ok and hits
            print(f"{label:14s} {str(pose):24s} guard={why:9s} "
                  f"sw={'clear' if not hits else hits}{'   <-- GUARD MISSED' if bad else ''}",
                  flush=True)


def route_rows(lamp, rows, kind, name, cur, tgt, straight=False):
    """Check the straight move (straight=True) or plan()'s route, sampled
    every 5 deg of the biggest joint change, in SolidWorks."""
    legs = [tgt] if straight else G.plan(cur, tgt)
    if legs is None:
        rows.append(dict(kind=kind, label=name, start=cur, goal=tgt, route=None, hits=[]))
        print(f"{name}: plan() refuses the move")
        return
    at = tuple(cur)
    for leg in legs:
        for p in G.path_samples(at, leg, step=5.0):
            p = tuple(round(v, 2) for v in p)
            hits = check(lamp, p)
            rows.append(dict(kind=kind, label=name, pose=p, route=legs,
                             guard_model_clear=not G.collides(*p), hits=hits))
            print(f"{kind:9s} {name:26s} {str(p):40s} "
                  f"{'clear' if not hits else hits}", flush=True)
        at = tuple(leg)


# moves that swing an arm folded back low across the pod
ROUTES = [
    ("fold-back -80 -> +80", (-80, 0, -110, -45), (80, 0, -110, -45)),
    ("fold-back -60 -> +60", (-60, -10, -100, -60), (60, -10, -100, -60)),
    ("reach-back -90 -> +90", (-90, -30, -110, 0), (90, -30, -110, 0)),
    ("rest -90 -> +90", (-90, 15, 105, 70), (90, 15, 105, 70)),
]


def main():
    sw = L.SW()
    sw.close_all()
    sw.to_front()
    lamp = A.Lamp(sw)
    if "--time" in sys.argv:
        t = time.time()
        print(check(lamp, (0, 15, 105, 70)), f"{time.time() - t:.1f} s")
        return 0
    rows = []
    t0 = time.time()
    try:
        sweep(lamp, rows)
        for name, a, b in ROUTES:
            route_rows(lamp, rows, "straight", name, a, b, straight=True)
            route_rows(lamp, rows, "planned", name, a, b)
        seq = [p for _n, p in PRESETS]
        for i in range(len(seq)):
            a, b = seq[i], seq[(i + 1) % len(seq)]
            route_rows(lamp, rows, "preset", f"{PRESETS[i][0]} -> {PRESETS[(i + 1) % len(seq)][0]}", a, b)
    finally:
        json.dump(rows, open(REPORT, "w"), indent=1)
    lamp.pose(PRESETS[1][1])

    sw_rows = [r for r in rows if r["kind"] == "sweep"]
    missed = [r for r in sw_rows if r["guard"] == "ok" and r["hits"]]
    preset_hits = [r for r in sw_rows if r["label"].startswith("preset") and r["hits"]]
    straight_hits = [r for r in rows if r["kind"] == "straight" and r["hits"]]
    planned_hits = [r for r in rows if r["kind"] in ("planned", "preset") and r["hits"]]
    refused = [r for r in rows if r.get("route", 1) is None]
    print(f"\n{len(rows)} poses checked in SolidWorks in {(time.time() - t0) / 60:.1f} min")
    print(f"sweep: {len(sw_rows)} poses, guard allowed "
          f"{sum(r['guard'] == 'ok' for r in sw_rows)}, of those interfering: {len(missed)}")
    print(f"presets interfering at some pan angle: {len(preset_hits)}")
    print(f"straight moves: {len(straight_hits)} interfering samples")
    print(f"planned routes + preset moves: {len(planned_hits)} interfering samples, "
          f"{len(refused)} refused")
    return 1 if (missed or preset_hits or planned_hits) else 0


if __name__ == "__main__":
    sys.exit(main())
