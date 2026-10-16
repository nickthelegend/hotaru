"""h3_insert.py -- can every part actually GET to its seat?

h3_assembly.py proves the finished lamp has no interference. It never asked
whether each part can travel there -- and on the first run of this check
the shoulder servo could not reach its place in the tower, the speaker and
mic could not get under the tub's cone, and the shade could not get its
horn pocket over the horn. The fixes are hand cuts on the printed parts
(hotaru3.HANDFIX), a different build order, and the paths below.

The build order is walked step by step. Parts not yet installed are parked
far away. The part (or subassembly) being installed starts at the far end
of its path -- a polyline of offsets from its seat, so it can come in
tilted, sideways, then down -- and is moved home in small steps, with
SolidWorks' interference detection run at every step against everything
already built. A [bench] step happens at its own spot away from the lamp,
on whatever part it names as the host.

    python sw/h3_insert.py
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3 as H
import h3_assembly as A
import swlib as L


def up(d):
    return [(0, 0, 0), (0, d, 0)]


def line(dx, dy, dz):
    return [(0, 0, 0), (dx, dy, dz)]


_c = -np.array([H.CAP_XZ[0], 0, H.CAP_XZ[1]]) / math.hypot(*H.CAP_XZ)
CAP_IN = _c * 8.5                                  # toward the tub's middle

FAR = np.array([5000.0, 0.0, 0.0])
BENCH = {"turret": (0, 0, 2000.0), "shoulder": (0, 0, 3000.0), "link1": (0, 0, 4000.0),
         "head": (0, 0, 5000.0), "shade": (0, 0, 6000.0)}

# label, parts moving together, path (offsets from the seat, seat first),
# pairs allowed to touch on the way (and why), bench spot, host parts
STEPS = [
    dict(label="capacitor into its clip, tilted in from the middle", move=["bulkcap"],
         path=[(0, 0, 0), (0, 11.5, 0), tuple(CAP_IN + (0, 12.0, 0)), tuple(CAP_IN + (0, 80.0, 0))]),
    dict(label="PCA9685 onto its pins", move=["pca9685"], path=up(40)),
    # both boards reach out under the slew-ring ledge: in from the middle, then down
    dict(label="ESP32 in from the middle, onto its posts", move=["esp32"],
         path=[(0, 0, 0), (0, 18.0, 0), (-10.0, 18.0, 0), (-10.0, 80.0, 0)]),
    dict(label="amp in from the middle, onto its posts", move=["amp"],
         path=[(0, 0, 0), (0, 6.0, 0), (10.1, 6.0, -6.45), (10.1, 80.0, -6.45)]),
    dict(label="speaker in from the middle, down the snipped rails", move=["speaker"],
         path=[(0, 0, 0), (0, 10.5, 0), (-11.0, 10.5, 0), (-11.0, 80.0, 0)]),
    dict(label="mic slides in sideways through the opened end", move=["mic"],
         path=[(0, 0, 0), (0, 0, -12.0), (0, -1.0, -15.0), (0, -3.0, -17.0), (0, -8.0, -18.0),
               (0, -12.0, -20.0), (17.0, -12.0, -20.0), (17.0, 70.0, -20.0)]),
    # the model's cable stub is a rigid 5.2 x 7 x 5 block; the real boot is
    # rubber and rides down the 2.5 mm hand-cut groove. Only the stub's own
    # overlap past the groove (2.1 x 7 x 5 plus its edges, 81 mm3) is let off.
    dict(label="pan servo between the pillars", move=["servo-pan"], path=up(60),
         allow=[("MG996R", "h3-tub", 90.0)]),
    dict(label="pan horn onto the spline", move=["horn-pan"], path=up(15)),
    dict(label="[bench] tower onto the disc", move=["tower"], path=up(40),
         bench="turret", hosts=["disc"]),
    dict(label="disc + tower onto the pan horn", move=["disc", "tower"], path=up(40)),
    dict(label="lip ring over the disc", move=["lipring"], path=up(30)),
    dict(label="MX switch snaps into the pod", move=["mx"], path=up(30),
         allow=[("CherryMX", "h3-lipring")]),       # its clips flex through the 14 mm plate hole
    dict(label="keycap onto the switch", move=["keycap"], path=up(20)),
    dict(label="[bench] trimmed horn onto the shoulder servo", move=["horn-shoulder"],
         path=line(10, 0, 0), bench="shoulder", hosts=["servo-shoulder"]),
    dict(label="[bench] pinion over the horn", move=["pinion"], path=line(15, 0, 0),
         bench="shoulder"),
    dict(label="shoulder servo + pinion drop in from above", move=["servo-shoulder",
         "horn-shoulder", "pinion"], path=up(70)),
    dict(label="keeper back into its slot", move=["keeper"], path=up(30)),
    dict(label="[bench] elbow servo into link1", move=["servo-elbow"], path=line(-30, 0, 0),
         bench="link1", hosts=["link1"]),
    dict(label="[bench] link1 cover", move=["link1-cover"], path=line(-15, 0, 0), bench="link1"),
    dict(label="link1 down between the cheeks", move=["link1", "link1-cover", "servo-elbow"],
         path=up(70), allow=[("h3-pinion", "h3-link1")]),   # turn link1 so the teeth mesh
    dict(label="shoulder pin, +X side", move=["pin-shoulder-A"], path=line(15, 0, 0)),
    dict(label="shoulder pin, -X side", move=["pin-shoulder-B"], path=line(-15, 0, 0)),
    dict(label="elbow horn onto the spline", move=["horn-elbow"], path=line(10, 0, 0)),
    dict(label="link2 onto the elbow horn", move=["link2"], path=line(15, 0, 0)),
    dict(label="[bench] head servo into the head", move=["servo-head"], path=line(-30, 0, 0),
         bench="head", hosts=["head"]),
    dict(label="[bench] trimmed horn into the shade's pocket", move=["horn-head"],
         path=line(-12, 0, 0), bench="shade", hosts=["shade"]),
    dict(label="[bench] shade + horn onto the head servo (cover still off)",
         move=["shade", "horn-head"], path=line(5, 0, 0), bench="head"),
    dict(label="[bench] head cover slides in under the idler arm", move=["head-cover"],
         path=line(0, -70, 0), bench="head"),
    dict(label="[bench] head pin", move=["pin-head"], path=line(-15, 0, 0), bench="head"),
    dict(label="head + shade into link2's end", move=["head", "head-cover", "servo-head",
         "horn-head", "shade", "pin-head"], path=up(40)),
    dict(label="link2 cover", move=["link2-cover"], path=line(-15, 0, 0)),
    dict(label="elbow pin", move=["pin-elbow"], path=line(-15, 0, 0)),
    dict(label="LED ring into the shade", move=["ledring"], path=up(40)),
    dict(label="LED clamp", move=["ledclamp"], path=up(40)),
]


def samples(path, fine=2.0, coarse=6.0, near=25.0):
    """Points along the polyline from its far end to the seat: every `fine`
    mm within `near` of the seat (path length), every `coarse` beyond."""
    P = [np.array(p, float) for p in path]
    out, run = [], 0.0
    for a, b in zip(P[:-1], P[1:]):
        L_ = np.linalg.norm(b - a)
        s = 0.0
        while s < L_:
            out.append((run + s, a + (b - a) * (s / L_)))
            s += fine if run + s < near else coarse
        run += L_
    out.append((run, P[-1]))
    return sorted(out, key=lambda r: -r[0])        # far end first


def part_of(inst):
    return {i: f for i, f, _g, _m in A.LAYOUT}[inst]


def main():
    sw = L.SW()
    sw.close_all()
    lamp = A.Lamp(sw)
    G = A.group_tf(A.POSES["neutral"])
    home = {inst: G[g] @ M for inst, (c, f, g, M) in lamp.comps.items()}

    def place(inst, offset):
        M = home[inst].copy()
        M[:3, 3] += offset
        lamp.comps[inst][0].Transform2 = lamp.mu.CreateTransform(A.sw_array(M))

    rest = {inst: FAR for inst in lamp.comps}
    rest["tub"] = np.zeros(3)
    for inst in lamp.comps:
        place(inst, rest[inst])
    results = []
    for st in STEPS:
        base = np.array(BENCH[st["bench"]]) if st.get("bench") else np.zeros(3)
        for h in st.get("hosts", []):
            rest[h] = base
            place(h, base)
        allowed = {tuple(sorted(a[:2])): (a[2] if len(a) > 2 else math.inf)
                   for a in st.get("allow", [])}
        moving = {part_of(i) for i in st["move"]}
        first = None                               # the furthest-out blocked point
        for dist, off in samples(st["path"]):
            for inst in st["move"]:
                place(inst, base + off)
            lamp.doc.EditRebuild3()
            hits = []
            for names, vol in lamp.interferences():
                parts = [n.rsplit("-", 1)[0] for n in names]
                if not any(p in moving for p in parts):
                    continue
                if vol <= allowed.get(tuple(sorted(parts)), -1.0):
                    continue
                hits.append((names, round(vol, 2)))
            if hits and first is None:
                first = (round(dist, 1), [round(v, 1) for v in off], hits)
        ok = first is None
        results.append(dict(step=st["label"], ok=ok, path=[list(map(float, p)) for p in st["path"]],
                            blocked=None if ok else dict(path_mm_from_seat=first[0],
                                                         offset=first[1], hits=first[2])))
        print(f"{'ok     ' if ok else 'BLOCKED'}  {st['label']:58s}"
              + ("" if ok else f"  {first[0]} mm out {first[1]}: "
                 + "; ".join(f"{' x '.join(n)} {v} mm3" for n, v in first[2][:3])), flush=True)
        for inst in st["move"]:                    # it stays where it was put
            rest[inst] = base
            place(inst, base)
    json.dump(results, open(os.path.join(A.ROOT, "solidworks", "insert_report.json"), "w"), indent=1)
    return results


if __name__ == "__main__":
    r = main()
    sys.exit(0 if all(x["ok"] for x in r) else 1)
