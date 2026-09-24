"""h3_torque.py -- what each servo has to hold, worst case, HOTARU 3.

Masses come from SolidWorks: every printed part's volume and centre of mass
as SolidWorks computed them (build_report.json, written by h3_build.py),
times PLA density and a print-fill factor. Bought parts use their real
weights. The lamp is then posed over a grid spanning every joint's full
range and the gravity torque on each joint is taken at its worst.

    python sw/h3_torque.py
"""
from __future__ import annotations

import itertools
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3 as H
import h3_assembly as A

MG996R_STALL_48 = 9.4 * 0.0980665     # N.m at 4.8 V (datasheet 9.4 kg.cm)
MG996R_STALL_60 = 11.0 * 0.0980665    # N.m at 6.0 V
# A hobby servo holding more than about a third of stall runs hot and
# hunts. That, not stall, is the line a lamp that sits posed all day needs.
HOLD_FRACTION = 1 / 3


def main():
    ms = A.masses()
    lo = {k: v for k, v in A.LIMITS.items()}
    grid = lambda a, b, n: [a + (b - a) * i / (n - 1) for i in range(n)]
    worst = {"shoulder": (0, None), "elbow": (0, None), "head": (0, None)}
    for sh, el, hd in itertools.product(grid(*lo["shoulder"], 22), grid(*lo["elbow"], 25),
                                        grid(*lo["head"], 16)):
        t = A.joint_torques((0, sh, el, hd), ms)
        for j in worst:
            if abs(t[j]) > abs(worst[j][0]):
                worst[j] = (t[j], (0, round(sh), round(el), round(hd)))

    moving = sum(m for inst, (m, *_r) in ms.items()
                 if _r[1] in ("link1", "link2", "shade"))
    print("HOTARU 3 -- servo torque, worst case over the full range of motion")
    print(f"  moving arm mass (link1 onward)      {moving:6.0f} g")
    print(f"  MG996R stall                        {MG996R_STALL_48:.2f} N.m @4.8 V, "
          f"{MG996R_STALL_60:.2f} N.m @6 V")
    print(f"  comfortable holding (1/3 of stall)  {MG996R_STALL_48 * HOLD_FRACTION:.2f} N.m\n")
    out = {}
    for j, (tau, pose) in worst.items():
        servo = abs(tau) / (H.GEAR_RATIO if j == "shoulder" else 1.0)
        frac = servo / MG996R_STALL_48
        out[j] = dict(joint_Nm=round(abs(tau), 3), servo_Nm=round(servo, 3),
                      stall_fraction_4v8=round(frac, 3), worst_pose=pose)
        gear = f" / {H.GEAR_RATIO:.1f} gear" if j == "shoulder" else ""
        verdict = "OK" if frac <= HOLD_FRACTION else ("MARGINAL" if frac < 0.5 else "TOO HIGH")
        print(f"  {j:9s} joint {abs(tau):.3f} N.m{gear:12s} -> servo {servo:.3f} N.m "
              f"= {frac * 100:4.1f}% of stall  {verdict:8s} worst at {pose}")
    json.dump(out, open(os.path.join(os.path.dirname(HERE), "solidworks", "torque_report.json"),
                        "w"), indent=1)
    return out


if __name__ == "__main__":
    main()
