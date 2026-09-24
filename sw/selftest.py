"""selftest.py -- prove swlib's plane mapping and extrude/cut directions
against SolidWorks itself before any real part depends on them."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import swlib as L

sw = L.SW()
p = sw.new_part("selftest")
res = {}
for plane in ("XY", "XZ", "YZ"):
    with p.sketch(plane) as s:
        s.circle(10, 20, 2)
    f = p.extrude(3, 7)
    # the new body's box: bounding box of the whole part, diffed per step
    pd = L.cast(p.doc, "IPartDoc")
    bodies = pd.GetBodies2(0, True)
    b = max((L.cast(x, "IBody2") for x in bodies), key=lambda bb: 0)  # placeholder
    box = [round(v * 1000, 2) for v in b.GetBodyBox()]
    res[plane] = box
    print(plane, "circle@(10,20) r2, extrude 3..7 ->", box)
with p.sketch("XY") as s:
    s.crect(10, 20, 1, 1)
v_before = p.mass()[1]; p.cut(-2, 50); print("cut removed mm3:", round(v_before - p.mass()[1], 2), "(expect 4.0 = 1x1x4 through the 3..7 disc)")
p.view()
print("mass", p.mass())
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_selftest")
print(p.save(os.path.join(out, "selftest.SLDPRT")))
print(p.stl(os.path.join(out, "selftest.stl")))
p.close()
