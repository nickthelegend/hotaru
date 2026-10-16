"""h3_build.py -- build HOTARU 3 in SolidWorks, part by part.

    python sw/h3_build.py                 # everything, recorded
    python sw/h3_build.py link1 pinion    # just these, recorded
    python sw/h3_build.py --no-record servo

Each part is built live in SolidWorks, shown isometric, screenshotted,
saved as .SLDPRT under solidworks/parts/, and (printed parts only) exported
as STL under exports/v3/. A part with a hand fix (hotaru3.HANDFIX) exports
its STL first, as printed, then takes the cut and is saved as built. The
SolidWorks window -- and only that window --
is recorded to recordings/ for the timelapse.
"""
from __future__ import annotations

import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3 as H
import hotaru3 as X
import swlib as L

ROOT = os.path.dirname(HERE)
PARTS = os.path.join(ROOT, "solidworks", "parts")
STL = os.path.join(ROOT, "exports", "v3")
SHOTS = os.path.join(ROOT, "solidworks", "shots")
REC = os.path.join(os.path.dirname(ROOT), "recordings")
FRAMES = os.path.join(REC, "frames")


def catalogue():
    """name -> (builder, printed?)"""
    cat = {}
    for n, fn in X.PRINTED:
        cat[n] = (fn, True)
    for n, ln in X.PINS.items():
        cat[n] = ((lambda sw, ln=ln, n=n: X.pin(sw, ln, n)), True)
    cat["MG996R"] = (X.servo, False)
    cat["Horn"] = ((lambda sw: X.horn(sw, H.HORN_L, "Horn")), False)
    cat["HornTrim"] = ((lambda sw: X.horn(sw, H.HORN_TRIM_L, "HornTrim")), False)
    for n, fn in X.BOUGHT:
        if fn is not None and n not in cat:
            cat[n] = (fn, False)
    for n, fn in X.HANDCUT + X.HARDWARE:
        cat[n] = (fn, False)
    return cat


def build(names, record=True, pace=0.35):
    sw = L.SW(pace=pace)
    sw.close_all()
    sw.to_front()
    hwnd = sw.hwnd()
    rec = None
    if record:
        sw.start_frames(FRAMES)
    cat = catalogue()
    report = {}
    try:
        for n in names:
            fn, printed = cat[n]
            t0 = time.time()
            p = fn(sw)
            p.view()
            if printed:                         # the part as it is printed ...
                p.stl(os.path.join(STL, f"{n}.stl"))
            if n in X.HANDFIX:                  # ... and as it is built
                X.HANDFIX[n](p)
                p.view()
            time.sleep(2.0)                     # let the finished part sit on screen
            L.snap(hwnd, os.path.join(SHOTS, f"{n}.png"))
            p.save(os.path.join(PARTS, f"{n}.SLDPRT"))
            g, vol, com = p.mass()
            report[n] = dict(volume_mm3=round(vol, 1), com_mm=[round(c, 2) for c in com],
                             features=len(p.feats), seconds=round(time.time() - t0, 1),
                             printed=printed)
            print(f"{n:18s} {len(p.feats):3d} features  {vol / 1000:8.2f} cm3  "
                  f"{time.time() - t0:5.1f}s", flush=True)
            _save_report({n: report[n]})         # after EVERY part: a crash
            p.close()                            # mid-run must not lose them
    finally:
        if record:
            print("frames so far:", sw._fn)
    return report


def _save_report(entries):
    rp = os.path.join(PARTS, "build_report.json")
    old = json.load(open(rp)) if os.path.exists(rp) else {}
    old.update(entries)
    json.dump(old, open(rp, "w"), indent=1)


def backfill():
    """Mass properties for any saved part missing from the report, read
    straight from the .SLDPRT -- no rebuild."""
    rp = os.path.join(PARTS, "build_report.json")
    have = json.load(open(rp)) if os.path.exists(rp) else {}
    sw = L.SW()
    for n, (fn, printed) in catalogue().items():
        path = os.path.join(PARTS, n + ".SLDPRT")
        if n in have or not os.path.exists(path):
            continue
        r = sw.app.OpenDoc6(path, 1, 1, "", 0, 0)
        doc = L.cast(r[0], "IModelDoc2")
        ext = L.cast(doc.Extension, "IModelDocExtension")
        mp = L.cast(ext.CreateMassProperty(), "IMassProperty")
        com = [round(v * 1000, 2) for v in mp.CenterOfMass]
        _save_report({n: dict(volume_mm3=round(mp.Volume * 1e9, 1), com_mm=com,
                              printed=printed, features=None, seconds=None)})
        print("backfilled", n, round(mp.Volume * 1e6, 2), "cm3", com)
        sw.app.CloseDoc(os.path.basename(path))


if __name__ == "__main__":
    if "--backfill" in sys.argv:
        backfill()
        sys.exit(0)
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    record = "--no-record" not in sys.argv
    names = args or list(catalogue().keys())
    build(names, record=record)
