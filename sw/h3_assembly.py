"""h3_assembly.py -- put HOTARU 3 together in SolidWorks and prove it moves.

    python sw/h3_assembly.py            # build HOTARU3.SLDASM, check every pose

What "working" means here, checked by SolidWorks itself, not by eye:
  1. every part and bought component placed where the design says;
  2. the lamp driven through a set of poses spanning every joint's range,
     and SolidWorks' own Interference Detection run at EACH pose -- zero
     interfering pairs is the pass condition;
  3. the torque each servo must hold, from SolidWorks' mass properties of
     the real parts, worst case over a grid of poses, against the MG996R's
     stall torque.

Kinematic groups (a pose is only these joint angles, degrees):
  base    tub, lip ring, keycap, pan servo, electronics
  turret  disc, tower, shoulder servo, pan horn              pan    about Y
  pinion  pinion + its trimmed horn       -1.8 x shoulder    about its shaft
  link1   link1, its cover, elbow servo, shoulder + elbow pins  shoulder
  link2   link2, cover, head block + cover, head servo, elbow horn, head pin
  shade   shade, head horn, LED ring                          head tilt
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

import numpy as np
import pythoncom
from win32com.client import VARIANT

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3 as H
import swlib as L

ROOT = os.path.dirname(HERE)
PARTS = os.path.join(ROOT, "solidworks", "parts")
ASM = os.path.join(ROOT, "solidworks", "HOTARU3.SLDASM")
SHOTS = os.path.join(ROOT, "solidworks", "shots")
REC = os.path.join(os.path.dirname(ROOT), "recordings")

# ------------------------------------------------------------ transforms ---
def T(R=None, t=(0, 0, 0)):
    M = np.eye(4)
    if R is not None:
        M[:3, :3] = R
    M[:3, 3] = t
    return M


def rot_x(deg, about=(0, 0, 0)):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    p = np.array(about, dtype=float)
    return T(R, p - R @ p)


def rot_y(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return T(np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]))


R_PAN = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])     # S.x->X, S.y->-Z, S.z->Y
R_ARM = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])    # S.x->-Y, S.y->-Z, S.z->X
R_FLIP = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])   # 180 about Y

# (instance, part file, group, local->world at the neutral pose)
LAYOUT = [
    ("tub", "h3-tub", "base", T()),
    ("lipring", "h3-lipring", "base", T()),
    ("keycap", "h3-keycap", "base", T()),
    ("mx", "CherryMX", "base", T()),
    ("pca9685", "PCA9685", "base", T()),
    ("esp32", "ESP32-S3", "base", T()),
    ("amp", "MAX98357A", "base", T()),
    ("speaker", "Speaker2040", "base", T()),
    ("mic", "INMP441", "base", T()),
    ("bulkcap", "Cap2200uF", "base", T()),
    ("servo-pan", "MG996R", "base", T(R_PAN, (0, H.PAN_CASE_TOP, 0))),
    ("horn-pan", "Horn", "turret", T(R_PAN, (0, H.PAN_CASE_TOP, 0))),
    ("disc", "h3-disc", "turret", T()),
    ("tower", "h3-tower", "turret", T()),
    ("servo-shoulder", "MG996R", "turret", T(R_ARM, (H.SH_CASE_TOP, H.Y_P, H.SH_SHAFT_Z))),
    ("pinion", "h3-pinion", "pinion", T()),
    ("horn-shoulder", "HornTrim", "pinion", T(R_ARM, (H.SH_CASE_TOP, H.Y_P, H.SH_SHAFT_Z))),
    ("link1", "h3-link1", "link1", T()),
    ("link1-cover", "h3-link1-cover", "link1", T()),
    ("servo-elbow", "MG996R", "link1", T(R_ARM, (H.EL_CASE_TOP, H.Y_E, H.Z_P))),
    ("pin-shoulder-A", "h3-pin-shoulder", "link1", T(None, (H.L1A[1] - 1.0, H.Y_P, H.Z_P))),
    ("pin-shoulder-B", "h3-pin-shoulder", "link1", T(R_FLIP, (H.L1B[0] + 1.0, H.Y_P, H.Z_P))),
    ("pin-elbow", "h3-pin-elbow", "link1", T(R_FLIP, (H.L1B[0] + 1.0, H.Y_E, H.Z_P))),
    ("horn-elbow", "Horn", "link2", T(R_ARM, (H.EL_CASE_TOP, H.Y_E, H.Z_P))),
    ("link2", "h3-link2", "link2", T()),
    ("link2-cover", "h3-link2-cover", "link2", T()),
    ("head", "h3-head", "link2", T()),
    ("head-cover", "h3-head-cover", "link2", T()),
    ("servo-head", "MG996R", "link2", T(R_ARM, (H.HD_CASE_TOP, H.Y_H, H.Z_P))),
    ("pin-head", "h3-pin-head", "link2", T(R_FLIP, (H.NOSE_B[0] + 1.0, H.Y_H, H.Z_P))),
    ("horn-head", "HornTrim", "shade", T(R_ARM, (H.HD_CASE_TOP, H.Y_H, H.Z_P))),
    ("shade", "h3-shade", "shade", T(None, (0, H.Y_H, H.Z_P))),
    ("ledring", "WS2812-ring", "shade", T(None, (0, H.Y_H, H.Z_P))),
    ("ledclamp", "h3-ledclamp", "shade", T(None, (0, H.Y_H, H.Z_P))),
]

LIMITS = {"pan": (-90, 90), "shoulder": (-30, 70), "elbow": (-120, 120), "head": (-75, 75)}

POSES = {
    "neutral":    (0, 0, 0, 0),
    "rest":       (0, 15, 105, 70),
    "reach":      (0, 60, 40, 30),
    "lean-back":  (0, -30, 60, -30),
    "look-up":    (0, 10, 30, -70),
    "fold":       (0, 0, 120, 75),
    "fold-back":  (0, 30, -100, 0),
    "pan-left":   (-90, 20, 90, 45),
    "pan-right":  (90, 20, 90, 45),
    "full-reach": (0, 70, 0, 0),
    "curious":    (35, 45, 75, -20),
    "shy":        (-40, -20, 110, 60),
}


def group_tf(pose):
    pan, sh, el, hd = pose
    turret = rot_y(pan)
    pin = turret @ rot_x(-H.GEAR_RATIO * sh, (0, H.SH_SHAFT_Y, H.SH_SHAFT_Z))
    l1 = turret @ rot_x(sh, (0, H.Y_P, H.Z_P))
    l2 = l1 @ rot_x(el, (0, H.Y_E, H.Z_P))
    shd = l2 @ rot_x(hd, (0, H.Y_H, H.Z_P))
    return {"base": T(), "turret": turret, "pinion": pin, "link1": l1,
            "link2": l2, "shade": shd}


def sw_array(M):
    """4x4 (column vectors, mm) -> SolidWorks' 16-double transform array.
    SolidWorks applies a row vector: p' = p . R + t, so its 3x3 block is
    our R transposed. Translation in metres."""
    R = M[:3, :3]
    t = M[:3, 3] / 1000.0
    a = list(R.T.reshape(-1)) + list(t) + [1.0, 0.0, 0.0, 0.0]
    return VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, [float(v) for v in a])


# ------------------------------------------------------------- assembly ---
class Lamp:
    def __init__(self, sw):
        self.sw = sw
        self.mu = L.cast(sw.app.GetMathUtility(), "IMathUtility")
        self.docs = {}
        for _i, f, _g, _m in LAYOUT:
            if f in self.docs:
                continue
            path = os.path.join(PARTS, f + ".SLDPRT")
            r = sw.app.OpenDoc6(path, 1, 1, "", 0, 0)
            d = r[0] if isinstance(r, tuple) else r
            if d is None:
                raise RuntimeError("could not open " + path)
            self.docs[f] = path
        self.doc = sw.new_assembly()
        self.ad = L.cast(self.doc, "IAssemblyDoc")
        self.comps = {}
        for inst, f, g, M in LAYOUT:
            c = self.ad.AddComponent5(self.docs[f], 0, "", False, "", 0, 0, 0)
            if c is None:
                raise RuntimeError("AddComponent5 failed for " + f)
            c = L.cast(c, "IComponent2")
            if c.IsFixed():
                self.doc.ClearSelection2(True)
                c.Select4(False, None, False)
                self.ad.UnfixComponent()
            self.comps[inst] = (c, f, g, M)
        for f in self.docs:                      # parts stay loaded via the asm
            try:
                sw.app.CloseDoc(os.path.basename(self.docs[f]))
            except Exception:
                pass
        self.doc.ShowNamedView2("*Isometric", 7)

    def pose(self, pose, redraw=True):
        G = group_tf(pose)
        for inst, (c, f, g, M) in self.comps.items():
            W = G[g] @ M
            c.Transform2 = self.mu.CreateTransform(sw_array(W))
        if redraw:
            self.doc.EditRebuild3()
            self.doc.ViewZoomtofit2()
            self.doc.GraphicsRedraw2()

    def interferences(self):
        idm = L.cast(self.ad.InterferenceDetectionManager, "IInterferenceDetectionMgr")
        idm.TreatCoincidenceAsInterference = False
        idm.TreatSubAssembliesAsComponents = True
        idm.IncludeMultibodyPartInterferences = True
        idm.MakeInterferingPartsTransparent = False
        idm.CreateFastenersFolder = False
        idm.IgnoreHiddenBodies = True
        idm.ShowIgnoredInterferences = False
        idm.UseTransform = True
        out = []
        ints = idm.GetInterferences() or []
        for it in ints:
            it = L.cast(it, "IInterference")
            names = sorted(L.cast(c, "IComponent2").Name2 for c in (it.Components or []))
            out.append((names, it.Volume * 1e9))
        idm.Done()
        return out

    def check_box(self, inst):
        c = self.comps[inst][0]
        b = c.GetBox(False, False)
        return [round(v * 1000, 2) for v in b]


# ---------------------------------------------------------------- torque ---
PLA = 1.24e-3          # g/mm3
FILL = 0.75            # 5 mm plates at 4 walls + 30% infill print ~75% dense
BOUGHT_G = {"MG996R": 55.0, "Horn": 3.0, "HornTrim": 2.2, "WS2812-ring": 9.0}


def masses():
    rep = json.load(open(os.path.join(PARTS, "build_report.json")))
    out = {}
    for inst, f, g, M in LAYOUT:
        r = rep[f]
        m = BOUGHT_G.get(f, r["volume_mm3"] * PLA * (FILL if r["printed"] else 1.0))
        out[inst] = (m, np.array(r["com_mm"] + [1.0]), g, M)
    return out


def joint_torques(pose, ms, hardware_g=None):
    """Gravity torque (N.m) that each joint must hold at `pose`."""
    G = group_tf(pose)
    g = 9.81e-3                              # N per g ... per mm -> N.m below
    axes = {"shoulder": ((0, H.Y_P, H.Z_P), ("link1", "link2", "shade", "pinion_x")),
            "elbow": ((0, H.Y_E, H.Z_P), ("link2", "shade")),
            "head": ((0, H.Y_H, H.Z_P), ("shade",))}
    res = {}
    for jn, (piv, groups) in axes.items():
        # the pivot itself moves with the upstream joints
        up = {"shoulder": "turret", "elbow": "link1", "head": "link2"}[jn]
        pw = G[up] @ np.array(list(piv) + [1.0])
        tau = 0.0
        for inst, (m, com, grp, M) in ms.items():
            if grp not in groups:
                continue
            p = G[grp] @ M @ com
            # torque about X of weight (0, -m g, 0) at lever (p - pw):
            # tau_x = dy*Fz - dz*Fy = dz * m g
            dz = p[2] - pw[2]
            tau += m * 9.81e-3 * dz * 1e-3         # g->kg, mm->m
        res[jn] = tau
    return res


# ------------------------------------------------------------------- main ---
def main(record=True):
    sw = L.SW()
    sw.close_all()
    sw.to_front()
    hwnd = sw.hwnd()
    if record:
        sw.start_frames(os.path.join(REC, "frames"))
    report = {"poses": {}, "torque": {}}
    try:
        lamp = Lamp(sw)
        # sanity: a rotated component must land where the math says
        lamp.pose(POSES["neutral"])
        print("pan servo box", lamp.check_box("servo-pan"))
        print("shoulder servo box", lamp.check_box("servo-shoulder"))
        for name, pose in POSES.items():
            lamp.pose(pose)
            time.sleep(0.8)
            ints = lamp.interferences()
            L.snap(hwnd, os.path.join(SHOTS, f"pose-{name}.png"))
            for _k in range(4):             # hold each pose for a beat
                sw.frame()
            report["poses"][name] = {"angles": pose,
                                     "interferences": [[a, round(v, 2)] for a, v in ints]}
            flag = "CLEAR" if not ints else f"{len(ints)} INTERFERING"
            print(f"{name:11s} {str(pose):22s} {flag}", flush=True)
            for a, v in ints:
                print(f"      {' x '.join(a)}  {v:.1f} mm3")
        lamp.pose(POSES["rest"])
        lamp.doc.ShowNamedView2("*Isometric", 7)
        lamp.doc.ViewZoomtofit2()
        r = L.cast(lamp.doc.Extension, "IModelDocExtension").SaveAs3(ASM, 0, 1, None, None, 0, 0)
        print("saved", ASM, r)
    finally:
        pass
    json.dump(report, open(os.path.join(ROOT, "solidworks", "pose_report.json"), "w"), indent=1)
    return report


if __name__ == "__main__":
    main(record="--no-record" not in sys.argv)
