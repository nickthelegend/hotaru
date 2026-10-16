"""h3_video_steps.py -- the build order the HOTARU 3 assembly video plays.

Every approach path for a part is h3_insert's, the one SolidWorks walked
for interference. Bolts come in along their own axis and turn as they go;
nuts slide in from their pocket's mouth or slot (h3_hardware.ENTRY). Bench
work happens at a spot beside the lamp and the finished subassembly is then
carried over in free air to the start of its checked approach path.

Output (assets/steps.json), all in the GLB's Z-up frame:
  nodes  name -> {kind, group}
  steps  [{chapter, title, body, chips, t0, t1, cam, focus}]
  moves  [{n, t0, t1, p: polyline of offsets, e: ease, spin, cut}]
  hide   name -> time it leaves the scene (cut-off material)
"""
from __future__ import annotations

import math

import numpy as np

import h3 as H
import h3_hardware as W
import h3_insert as I

INTRO = 7.0
FOV = 0.56
ASPECT = (1920 - 780) / 1080      # the 3D view is the frame right of the text panel

BENCH = {"turret": (235.0, 0.0, 0.0), "shoulder": (175.0, 10.0, 70.0),
         "link1": (190.0, -40.0, 10.0), "head": (205.0, -175.0, 0.0),
         "shade": (205.0, -175.0, 175.0)}

PATH = {tuple(s["move"]): s["path"] for s in I.STEPS}
HW = {i: (p, g, M, step) for i, p, g, M, step, _into in W.ROWS}


def gl(v):
    v = np.asarray(v, float)
    return np.array([v[0], -v[2], v[1]])


def hw(step):
    return [i for i, (p, g, M, s) in HW.items() if s == step]


def bolts_of(ids):
    return [i for i in ids if not HW[i][0].endswith("nut")]


def nuts_of(ids):
    return [i for i in ids if HW[i][0].endswith("nut")]


class Film:
    def __init__(self, inst, boxes):
        self.inst, self.boxes = inst, boxes
        self.t = INTRO
        self.rest = {}                  # node -> offset (SW mm) it currently sits at
        self.member = {}                # node -> bench it was built on
        self.moves, self.steps, self.hide = [], [], {}
        self.cur = None

    # ------------------------------------------------------------ steps --
    def step(self, chapter, title, body, chips=(), dur=5.0, az=150.0, el=26.0,
             focus=(), context=(), zoom=1.0, bench=False, near=260.0):
        """bench=True: the work happens on the bench, and the video shows only
        what is on it (the lamp behind it only confused the shot). `near` is
        the closest the camera may come, in mm."""
        self.cur = dict(chapter=chapter, title=title, body=body, chips=list(chips),
                        t0=round(self.t, 3), t1=round(self.t + dur, 3), bench=bench, near=near,
                        az=az, el=el, focus=list(focus), context=list(context), zoom=zoom)
        self.steps.append(self.cur)
        self.t += dur
        return self

    def T(self, f):
        return self.cur["t0"] + f * (self.cur["t1"] - self.cur["t0"])

    def _move(self, n, pts, a, b, ease="out", spin=None, cut=False):
        self.moves.append(dict(n=n, t0=round(self.T(a), 3), t1=round(self.T(b), 3),
                               p=[[round(c, 2) for c in gl(p)] for p in pts], e=ease,
                               spin=spin, cut=cut))
        self.rest[n] = np.asarray(pts[-1], float)

    def show(self, nodes):
        for n in nodes:                          # there from the first frame
            self.moves.append(dict(n=n, t0=0.0, t1=0.0, p=[[0, 0, 0], [0, 0, 0]], e="out",
                                   spin=None, cut=False))
            self.rest[n] = np.zeros(3)

    # ----------------------------------------------------------- actions --
    def insert(self, nodes, path=None, bench=None, a=0.06, b=0.62, ease="out"):
        """Bring `nodes` home (or to their spot on a bench) along `path` --
        offsets from the seat, seat first, as h3_insert walks them. Anything
        already sitting elsewhere (on a bench) is carried over first."""
        path = path if path is not None else PATH[tuple(nodes)]
        base = np.array(BENCH[bench]) if bench else np.zeros(3)
        far = base + np.array(path[-1], float)
        carried = [n for n in self.rest if self.member.get(n) and self.member[n] != bench
                   and n not in nodes and self.member[n] in {self.member.get(x) for x in nodes}]
        carried += [n for n in self.rest if self.member.get(n) in {"@" + x for x in nodes}]
        for n in list(nodes) + carried:
            approach = [base + np.array(p, float) for p in reversed(path)]
            if n in self.rest and not np.allclose(self.rest[n], far):
                r = self.rest[n]
                top = max(r[1], far[1]) + 30.0
                lead = [r, (r[0], top, r[2]), (far[0], top, far[2])]
                pts = lead + approach
                self._move(n, pts, a, b, "inout")
            else:
                self._move(n, approach, a, b, ease)
            if bench:
                self.member[n] = bench
            else:
                self.member.pop(n, None)
        return self

    def bolts(self, ids, bench=None, a=0.55, b=0.95, stagger=0.06):
        base = np.array(BENCH[bench]) if bench else np.zeros(3)
        ids = bolts_of(ids)
        span = min(stagger * max(len(ids) - 1, 0), (b - a) * 0.5)
        dt = span / max(len(ids) - 1, 1)
        for k, i in enumerate(ids):
            part, g, M, _s = HW[i]
            L = float(part[3:])
            ax = M[:3, 2]
            p0 = base - ax * (L + 16.0)
            t0 = a + k * dt
            t1 = b - span + k * dt
            spin = [round(c, 4) for c in gl(ax)] + [round(c, 2) for c in gl(M[:3, 3])] + [5.0]
            self._move(i, [p0, base], t0, t1, "inout", spin=spin)
            if bench:
                self.member[i] = bench
        return self

    def nuts(self, ids, bench=None, a=0.08, b=0.6, stagger=0.05):
        base = np.array(BENCH[bench]) if bench else np.zeros(3)
        ids = nuts_of(ids)
        span = min(stagger * max(len(ids) - 1, 0), (b - a) * 0.6)
        dt = span / max(len(ids) - 1, 1)
        for k, i in enumerate(ids):
            p0 = base + W.ENTRY[i] * 14.0
            self._move(i, [p0, base], a + k * dt, b - span + k * dt, "out")
            if bench:
                self.member[i] = bench
        return self

    def cut(self, names, away, a=0.15, b=0.85):
        """Material a hand cut takes off: it glows, then lifts away and goes."""
        for n in names:
            r = self.rest.get(n, np.zeros(3))
            self._move(n, [r, r, r + np.asarray(away, float)], a, b, "in", cut=True)
            self.hide[n] = round(self.T(b), 3)
        return self

    def aside(self, nodes, to, a=0.2, b=0.85):
        for n in nodes:
            r = self.rest.get(n, np.zeros(3))
            to_ = np.asarray(to, float)
            top = max(r[1], to_[1]) + 40.0
            self._move(n, [r, (r[0], top, r[2]), (to_[0], top, to_[2]), to_], a, b, "inout",
                       cut=True)
            self.member[n] = "aside"
        return self

    def preload(self, host, ids, bench=None, a=0.02, b=0.22):
        """Nuts into a part still waiting at the far end of its path (a
        cover's inner pockets): the part shows there, the nuts slide in, and
        the next insert() carries both."""
        base = np.array(BENCH[bench]) if bench else np.zeros(3)
        far = base + np.array(PATH[(host,)][-1], float)
        self._move(host, [far, far], a, a)
        self.member[host] = bench
        for k, i in enumerate(nuts_of(ids)):
            self._move(i, [far + W.ENTRY[i] * 14.0, far], a + (b - a) * 0.15 * k, b)
            self.member[i] = "@" + host                 # it rides in with its host
        return self

    # ------------------------------------------------------------ camera --
    def _box(self, n, t):
        """GL box of node n where it is at time t (end of its last move <= t).
        Cut-off material and the set-aside keeper count where they started."""
        lo, hi = (np.array(v) for v in self.boxes[n])
        off = np.zeros(3)
        for m in self.moves:
            if m["n"] == n and m["t0"] <= t:
                off = np.array(m["p"][0] if m["cut"] else m["p"][-1])
        return lo + off, hi + off

    def finish(self):
        out = []
        for s in self.steps:
            names = s["focus"] or [m["n"] for m in self.moves if s["t0"] <= m["t0"] < s["t1"]]
            names = names + s["context"]
            los, his = zip(*(self._box(n, s["t1"]) for n in names))
            lo, hi = np.min(los, 0), np.max(his, 0)
            ctr = (lo + hi) / 2
            ext = hi - lo
            vert = ext[2]
            horiz = math.hypot(ext[0], ext[1])
            tanh = math.tan(FOV / 2)
            dist = max(max(vert / (2 * tanh), horiz / (2 * tanh * ASPECT)) * 1.25,
                       s["near"]) / s["zoom"]
            # the leader points at what moves this step, seated
            moving = [m["n"] for m in self.moves if s["t0"] <= m["t0"] < s["t1"]] or names
            mlo, mhi = zip(*(self._box(n, s["t1"]) for n in moving))
            pt = (np.min(mlo, 0) + np.max(mhi, 0)) / 2
            out.append(dict(chapter=s["chapter"], title=s["title"], body=s["body"],
                            chips=s["chips"], t0=s["t0"], t1=s["t1"], bench=s["bench"],
                            cam=dict(tgt=[round(c, 1) for c in ctr], dist=round(dist, 1),
                                     az=s["az"], el=s["el"]),
                            pin=[round(c, 1) for c in pt]))
        return out


def tub_view(pt_sw, el=60.0):
    """Look into the tub from the opposite rim."""
    x, z = pt_sw[0], pt_sw[2]
    gx, gy = x, -z                      # GL horizontal
    if math.hypot(gx, gy) < 5:
        return 150.0, el
    az = math.degrees(math.atan2(-gx, -gy))
    return az, el


def build(inst, boxes):
    F = Film(inst, boxes)
    nodes = {n: dict(kind=p, group=g) for n, (p, _t, g) in inst.items()}
    F.show(["tub", "chip-rail-front", "chip-rail-back", "chip-mic-wall", "chip-pillar-groove"])

    B = "THE BASE"
    F.step(B, "Before you start",
           "Power up, open the serial monitor at 115200 and type neutral: all four servos go "
           "to centre, 1500 us. Fit every horn with its servo sitting there.", ["serial: neutral"],
           dur=5.0, az=150, el=34, focus=["tub"])
    F.step(B, "Hand cut 1: speaker rails",
           "Snip both speaker rails down to 10 mm above the floor. The cone under the slew "
           "ring hangs over the speaker, so it has to come in tilted.",
           ["flush cutters"], dur=5.5, az=-115, el=58, focus=["chip-rail-front", "chip-rail-back"],
           near=400.0)
    F.cut(["chip-rail-front", "chip-rail-back"], (-30, 45, 0))
    F.step(B, "Hand cut 2: mic slot",
           "Nibble the back end wall out of the mic slot, down to the slot floor. The board "
           "slides in sideways from there.", ["flush cutters"],
           dur=5.0, az=55, el=58, focus=["chip-mic-wall"], near=330.0)
    F.cut(["chip-mic-wall"], (25, 40, -20))
    F.step(B, "Hand cut 3: pillar groove",
           "If the pan servo's cable boot catches on the right-hand pillar, carve a groove "
           "2.5 mm deep and 6 mm wide from the cable notch to the top. It misses both screw holes.",
           ["hobby knife", "only if it catches"], dur=5.5, az=-105, el=58,
           focus=["chip-pillar-groove"], near=330.0)
    F.cut(["chip-pillar-groove"], (-25, 40, 0))

    def tubstep(title, body, nodes, chips=(), dur=4.6, zoom=1.0):
        c = np.mean([inst[n][1].reshape(-1, 3).mean(0) for n in nodes], 0)
        az, el = tub_view(c)
        F.step(B, title, body, chips, dur=dur, az=az, el=el, focus=nodes, zoom=zoom, near=380.0)
        F.insert(nodes)

    tubstep("Bulk capacitor", "Tilt it in from the middle of the tub, then press it down "
            "into its clip.", ["bulkcap"], zoom=0.7)
    tubstep("PCA9685", "Down onto its four locating pins, servo headers toward the middle.",
            ["pca9685"], zoom=0.8)
    tubstep("ESP32-S3", "In from the middle, then down onto its posts between the fences. "
            "The two USB-C ports face the two windows.", ["esp32"], zoom=0.8)
    tubstep("Amplifier", "In from the middle, down onto its four posts.", ["amp"], zoom=0.7)
    tubstep("Speaker", "Lower it in the middle, then out and down into the snipped rails, "
            "cone facing the grille.", ["speaker"], dur=5.0, zoom=0.75)
    tubstep("Microphone", "Slide the round board in sideways through the opened end of its "
            "slot. Its sound hole faces the wall.", ["mic"], dur=5.0, zoom=0.5)
    F.step(B, "Pan servo", "Cable through the notch, then down between the pillars, shaft up. "
           "Four M3x10 through the tabs straight into the pillars: they cut their own thread.",
           ["4 x M3x10", "self-tapping, no nuts"], dur=6.2, az=150, el=58,
           focus=["servo-pan"], context=["tub"], near=360.0)
    F.insert(["servo-pan"], b=0.5)
    F.bolts(hw("pan-servo"), a=0.5)
    F.step(B, "Pan horn", "Servo at centre: push the full horn onto the spline, arms "
           "running left to right.", dur=4.2, az=150, el=55, focus=["horn-pan", "servo-pan"],
           near=380.0)
    F.insert(["horn-pan"])

    C = "THE TURNTABLE"
    F.step(C, "Disc nuts", "On the bench, disc upside down: drop four M3 nuts into the hex "
           "pockets on its underside.", ["4 x M3 nut"], dur=4.8, az=150, el=-38,
           focus=["disc"], bench=True)
    F.insert(["disc"], path=[(0, 0, 0), (0, 0, 0)], bench="turret", a=0.0, b=0.08)
    F.nuts(hw("tower-nuts"), bench="turret", a=0.2, b=0.85)
    F.step(C, "Tower onto the disc", "Tower on top, then four M3x12 down through its base "
           "plate into those nuts.", ["4 x M3x12", "into nuts"], dur=5.8, az=150, el=30,
           focus=["disc", "tower"], bench=True)
    F.insert(["tower", "keeper"], path=PATH[("tower",)], bench="turret", b=0.5)
    F.bolts(hw("tower"), bench="turret", a=0.5)
    F.step(C, "Turntable on", "Feed the servo leads through the kidney slot. Lower disc and "
           "tower onto the slew ring: the pocket underneath drops over the pan horn.",
           dur=5.4, az=150, el=28, focus=["disc", "tower", "tub"])
    F.insert(["disc", "tower"], a=0.04, b=0.9)
    F.step(C, "Lip ring", "Over the disc's rim. Six M3x12 cut their own thread into the "
           "tub's cone.", ["6 x M3x12", "self-tapping, no nuts"], dur=5.6, az=150, el=48,
           focus=["lipring"])
    F.insert(["lipring"], b=0.45)
    F.bolts(hw("lipring"), a=0.45, stagger=0.05)
    F.step(C, "Button", "The Cherry MX snaps into the pod from above, leads down the hole. "
           "Keycap on.", dur=4.6, az=15, el=40, focus=["mx", "keycap"], near=260.0)
    F.insert(["mx"], b=0.45)
    F.insert(["keycap"], a=0.5, b=0.9)

    D = "THE SHOULDER"
    keeper_aside = np.array(BENCH["shoulder"]) + (0.0, 45.0, 0.0)
    F.step(D, "Hand cut 4: tower",
           "Saw out the bar above the servo window: one cut flush down the back wall, one "
           "straight up from the window's front edge. Keep the piece.",
           ["hacksaw or razor saw", "keep the piece"], dur=6.6, az=165, el=52,
           focus=["keeper"], context=["tower"], near=230.0)
    F.aside(["keeper"], keeper_aside, a=0.3, b=0.95)
    F.step(D, "Shoulder drive, on the bench", "Servo at centre: trimmed horn onto the spline, "
           "pinion over the horn, one M3x10 into the shaft. Two nuts into the keeper.",
           ["1 x M3x10", "into the servo shaft", "2 x M3 nut"], dur=6.2, az=120, el=26,
           focus=["servo-shoulder", "pinion", "keeper"], bench=True)
    F.insert(["servo-shoulder"], path=[(0, 0, 0), (0, 0, 0)], bench="shoulder", a=0.0, b=0.06)
    F.insert(["horn-shoulder"], bench="shoulder", a=0.06, b=0.3)
    F.insert(["pinion"], bench="shoulder", a=0.3, b=0.55)
    F.bolts(hw("shoulder-bench"), bench="shoulder", a=0.55, b=0.8)
    for i in hw("keeper-nuts"):
        F.member[i] = "keeper"
    kb = keeper_aside
    for k, i in enumerate(nuts_of(hw("keeper-nuts"))):
        F._move(i, [kb + W.ENTRY[i] * 14.0, kb], 0.62 + 0.05 * k, 0.9 + 0.05 * k - 0.05)
    F.step(D, "Bulkhead nuts", "Two M3 nuts into the lower pockets on the servo bulkhead's "
           "pinion side.", ["2 x M3 nut"], dur=4.2, az=115, el=62,
           focus=nuts_of(hw("shoulder-nuts")), near=320.0)
    F.nuts(hw("shoulder-nuts"), a=0.15, b=0.8)
    F.step(D, "Shoulder servo", "Drop servo and pinion in from above, the pinion through the "
           "back-wall notch. Two M3x10 through the lower tabs.", ["2 x M3x10", "into nuts"],
           dur=6.4, az=160, el=44, focus=["servo-shoulder", "pinion", "tower"], near=220.0)
    F.insert(["servo-shoulder", "horn-shoulder", "pinion"], a=0.04, b=0.56)
    F.bolts(hw("shoulder-tabs"), a=0.6)
    F.step(D, "Keeper", "Slide the sawn piece back into its slot. Two M3x10 through the "
           "upper tabs into its nuts.", ["2 x M3x10", "into nuts"], dur=5.8, az=150, el=46,
           focus=["keeper", "servo-shoulder"], near=290.0)
    F.insert(["keeper"] + nuts_of(hw("keeper-nuts")), path=PATH[("keeper",)], a=0.04, b=0.55)
    F.bolts(hw("keeper-bolts"), a=0.58)

    E = "LINK 1"
    F.step(E, "Link 1 nuts", "On the bench: one nut in the gear-side pin pocket, six in the "
           "cover-bolt slots, four in the elbow ledge slots.", ["11 x M3 nut"], dur=5.6,
           az=-120, el=24, focus=["link1"], bench=True)
    F.insert(["link1"], path=[(0, 0, 0), (0, 0, 0)], bench="link1", a=0.0, b=0.06)
    F.nuts(hw("link1-nuts"), bench="link1", a=0.1, b=0.9, stagger=0.04)
    F.step(E, "Elbow servo", "In from the open side, shaft through link 1's plate. Four M3x10 "
           "through the tabs.", ["4 x M3x10", "into nuts"], dur=5.8, az=-120, el=24,
           focus=["link1", "servo-elbow"], bench=True)
    F.insert(["servo-elbow"], bench="link1", b=0.5)
    F.bolts(hw("elbow-servo"), bench="link1", a=0.52)
    F.step(E, "Link 1 cover", "Two nuts into the cover's pin pockets, then the cover on with "
           "six M3x10.", ["2 x M3 nut", "6 x M3x10"], dur=6.0, az=-120, el=24,
           focus=["link1", "link1-cover"], bench=True)
    F.preload("link1-cover", hw("link1-cover-nuts"), bench="link1", a=0.02, b=0.22)
    F.insert(["link1-cover"], bench="link1", a=0.24, b=0.5)
    F.bolts(hw("link1-cover"), bench="link1", a=0.52, stagger=0.04)
    F.step(E, "Link 1 into the tower", "Lower it between the cheeks. Rock it as it drops so the "
           "gear teeth find the pinion.", dur=5.6, az=140, el=22,
           focus=["link1", "tower"])
    F.insert(["link1", "link1-cover", "servo-elbow"], a=0.04, b=0.92)
    F.step(E, "Shoulder pins", "One pin from each side through the cheek into link 1. An "
           "M3x16 through each pin into its nut.", ["2 x M3x16", "into nuts"], dur=6.0,
           az=115, el=16, focus=["pin-shoulder-A", "pin-shoulder-B", "tower"], near=320.0)
    F.insert(["pin-shoulder-A"], a=0.04, b=0.35)
    F.insert(["pin-shoulder-B"], a=0.2, b=0.5)
    F.bolts(hw("shoulder-pins"), a=0.55)

    G = "LINK 2 AND THE HEAD"
    F.step(G, "Link 2", "Elbow servo at centre: full horn onto the spline, link 2's pocket "
           "over the horn, one M3x10 into the shaft. Four nuts into link 2's wall slots.",
           ["1 x M3x10", "into the servo shaft", "4 x M3 nut"], dur=6.4, az=100, el=20,
           focus=["link2", "horn-elbow", "servo-elbow"])
    F.insert(["horn-elbow"], a=0.03, b=0.22)
    F.insert(["link2"], a=0.22, b=0.45)
    F.bolts(hw("link2"), a=0.46, b=0.68)
    F.nuts(hw("link2-nuts"), a=0.7, b=0.97)
    F.step(G, "Head, on the bench", "Nine nuts into the head's slots: four for the servo tabs, "
           "one for the cover, four for mounting. Head servo in from the open side, four M3x10.",
           ["9 x M3 nut", "4 x M3x10"], dur=6.6, az=-125, el=24, focus=["head", "servo-head"],
           bench=True)
    F.insert(["head"], path=[(0, 0, 0), (0, 0, 0)], bench="head", a=0.0, b=0.05)
    F.nuts(hw("head-nuts"), bench="head", a=0.06, b=0.42, stagger=0.03)
    F.insert(["servo-head"], bench="head", a=0.42, b=0.68)
    F.bolts(hw("head-servo"), bench="head", a=0.68, b=0.97)
    F.step(G, "Horn into the shade", "Servo-side arm of the shade: press the trimmed horn into "
           "its pocket.", dur=4.2, az=-70, el=20, focus=["shade", "horn-head"], bench=True)
    F.insert(["shade"], path=[(0, 0, 0), (0, 0, 0)], bench="shade", a=0.0, b=0.05)
    F.insert(["horn-head"], bench="shade", a=0.12, b=0.7)
    F.step(G, "Shade onto the head", "Cover still off: slide the shade over the head so the "
           "horn's hub goes onto the spline. One M3x8 through the arm into the shaft.",
           ["1 x M3x8", "into the servo shaft"], dur=6.0, az=70, el=20,
           focus=["head", "shade"], bench=True)
    F.insert(["shade", "horn-head"], bench="head", a=0.04, b=0.58)
    F.bolts(hw("shade-hub"), bench="head", a=0.62, b=0.95)
    F.step(G, "Head cover", "Nut into the cover's pin pocket. Slide the cover in under the "
           "shade's idler arm, then one M3x10 at the top.", ["1 x M3 nut", "1 x M3x10"],
           dur=6.0, az=-105, el=16, focus=["head", "head-cover"], bench=True)
    F.preload("head-cover", hw("head-cover-nut"), bench="head", a=0.02, b=0.2)
    F.insert(["head-cover"], bench="head", a=0.24, b=0.62)
    F.bolts(hw("head-cover"), bench="head", a=0.66, b=0.95)
    F.step(G, "Head pin", "Through the idler arm into the cover. One M3x12 into its nut.",
           ["1 x M3x12", "into the nut"], dur=4.8, az=-100, el=16,
           focus=["pin-head", "head-cover"], bench=True)
    F.insert(["pin-head"], bench="head", a=0.04, b=0.45)
    F.bolts(hw("head-pin"), bench="head", a=0.5)
    F.step(G, "Head onto link 2", "Lower the head's tail into link 2's open end. Two M3x16 "
           "through link 2's plate into the head.", ["2 x M3x16", "into nuts"], dur=6.2,
           az=130, el=20, focus=["head", "shade", "link2"], near=330.0)
    F.insert(["head", "head-cover", "servo-head", "horn-head", "shade", "pin-head"],
             a=0.03, b=0.6)
    F.bolts(hw("head-mount-D"), a=0.64)
    F.step(G, "Link 2 cover", "Cover on: four M3x10, and two M3x16 through to the head.",
           ["4 x M3x10", "2 x M3x16", "into nuts"], dur=6.0, az=-110, el=18,
           focus=["link2-cover", "head"], near=320.0)
    F.insert(["link2-cover"], a=0.04, b=0.42)
    F.bolts(hw("link2-cover"), a=0.46, stagger=0.04)
    F.step(G, "Elbow pin", "Through the link 2 cover into link 1. One M3x12 into the nut "
           "waiting in link 1's cover.", ["1 x M3x12", "into the nut"], dur=4.8, az=-105, el=14,
           focus=["pin-elbow", "link2-cover"])
    F.insert(["pin-elbow"], a=0.04, b=0.45)
    F.bolts(hw("elbow-pin"), a=0.5)
    F.step(G, "LED ring", "Ring into the back of the shade, LEDs facing out. Push the clamp "
           "ring down the cone until it wedges.", dur=5.4, az=150, el=62,
           focus=["shade", "ledring"], near=340.0)
    F.insert(["ledring"], a=0.04, b=0.45)
    F.insert(["ledclamp"], a=0.5, b=0.9)

    steps = F.finish()
    end = F.t
    return dict(intro=INTRO, body_end=round(end, 3), duration=round(end + 9.0, 3),
                nodes=nodes, steps=steps, moves=F.moves, hide=F.hide,
                joints=dict(shaft=[0.0, -H.SH_SHAFT_Z, H.SH_SHAFT_Y], p=[0.0, -H.Z_P, H.Y_P],
                            e=[0.0, -H.Z_P, H.Y_E], h=[0.0, -H.Z_P, H.Y_H],
                            ratio=H.GEAR_RATIO))
