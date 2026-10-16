"""h3_actions.py -- HOTARU's gestures, checked, then written as C for the firmware.

    python sw/h3_actions.py      # check every action, write firmware/hotaru/actions.h

An action is a list of keys (pan, shoulder, elbow, head, ms): joint degrees
the same way the motion firmware counts them (0 0 0 0 = arm straight up,
shoulder + leans forward, elbow + folds forward, head + looks down), and the
time to get there. A key equal to the previous one is a hold.

Every key must pass the firmware's pose guard (holding torque, no part of
the arm in the base, tower or button pod -- h3_guard is the same code) inside
the joint range the user's calibration allows, and every step between two
keys must have a route the firmware's planner accepts. The script refuses to
write the header otherwise. It also reports keys the speed limits will
stretch, so a gesture designed for 200 ms that really takes 450 is visible.
"""
from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3_guard as G

ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "firmware", "hotaru", "actions.h")

# joint range from the user's calibration (2026-10-14): shoulder servo 0..125
# deg through the 1.8 : 1 gear, elbow servo 0..120, both 0 at straight up.
# Pan and head: the CAD limits until they are measured.
LIMITS = [(-90.0, 90.0), (0.0, 125.0 / 1.8), (0.0, 120.0), (-75.0, 75.0)]
# per-joint speed caps the firmware uses (deg/s, deg/s^2 at the joint)
VCAP = [180.0, 60.0, 150.0, 300.0]
ACAP = [900.0, 240.0, 600.0, 2000.0]

MOODS = ["calm", "happy", "excited", "sad", "shy", "curious", "angry", "sleepy",
         "love", "surprised", "scared", "proud", "thinking", "playful"]

# ------------------------------------------------------------------ poses ---
STAND = (0, 10, 55, 25)      # attentive, light straight ahead
LOOKU = (0, 10, 55, 10)      # looking up at a face
TALL = (0, 0, 20, 60)
PROUD = (0, 0, 15, 40)
REST = (0, 15, 105, 70)      # parked: folded, light on the desk (power-up pose)
SLEEP = (0, 10, 115, 75)
DESK = (0, 30, 90, 60)
SPOT = (0, 35, 80, 75)
LEANIN = (0, 35, 70, 0)
BOW = (0, 45, 60, 60)
UP = (0, 5, 30, -60)
LOW = (0, 40, 110, 30)
SHY = (-45, 0, 115, 60)
HIDE = (-60, 0, 120, 75)


def P(base, pan=None, sh=None, el=None, hd=None, dpan=0, dsh=0, del_=0, dhd=0):
    b = list(base)
    for i, v in enumerate((pan, sh, el, hd)):
        if v is not None:
            b[i] = v
    for i, d in enumerate((dpan, dsh, del_, dhd)):
        b[i] += d
    return tuple(b)


def K(pose, ms):
    return tuple(pose) + (ms,)


def hold(pose, ms):
    return K(pose, ms)


def osc(base, joint, amp, n, ms, start=+1):
    """n swings of +-amp on one joint about base, then back to base."""
    keys = []
    s = start
    for _ in range(n):
        p = list(base)
        p[joint] += s * amp
        keys.append(K(p, ms))
        s = -s
    keys.append(K(base, ms))
    return keys


ACTIONS = []                 # (name, mood, description for the LLM, keys, public)


def A(name, mood, desc, keys, public=True):
    ACTIONS.append((name, mood, desc, keys, public))


# ---------------------------------------------------------------- the set ---
A("hello", "happy", "perks up tall, then a small friendly bow",
  [K(TALL, 600), K((0, 25, 50, 60), 450), K(STAND, 500)])
A("wave", "happy", "sways its light side to side, waving",
  [K((-25, 10, 55, 15), 350), K((25, 10, 55, 35), 450), K((-25, 10, 55, 15), 450),
   K((25, 10, 55, 35), 450), K(STAND, 350)])
A("bow", "calm", "a slow polite bow",
  [K(BOW, 800), hold(BOW, 500), K(STAND, 800)])
A("goodbye", "calm", "waves, bows, then folds down to rest",
  [K((-20, 10, 55, 15), 350), K((20, 10, 55, 35), 400), K((0, 30, 60, 60), 600), K(REST, 1000)])
A("nod", "happy", "nods yes",
  [K(STAND, 300)] + osc(STAND, 3, 16, 4, 280))
A("double_nod", "excited", "two quick enthusiastic nods",
  [K((0, 15, 60, 25), 400)] + osc((0, 15, 60, 25), 3, 18, 4, 300) + [K(STAND, 300)])
A("shake", "calm", "shakes its head no",
  [K(STAND, 300)] + osc(STAND, 0, 12, 4, 330))
A("slow_shake", "sad", "a slow disappointed no, light lowered",
  [K((0, 10, 60, 40), 500)] + osc((0, 10, 60, 40), 0, 14, 4, 550) + [K(STAND, 600)])
A("shy", "shy", "turns away and tucks its light down, then peeks back",
  [K((-30, 5, 80, 45), 500), K(SHY, 700), hold(SHY, 900), K((-20, 5, 70, 30), 600),
   hold((-20, 5, 70, 30), 500), K(STAND, 800)])
A("blush", "shy", "glances at you, then hides its face to the side",
  [K((20, 5, 70, 35), 500), K((35, 0, 100, 60), 600), hold((35, 0, 100, 60), 800), K(STAND, 700)])
A("curious", "curious", "leans in close with a little head tilt",
  [K((20, 35, 70, 0), 800), K((20, 35, 70, -15), 300), K((20, 35, 70, 5), 300),
   hold((20, 35, 70, 5), 600), K(STAND, 800)])
A("peek", "playful", "hides to the side, then pops up to peek",
  [K((60, 0, 110, 70), 800), K((60, 15, 70, 20), 500), hold((60, 15, 70, 20), 600),
   K((60, 0, 110, 70), 450), K(STAND, 900)])
A("look_left", "curious", "turns to look to the left and stays", [K((-60, 10, 55, 20), 700)])
A("look_right", "curious", "turns to look to the right and stays", [K((60, 10, 55, 20), 700)])
A("look_up", "curious", "looks up at the ceiling", [K(UP, 800)])
A("look_down", "calm", "looks down at the desk", [K(DESK, 800)])
A("look_at_you", "happy", "looks straight at you", [K(LOOKU, 500)])
A("scan", "curious", "looks slowly around the room",
  [K((-70, 10, 55, 25), 900), K((-70, 10, 55, 10), 300), K((70, 10, 55, 10), 1800),
   K((70, 10, 55, 30), 300), K(STAND, 900)])
A("think", "thinking", "looks up and away, thinking it over",
  [K((25, 10, 40, -35), 800), hold((25, 10, 40, -35), 900), K((25, 10, 40, -25), 300),
   K((25, 10, 40, -40), 300), K(STAND, 700)])
A("confused", "curious", "tilts its light this way and that, puzzled",
  [K((-15, 10, 55, 10), 350), K((15, 10, 55, 40), 400), K((-15, 10, 55, 10), 400),
   K((15, 10, 55, 40), 400), hold((15, 10, 55, 40), 300), K(STAND, 450)])
A("surprised", "surprised", "jerks back upright, startled",
  [K((0, 0, 30, -10), 250), hold((0, 0, 30, -10), 500), K((0, 5, 45, 10), 400), K(STAND, 600)])
A("scared", "scared", "shrinks away low and trembles",
  [K((-50, 0, 110, 50), 450)] + osc((-50, 0, 110, 50), 3, 5, 6, 150) +
  [hold((-50, 0, 110, 50), 500), K(STAND, 900)])
A("happy", "happy", "a happy little bounce",
  [K((0, 12, 75, 15), 350), K((0, 10, 45, 30), 350), K((0, 12, 75, 15), 350), K(STAND, 350)])
A("excited", "excited", "bounces and wiggles with excitement",
  [K((-15, 12, 75, 15), 350), K((15, 10, 45, 30), 350), K((-15, 12, 75, 15), 350),
   K((15, 10, 45, 30), 350), K(STAND, 350)])
A("dance", "playful", "dances side to side with head bobs",
  [K((-30, 15, 65, 15), 450), K((30, 5, 45, 35), 450), K((-30, 15, 65, 15), 450),
   K((30, 5, 45, 35), 450), K((-30, 15, 65, 15), 450), K((30, 5, 45, 35), 450)] +
  osc((0, 10, 55, 25), 3, 14, 4, 260))
A("celebrate", "excited", "stretches up tall and spins around",
  [K(TALL, 500), K((-60, 0, 20, 40), 500), K((60, 0, 20, 40), 900), K((0, 0, 15, 30), 500)] +
  osc((0, 0, 15, 30), 3, 14, 4, 260) + [K(STAND, 600)])
A("laugh", "happy", "giggles with quick little nods",
  [K((0, 10, 50, 30), 300)] + osc((0, 10, 50, 30), 3, 7, 8, 180) + [K(STAND, 300)])
A("sad", "sad", "droops down slowly and stays low a while",
  [K((0, 20, 100, 60), 1500), hold((0, 20, 100, 60), 1500), K(STAND, 1500)])
A("sigh", "calm", "a big slow sigh",
  [K((0, 15, 75, 50), 900), hold((0, 15, 75, 50), 300), K(STAND, 900)])
A("sleepy", "sleepy", "starts nodding off, jerks awake",
  [K((0, 10, 95, 55), 1200), K((0, 10, 105, 70), 1000), K((0, 10, 70, 30), 350),
   K((0, 10, 95, 55), 1200), K(STAND, 900)])
A("yawn", "sleepy", "tips its head back in a big yawn",
  [K((0, 0, 30, -40), 1200), hold((0, 0, 30, -40), 700), K((0, 10, 60, 40), 900), K(STAND, 600)])
A("wake_up", "happy", "unfolds from rest and stretches awake",
  [K((0, 10, 90, 50), 800), K(TALL, 900), K((0, 0, 0, -20), 800), K(STAND, 800)])
A("stretch", "calm", "stretches up and twists side to side",
  [K((0, 0, 0, -25), 1000), K((-40, 0, 0, -25), 700), K((40, 0, 0, -25), 1000), K(STAND, 1000)])
A("sleep", "sleepy", "folds down and goes to sleep", [K(REST, 1500), K(SLEEP, 1000)])
A("proud", "proud", "stands tall, chin up", [K(PROUD, 700), hold(PROUD, 900), K(STAND, 700)])
A("agree", "happy", "leans in and nods along",
  [K(LEANIN, 600)] + osc(LEANIN, 3, 15, 4, 220) + [K(STAND, 600)])
A("disagree", "calm", "leans back and shakes no",
  [K((0, 0, 30, 30), 500)] + osc((0, 0, 30, 30), 0, 15, 4, 330) + [K(STAND, 600)])
A("love", "love", "sways slowly and warmly side to side",
  [K((-15, 15, 65, 30), 900), K((15, 15, 65, 30), 900), K((-15, 15, 65, 30), 900),
   K((15, 15, 65, 30), 900), K(STAND, 800)])
A("angry", "angry", "leans in and shakes sharply",
  [K((0, 25, 60, 10), 400)] + osc((0, 25, 60, 10), 0, 14, 4, 300) + [K(STAND, 500)])
A("sulk", "sad", "turns away and looks down, sulking",
  [K((-70, 10, 90, 55), 1100), hold((-70, 10, 90, 55), 1500), K(STAND, 1000)])
A("hide", "shy", "folds down and turns away to hide", [K(HIDE, 900)])
A("peekaboo", "playful", "hides, then pops up: peekaboo!",
  [K(HIDE, 700), hold(HIDE, 800), K((0, 0, 30, 20), 450), hold((0, 0, 30, 20), 300)] +
  osc((0, 0, 30, 20), 3, 14, 2, 260) + [K(STAND, 500)])
A("shiver", "scared", "shivers", [K(STAND, 300)] + osc(STAND, 3, 5, 8, 150))
A("sneeze", "surprised", "winds up and sneezes: achoo!",
  [K((0, 5, 35, -25), 700), hold((0, 5, 35, -25), 250), K((0, 10, 65, 45), 320),
   hold((0, 10, 65, 45), 400), K(STAND, 800)])
A("dizzy", "playful", "wobbles around in a dizzy circle",
  [K((-30, 15, 60, 15), 450), K((0, 25, 60, 35), 450), K((30, 15, 60, 15), 450),
   K((0, 5, 50, 5), 450), K((-30, 15, 60, 15), 450), K((0, 25, 60, 35), 450),
   K((30, 15, 60, 15), 450), K(STAND, 600)])
A("bounce", "happy", "bounces up and down",
  [K((0, 12, 75, 15), 350), K((0, 10, 45, 30), 350), K((0, 12, 75, 15), 350),
   K((0, 10, 45, 30), 350), K(STAND, 350)])
A("point_left", "calm", "leans over and points its light to the left",
  [K((-70, 35, 70, 15), 900), hold((-70, 35, 70, 15), 700), K(STAND, 900)])
A("point_right", "calm", "leans over and points its light to the right",
  [K((70, 35, 70, 15), 900), hold((70, 35, 70, 15), 700), K(STAND, 900)])
A("spotlight", "calm", "aims its light straight down at the desk and stays", [K(SPOT, 1200)])
A("reading_light", "calm", "angles its light over the desk for reading and stays", [K(DESK, 1200)])
A("attention", "proud", "snaps to attention", [K(TALL, 400), hold(TALL, 300), K(STAND, 500)])
A("relax", "calm", "settles back down to rest", [K(REST, 1500)])
A("sneak", "playful", "creeps low and slowly across the desk",
  [K((-70, 40, 105, 40), 1600), K((70, 40, 105, 40), 3000), K(STAND, 1200)])
A("tada", "excited", "ta-da! presents itself with a flourish",
  [K((0, 0, 20, 40), 400), K((-35, 0, 20, 30), 400), K((0, 0, 20, 20), 400), K(STAND, 500)])
A("shrug", "playful", "a quick shrug: no idea",
  [K((0, 8, 40, 15), 350), K((0, 10, 55, 30), 350), K((0, 8, 40, 15), 350), K(STAND, 350)])
A("hmm", "thinking", "a slow thoughtful nod", [K((0, 10, 55, 35), 500), K(STAND, 500)])
A("stand", "calm", "stands up, attentive, light forward", [K(STAND, 700)])
# used by the firmware itself, not offered to the LLM
A("listen", "curious", "leans in to listen", [K((10, 25, 65, 10), 600)], public=False)
A("talk", "happy", "small lively moves while talking",
  [K((0, 12, 57, 20), 400), K((-5, 10, 55, 28), 450), K((5, 12, 58, 22), 450), K(STAND, 400)],
  public=False)
A("idle", "calm", "tiny breathing moves while idle",
  [K((5, 12, 57, 22), 1500), K((-5, 10, 55, 28), 1500), K(STAND, 1500)], public=False)


# ------------------------------------------------------------- checking ---
def move_time(d, v, a):
    d = abs(d)
    return max(1.5 * d / v, math.sqrt(4.5 * d / a))


def check():
    problems, stretched = [], []
    starts = {"stand": STAND, "rest": REST}
    for name, mood, desc, keys, _pub in ACTIONS:
        if mood not in MOODS:
            problems.append(f"{name}: unknown mood {mood}")
        for k in keys:
            pose = k[:4]
            for v, (lo, hi), j in zip(pose, LIMITS, ("pan", "shoulder", "elbow", "head")):
                if not lo - 1e-6 <= v <= hi + 1e-6:
                    problems.append(f"{name}: {j} {v} outside {lo:.1f}..{hi:.1f}")
            ok, why = G.allowed(pose)
            if not ok:
                problems.append(f"{name}: key {pose} refused by the guard ({why})")
        for sname, s in starts.items():                 # from where the lamp usually is
            if G.plan(s, keys[0][:4]) is None:
                problems.append(f"{name}: no route from {sname} to its first key")
        prev = keys[0][:4]
        for k in keys[1:]:
            route = G.plan(prev, k[:4])
            if route is None:
                problems.append(f"{name}: no route {prev} -> {k[:4]}")
            elif len(route) > 1:
                stretched.append(f"{name}: {prev} -> {k[:4]} goes round the pod ({len(route)} legs)")
            need = max(move_time(b - a, VCAP[i], ACAP[i]) for i, (a, b) in enumerate(zip(prev, k[:4])))
            if need * 1000 > k[4] * 1.25 and need > 0.05:
                stretched.append(f"{name}: {prev} -> {k[:4]} asked {k[4]} ms, needs {need * 1000:.0f}")
            prev = k[:4]
    return problems, stretched


def emit():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    L = ["// actions.h -- GENERATED by sw/h3_actions.py. Edit there and re-run, not here.",
         "// Every key passed the pose guard and the route planner (sw/h3_guard.py, the",
         "// same code as motion.h) inside the calibrated joint range.",
         "#pragma once", "",
         "struct Key { int16_t a[4]; uint16_t ms; };",
         "struct Action { const char* name; Mood mood; const char* desc; uint8_t n;"
         " const Key* keys; bool pub; };", ""]
    for name, mood, desc, keys, pub in ACTIONS:
        body = ", ".join("{{%d,%d,%d,%d},%d}" % tuple(int(round(v)) for v in k) for k in keys)
        L.append(f"static const Key K_{name}[] = {{{body}}};")
    L.append("")
    L.append("static const Action ACTIONS[] = {")
    for name, mood, desc, keys, pub in ACTIONS:
        L.append(f'  {{"{name}", MOOD_{mood.upper()}, "{desc}", {len(keys)}, K_{name}, '
                 f'{"true" if pub else "false"}}},')
    L.append("};")
    L.append("static const int N_ACTIONS = sizeof(ACTIONS) / sizeof(ACTIONS[0]);")
    L.append("")
    open(OUT, "w", encoding="utf-8", newline="\n").write("\n".join(L) + "\n")


if __name__ == "__main__":
    problems, stretched = check()
    pub = sum(1 for a in ACTIONS if a[4])
    print(f"{len(ACTIONS)} actions ({pub} offered to the LLM), "
          f"{sum(len(a[3]) for a in ACTIONS)} keys")
    for s in stretched:
        print("  note:", s)
    for p in problems:
        print("  PROBLEM:", p)
    if problems:
        sys.exit(1)
    emit()
    print("wrote", os.path.relpath(OUT, ROOT))
