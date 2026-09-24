"""h3.py -- HOTARU 3 numbers. Every dimension of the SolidWorks rebuild lives
here; the part builders in hotaru3.py only ever read from this module.

World frame (SolidWorks convention, so its standard views make sense):
    X  lateral -- every arm joint axis runs along X
    Y  up
    Z  toward the viewer: the lamp faces +Z
Origin: pan axis, on the underside of the base.

Neutral pose = arm straight up, shade looking straight up the arm line.
Every printed part except the pins and the shade is modelled directly in
this neutral world frame, so the assembly places them with an identity
transform and a pose is nothing but joint rotations.

What changed from v2, and why (the user's complaint: the arm is not
strong enough and the servos struggle under torque):

  shoulder   GEARED 1.8 : 1. A 30T pinion on the shoulder servo drives a 54T
             gear cut on link1's own plate. The servo sees 1/1.8 of
             the arm's moment, and link1 no longer hangs on the servo's
             spline at all -- it pivots on two printed journal pins in the
             tower cheeks, so the spline only ever carries torque.
  links      BOX SECTIONS. Each link is a plate with two full-length edge
             walls printed on, closed by a second plate bolted on with M3.
             v2's links were two flat plates joined by a few printed
             standoffs -- a parallelogram that racks. A closed box does not.
  joints     printed O14 journal pins held by one M3 each, instead of
             printed threaded stubs. The load goes through a 14 mm journal,
             the bolt only keeps it from walking out.
  fasteners  M3 socket-head bolts and M3 hex nuts everywhere, nuts in
             printed hex pockets. No printed threads anywhere.
  pan        the disc rides a 10 mm-wide printed slew ring and is trapped
             by a bolted lip ring, so tipping loads never reach the pan
             servo. The servo drives the disc through the horn sitting in
             a slot pocket -- torque only, zero axial load.
  driver     a PCA9685 16-channel board in the base drives all four servos.
"""
from __future__ import annotations

import math

# ======================================================= print / fits ======
FIT = 0.3            # printed-on-printed sliding clearance, per side
RUN = 0.25           # journal running clearance, per side
M3_CLEAR = 3.4       # through-hole
M3_HEAD_D = 5.6      # DIN 912 head
M3_HEAD_H = 3.0
M3_CB_D = 6.4        # counterbore for the head
M3_NUT_AF = 5.5      # DIN 934 across flats
M3_NUT_H = 2.4
NUT_POCKET_AF = 5.9  # printed hex pocket: nut drops in, does not spin
NUT_POCKET_H = 2.7

# ============================================================ MG996R =======
# Servo frame S: origin on the output-shaft axis IN THE CASE-TOP PLANE,
# +z out of the case top along the shaft, body length along +x.
# Body numbers are the repo's (params.py); the shaft offset and tab height
# carry the same TODO-verify they always did.
SV_L, SV_W, SV_H = 40.7, 19.7, 42.9
SV_X0 = -10.0                        # shaft sits 10.0 from the near end
SV_X1 = SV_X0 + SV_L                 # 30.7
SV_XC = (SV_X0 + SV_X1) / 2          # 10.35, body centre
SV_TAB_SPAN = 53.6
SV_TAB_T = 2.5
SV_TAB_Z0 = -SV_H + 26.6             # -16.3, tab face toward the case bottom
SV_TAB_Z1 = SV_TAB_Z0 + SV_TAB_T     # -13.8, tab face toward the case top
SV_TAB_X0 = SV_XC - SV_TAB_SPAN / 2  # -16.45
SV_TAB_X1 = SV_XC + SV_TAB_SPAN / 2  # 37.15
SV_HOLE_DX, SV_HOLE_DY = 49.5, 10.0
SV_HOLES = [(SV_XC + sx * SV_HOLE_DX / 2, sy * SV_HOLE_DY / 2)
            for sx in (-1, 1) for sy in (-1, 1)]          # x -14.4 / 35.1
SV_TAB_HOLE_D = 3.6   # modelled small: the real grommet hole breaks the tab tip
SV_BOSS_D, SV_BOSS_H = 13.0, 2.0
SV_SPLINE_D, SV_SPLINE_TOP = 5.9, 4.7
# the lead leaves the far end face, low on the case
SV_LEAD_Z = (-SV_H + 2.0, -SV_H + 7.0)

# The STOCK double horn, hub toward the servo. HORN_TOP is the one number
# the whole joint stack hangs on and it is NOT measured: press a horn onto
# a servo and measure case-top to horn-face. Every stack below is built
# with 1.5 mm of float so a +/-1 mm error is absorbed (see JOINT_FLOAT).
HORN_TOP = 9.0
HORN_T = 2.5
HORN_L, HORN_W = 54.0, 8.0
HORN_HUB_D, HORN_HUB_H = 14.0, 4.3   # hub stops 0.2 above the boss
HORN_POCKET_W = HORN_W + 2 * 0.4
HORN_POCKET_L = HORN_L + 2 * 0.5
# For the pinion the horn must fit INSIDE a 30T m1.25 gear: trim both arms
# to 30 mm tip to tip with side cutters. Tips at r15 against a root at
# r17.2 leave a 2.2 mm wall.
HORN_TRIM_L = 30.0
JOINT_FLOAT = 1.0    # axial float designed into every idler pin

# ================================================== shoulder gearing =======
GEAR_M = 1.25
PINION_N, GEAR_N = 30, 54        # 1.8 : 1 -- 45T (1.5 : 1) left 37% of stall at full reach
GEAR_RATIO = GEAR_N / PINION_N                     # 1.8
GEAR_CD = GEAR_M * (PINION_N + GEAR_N) / 2 + 0.10  # 52.6, +0.1 print slack
GEAR_FACE = 6.0

# ============================================================ layout =======
# tub
TUB_R = 82.0            # O164
TUB_WALL = 3.5
TUB_FLOOR = 2.5
Y_T = 53.0              # top of the tub = slew face
LEDGE_R0 = 62.0         # slew ring, inner edge
BAND_R0 = 70.5          # rim band (nut zone) inner edge
BAND_H = 6.0            # rim band depth below Y_T
LEDGE_H = 4.0
RING_BOLT_R = 76.5
RING_BOLT_ANG = [0, 60, 120, 180, 240, 300]    # deg in XZ, x = r cos, z = r sin

# disc (turntable) and lip ring
DISC_R = 70.0
DISC_T = 6.0
Y_D = Y_T + DISC_T      # 59, disc top
RABBET_R = 65.5
RABBET_D = 2.5          # step down on the disc's top outer ring
LIP_R0 = 66.0
LIP_T = 2.2
RING_OUT_R = TUB_R

# pan servo: horn arm top sits 2.5 above the disc underside, inside a 4.0
# pocket -- 1.5 of float each way and NO axial contact: torque only
PAN_HORN_TOP = Y_T + 2.5
PAN_CASE_TOP = PAN_HORN_TOP - HORN_TOP      # 46.5
PAN_POCKET_D = 4.0

# shoulder
Y_P, Z_P = 102.5, 14.0         # pivot; 14 forward so the 54T gear's pinion clears the back wall
L1, L2 = 100.0, 90.0
HEAD_OFF = 28.0                # tilt axis beyond link2's square end
Y_E = Y_P + L1                 # elbow
Y_H = Y_E + L2 + HEAD_OFF      # head tilt axis

TOWER_BASE_T = 5.0
TOWER_Y0 = Y_D
TOWER_Y1 = Y_D + TOWER_BASE_T
CHEEK_X0, CHEEK_T = 30.0, 6.0  # cheek inner face |x|, thickness
TOWER_BACK_Z = -54.0
TOWER_FRONT_Z = 30.0
TOWER_BOLTS = [(sx * 26.0, z) for sx in (-1, 1) for z in (-44.0, 22.0)]

# shoulder servo: shaft along +X, pinion in the gear plane
SH_SHAFT_Y = Y_P
SH_SHAFT_Z = Z_P - GEAR_CD     # straight behind the pivot
PLATE_T = 5.0
# link1 plates
L1A = (22.05, 27.05)           # P_A, horn/gear side
L1B = (-27.05, -22.05)         # P_B, cover
GEAR_X = (L1A[0] - 1.0, L1A[1])  # gear rim is one wider, inward
PINION_T = 7.0
PINION_X = (GEAR_X[0], GEAR_X[0] + PINION_T)   # 21.05 .. 28.05
# horn pocket in the pinion's -X face is 2.3 deep; the horn face bears on
# its floor, so the case top sits HORN_TOP below that floor
PINION_POCKET = 2.3
SH_CASE_TOP = PINION_X[0] + PINION_POCKET - HORN_TOP        # 14.35
SH_BULK_X = (SH_CASE_TOP + SV_TAB_Z1, SH_CASE_TOP + SV_TAB_Z1 + 5.0)  # 0.55..5.55

# elbow servo inside link1, case top 0.2 off P_A's inner face
EL_CASE_TOP = L1A[0] - 0.2                                   # 21.85
# link2 plates, straddling link1
L2A = (L1A[1] + 1.5, L1A[1] + 1.5 + PLATE_T + 1.0)           # D2 28.55..34.55, 6 thick
L2A_BOSS = 0.0      # was a 1.0 raised boss: it held the plate off the print bed
L2B = (-(L2A[0] + PLATE_T), -L2A[0])                         # I2, 5 thick
# head servo in the nose
HD_CASE_TOP = 22.0
NOSE_A = (HD_CASE_TOP + 0.2, HD_CASE_TOP + 4.2)              # +X wall
NOSE_B = (HD_CASE_TOP - SV_H - 1.0 - 5.0, HD_CASE_TOP - SV_H - 1.0)  # -X cover
YOKE_A = (NOSE_A[1] + 1.5, NOSE_A[1] + 1.5 + 6.0)            # shade drive arm
YOKE_B = (NOSE_B[0] - 1.5 - 5.0, NOSE_B[0] - 1.5)            # shade idler arm

# link planform
CAP_R = 29.0
LINK_W = 36.0      # 34 let the cover-bolt counterbores break the edge by 0.2
WALL_T = 3.5
WALL_T0 = LINK_W / 2 - WALL_T                                # 13.5
PIN_D = 14.0
PIN_FLANGE_D = 22.0
PIN_FLANGE_T = 2.5

# shade
SHADE_MOUTH_D = 88.0
SHADE_BACK_D = 56.0
SHADE_LEN = 58.0
SHADE_WALL = 1.8
SHADE_BACK = 30.0          # back plate distance from the tilt axis
RING_OD, RING_ID, RING_T = 45.0, 31.7, 2.2

# journal pin shaft lengths: from the seat floor in the part that carries
# the pin to JOINT_FLOAT past the outer face of the part that turns on it
SHOULDER_PIN = (CHEEK_X0 + CHEEK_T + JOINT_FLOAT) - (L1A[1] - 1.0)     # 10.95
ELBOW_PIN = (-L2B[0] + JOINT_FLOAT) - (-L1B[0] - 1.0)                   # 8.5
HEAD_PIN = (-YOKE_B[0] + JOINT_FLOAT) - (-NOSE_B[0] - 1.0)              # 8.5

# ============================================================ boards =======
PCA_L, PCA_W, PCA_T = 62.2, 25.4, 1.6
PCA_HOLES = [(sx * 27.9, sy * 9.5) for sx in (-1, 1) for sy in (-1, 1)]
PCA_XZ = (0.0, -38.0)      # board centre in the tub (x, z)
PCA_POST = 6.0
ESP_L, ESP_W, ESP_T = 64.0, 30.0, 1.6
ESP_XZ = (13.0, 35.0)
ESP_POST = 10.0
AMP_L, AMP_W = 20.0, 18.0
AMP_XZ = (-47.0, 30.0)
AMP_POST = 7.5
SPK_L, SPK_H, SPK_T = 40.0, 20.0, 5.5
SPK_X = 61.0               # driver's inner face, standing on edge
SPK_Z0 = -24.0             # driver spans z -24 .. 16, clear of the USB well
MIC_L, MIC_W = 15.0, 12.6
MIC_X = -71.0
MIC_Y = 30.0
CAP_D, CAP_H = 16.0, 26.0
CAP_XZ = (-44.0, -44.0)
MX_CUT = 14.1
MX_PLATE_T = 1.5
MX_POD_Z = -81.5           # switch centre, on the lip ring's rear pod
MX_TOP = 67.0              # top of the 1.5 clip plate
MX_WIRE_Z = -76.5          # lead hole down through ring and rim band


# ============================================================ helpers ======
def involute_gear(m, n, pa_deg=20.0, backlash=0.10, pts_per_flank=6,
                  root_pts=2, tip_pts=2):
    """Closed polyline of a standard involute spur gear, centred on the
    origin. `backlash` thins every tooth by that much on the pitch circle.
    Root is a plain arc on the dedendum circle -- ample for a printed gear
    at this module, and it keeps the sketch small."""
    pa = math.radians(pa_deg)
    rp = m * n / 2
    rb = rp * math.cos(pa)
    ra = rp + m
    rf = rp - 1.25 * m
    inv = lambda a: math.tan(a) - a
    # half tooth angle at the pitch circle, minus backlash
    half = (math.pi / (2 * n)) - (backlash / 2) / rp
    phi_p = inv(pa)

    def flank(r):
        # polar angle of the involute point at radius r, relative to the
        # tooth centreline, for the +side flank
        if r < rb:
            r = rb
        a = math.acos(rb / r)
        return half + phi_p - inv(a)

    r_start = max(rb, rf)
    rs = [r_start + (ra - r_start) * k / (pts_per_flank - 1)
          for k in range(pts_per_flank)]
    pts = []
    pitch = 2 * math.pi / n
    for i in range(n):
        c = i * pitch
        # root arc from the previous tooth's -flank to this tooth's +... we
        # walk: -flank up, tip arc, +flank down, root arc to next tooth
        left = [(r, c - flank(r)) for r in rs]            # rising
        right = [(r, c + flank(r)) for r in reversed(rs)]  # falling
        tip0, tip1 = c - flank(ra), c + flank(ra)
        tip = [(ra, tip0 + (tip1 - tip0) * k / (tip_pts + 1))
               for k in range(1, tip_pts + 1)]
        root0 = c + flank(r_start)
        root1 = c + pitch - flank(r_start)
        root = [(rf, root0 + (root1 - root0) * k / (root_pts + 1))
                for k in range(0, root_pts + 2)]
        for r, a in left + tip + right:
            pts.append((r * math.cos(a), r * math.sin(a)))
        if rf < r_start - 1e-6:
            pts.append((rf * math.cos(root0), rf * math.sin(root0)))
        for r, a in root[1:-1]:
            pts.append((r * math.cos(a), r * math.sin(a)))
        if rf < r_start - 1e-6:
            pts.append((rf * math.cos(root1), rf * math.sin(root1)))
    # drop near-duplicates
    out = []
    for p in pts:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 1e-3:
            out.append(p)
    if math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) < 1e-3:
        out.pop()
    return out


def gear_segments(m, n, rot_deg=0.0, centre=(0.0, 0.0), pa_deg=20.0,
                  backlash=0.10, flank_pts=6):
    """The same involute gear as involute_gear(), as SKETCH SEGMENTS: one
    spline per flank, true arcs on the tip and root circles, and a short
    radial line where the root sits inside the base circle. ~4 entities per
    tooth instead of ~16 lines -- the 54T gear as lines (864 of them) made
    SolidWorks drop the API connection mid-sketch, every time."""
    pa = math.radians(pa_deg)
    rp = m * n / 2
    rb = rp * math.cos(pa)
    ra = rp + m
    rf = rp - 1.25 * m
    inv = lambda a: math.tan(a) - a
    half = (math.pi / (2 * n)) - (backlash / 2) / rp
    phi_p = inv(pa)

    def flank(r):
        r = max(r, rb)
        return half + phi_p - inv(math.acos(rb / r))

    r0 = max(rb, rf)
    rs = [r0 + (ra - r0) * k / (flank_pts - 1) for k in range(flank_pts)]
    rot = math.radians(rot_deg)
    cx, cy = centre
    P = lambda r, a: (cx + r * math.cos(a + rot), cy + r * math.sin(a + rot))
    C = (cx, cy)
    pitch = 2 * math.pi / n
    segs = []
    for i in range(n):
        c = i * pitch
        left = [P(r, c - flank(r)) for r in rs]            # rising
        right = [P(r, c + flank(r)) for r in reversed(rs)]  # falling
        if rf < r0 - 1e-6:
            segs.append(("line", P(rf, c - flank(r0)), left[0]))
        segs.append(("spline", left))
        segs.append(("arc", C, left[-1], right[0], True))   # tip
        segs.append(("spline", right))
        if rf < r0 - 1e-6:
            segs.append(("line", right[-1], P(rf, c + flank(r0))))
            a0, a1 = c + flank(r0), c + pitch - flank(r0)
        else:
            a0, a1 = c + flank(r0), c + pitch - flank(r0)
        segs.append(("arc", C, P(rf, a0), P(rf, a1), True))  # root
    return segs


def gear_radii(m, n):
    rp = m * n / 2
    return dict(pitch=rp, tip=rp + m, root=rp - 1.25 * m)


def link_outline(L, r0, r1, w, rf=6.0):
    """Planform of a link in its own (s, t) frame: a round cap of radius r0
    at s=0, one of radius r1 at s=L, joined by a body of width w, with
    concave fillets of radius rf where body meets cap. Returned as a list of
    ('arc', c, p0, p1, ccw) / ('line', p0, p1) segments, counter-clockwise.
    """
    h = w / 2

    def fillet(cx, R, side, sgn):
        # fillet circle tangent to cap circle (centre (cx,0), radius R) and
        # to the body edge t = side*h; sgn = +1 on the far side of the cap
        fc_t = side * (h + rf)
        ds = math.sqrt((R + rf) ** 2 - (h + rf) ** 2)
        fc = (cx + sgn * ds, fc_t)
        k = R / (R + rf)
        on_cap = (cx + (fc[0] - cx) * k, fc[1] * k)
        on_edge = (fc[0], side * h)
        return fc, on_cap, on_edge

    # four fillets: at cap0 (+t, -t) facing +s; at cap1 (+t, -t) facing -s
    f0p = fillet(0.0, r0, +1, +1)
    f0m = fillet(0.0, r0, -1, +1)
    f1p = fillet(L, r1, +1, -1)
    f1m = fillet(L, r1, -1, -1)
    segs = []
    # start on cap0 at the -t fillet tangent, go round the back of cap0
    # (clockwise in s,t == through s<0) to the +t fillet tangent
    segs.append(("arc", (0.0, 0.0), f0m[1], f0p[1], False))
    segs.append(("arc", f0p[0], f0p[1], f0p[2], True))     # concave fillet
    segs.append(("line", f0p[2], f1p[2]))
    segs.append(("arc", f1p[0], f1p[2], f1p[1], True))
    segs.append(("arc", (L, 0.0), f1p[1], f1m[1], False))
    segs.append(("arc", f1m[0], f1m[1], f1m[2], True))
    segs.append(("line", f1m[2], f0m[2]))
    segs.append(("arc", f0m[0], f0m[2], f0m[1], True))
    return segs


if __name__ == "__main__":
    print("shoulder case top", SH_CASE_TOP, "bulkhead", SH_BULK_X)
    print("gear CD", GEAR_CD, "ratio", GEAR_RATIO,
          "pinion", gear_radii(GEAR_M, PINION_N), "gear", gear_radii(GEAR_M, GEAR_N))
    print("link2 plates", L2A, L2B, "inner width", L2A[0] - L2B[1])
    print("nose", NOSE_A, NOSE_B, "yoke", YOKE_A, YOKE_B)
    print("elbow Y", Y_E, "head Y", Y_H)
    g = involute_gear(GEAR_M, PINION_N)
    print("pinion polyline pts", len(g))
