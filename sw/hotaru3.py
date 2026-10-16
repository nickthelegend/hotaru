"""hotaru3.py -- HOTARU 3, built natively in SolidWorks.

One function per part. Each opens a new part, draws real sketches and
features (so the FeatureManager tree reads like a hand-built part and every
dimension is editable afterwards), and returns the open Part. All numbers
come from h3.py.

Sketch frames used below:
  "wXZ"  world (x, z) on the Top plane, features extrude along +Y
  "wYZ"  world (y, z) on the Right plane, features extrude along +X
  link   (s, t) on the Right plane: s along the link from its near joint,
         t toward the lamp's front. world y = Y0 + s, z = Z_P + t.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import h3 as H
import swlib as L

OVL = 0.2

COL = {
    "shell": "#ECEAE4", "arm": "#F3F1EC", "dark": "#2B2D31", "accent": "#2C6BED",
    "servo": "#1B1C1F", "horn": "#3A3D42", "pcb": "#1E3F8F", "pcb2": "#5B2A83",
    "pcb3": "#0E5B3A", "metal": "#A9AFB7", "led": "#FFC85C", "spk": "#151515",
    "cap": "#2A6F4F", "mx": "#E6E6E6",
}


def link_xf(y0):
    """(s, t) -> Right-plane sketch coordinates for a link whose near joint
    sits at world y0 in the neutral pose."""
    return (0, -1, -H.Z_P, 1, 0, y0)


def _rot(pts, deg, c=(0.0, 0.0)):
    a = math.radians(deg)
    ca, sa = math.cos(a), math.sin(a)
    return [(c[0] + x * ca - y * sa, c[1] + x * sa + y * ca) for x, y in pts]


def link_outline_open(L, r0, w, rf=6.0, rc=4.0):
    """Cap of radius r0 at s=0, body of width w running to a square end at
    s=L with corner radius rc."""
    h = w / 2
    ds = math.sqrt((r0 + rf) ** 2 - (h + rf) ** 2)
    k = r0 / (r0 + rf)
    fp = (ds, h + rf)
    fm = (ds, -(h + rf))
    capp = (fp[0] * k, fp[1] * k)
    capm = (fm[0] * k, fm[1] * k)
    return [
        ("arc", (0.0, 0.0), capm, capp, False),
        ("arc", fp, capp, (ds, h), True),
        ("line", (ds, h), (L - rc, h)),
        ("arc", (L - rc, h - rc), (L - rc, h), (L, h - rc), False),
        ("line", (L, h - rc), (L, -h + rc)),
        ("arc", (L - rc, -h + rc), (L, -h + rc), (L - rc, -h), False),
        ("line", (L - rc, -h), (ds, -h)),
        ("arc", fm, (ds, -h), capm, True),
    ]


def nut_slot(s, c, af, toward, length):
    """Hex pocket at c=(p,q) that opens into a straight slot running `length`
    along `toward` (a unit vector), so a nut slides in from that face.

    The hexagon is turned so two of its FLATS run parallel to the slide:
    only then is the nut's width across the slot equal to AF, and a slot of
    width AF lets it through. (A flat facing the slot makes the nut AF/cos30
    wide across it and it jams at the mouth.) The outline is one closed
    polygon: four hex vertices round the back, then the slot."""
    r = af / 2 / math.cos(math.pi / 6)
    dp, dq = toward
    a0 = math.atan2(dq, dp)
    V = lambda k: (c[0] + r * math.cos(a0 + math.radians(60 * k)),
                   c[1] + r * math.sin(a0 + math.radians(60 * k)))
    nx, ny = -dq, dp
    far = (c[0] + dp * length, c[1] + dq * length)
    hw = af / 2
    pts = [V(1), V(2), V(3), V(4), V(5),
           (far[0] - nx * hw, far[1] - ny * hw),
           (far[0] + nx * hw, far[1] + ny * hw)]
    s.poly(pts)


# =================================================================== buy ===
def servo(sw):
    p = sw.new_part("MG996R")
    with p.sketch("XY") as s:
        s.crect(H.SV_XC, 0, H.SV_L, H.SV_W, 1.2)
    p.extrude(-H.SV_H, 0, "Case")
    with p.sketch("XY") as s:
        s.crect(H.SV_XC, 0, H.SV_TAB_SPAN, H.SV_W - 1.0, 1.0)
        for x, y in H.SV_HOLES:
            s.circle(x, y, H.SV_TAB_HOLE_D / 2)
    p.extrude(H.SV_TAB_Z0, H.SV_TAB_Z1, "Tabs")
    with p.sketch("XY") as s:
        s.circle(0, 0, H.SV_BOSS_D / 2)
    p.extrude(0, H.SV_BOSS_H, "Boss")
    with p.sketch("XY") as s:
        s.circle(0, 0, H.SV_SPLINE_D / 2)
    p.extrude(H.SV_BOSS_H, H.SV_SPLINE_TOP, "Spline")
    with p.sketch("XY") as s:
        s.circle(0, 0, 1.25)
    p.cut(H.SV_SPLINE_TOP - 4.0, H.SV_SPLINE_TOP + 0.1, "M3 tap")
    with p.sketch("YZ") as s:     # lead: (u, v) = (-z, y) on the Right plane
        s.crect(-(H.SV_LEAD_Z[0] + H.SV_LEAD_Z[1]) / 2, 0, H.SV_LEAD_Z[1] - H.SV_LEAD_Z[0], 7.0)
    p.extrude(H.SV_X1 - 0.2, H.SV_X1 + 5.0, "Lead")
    p.colour(COL["servo"])
    return p


def horn(sw, length=H.HORN_L, name="Horn"):
    p = sw.new_part(name)
    z1 = H.HORN_TOP - H.HORN_T
    with p.sketch("XY") as s:
        s.circle(0, 0, H.HORN_HUB_D / 2)
        s.circle(0, 0, H.SV_SPLINE_D / 2 + 0.1)       # splined bore
    p.extrude(z1 - H.HORN_HUB_H, z1, "Hub")     # touching faces merge in SW
    with p.sketch("XY") as s:
        a = (length - H.HORN_W) / 2
        s.slot(-a, 0, a, 0, H.HORN_W)
        s.circle(0, 0, 1.6)
        for sx in (-1, 1):
            for k in range(3 if length > 40 else 1):
                s.circle(sx * (10.0 + 5.0 * k), 0, 0.8)
    p.extrude(z1, H.HORN_TOP, "Arm")
    p.colour(COL["horn"])
    return p


def pca9685(sw):
    p = sw.new_part("PCA9685")
    y0 = H.TUB_FLOOR + H.PCA_POST
    cx, cz = H.PCA_XZ
    with p.sketch("wXZ") as s:
        s.crect(cx, cz, H.PCA_L, H.PCA_W, 1.0)
        for hx, hz in H.PCA_HOLES:
            s.circle(cx + hx, cz + hz, 1.25)
    p.extrude(y0, y0 + H.PCA_T, "Board")
    with p.sketch("wXZ") as s:      # 16 x 3-pin servo headers on the inboard edge
        s.crect(cx, cz + H.PCA_W / 2 - 5.2, 40.6, 7.6)
    p.extrude(y0 + H.PCA_T, y0 + H.PCA_T + 11.5, "Servo headers")
    with p.sketch("wXZ") as s:      # V+ screw terminal
        s.crect(cx, cz - H.PCA_W / 2 + 4.5, 10.0, 7.5)
    p.extrude(y0 + H.PCA_T, y0 + H.PCA_T + 10.0, "V+ terminal")
    with p.sketch("wXZ") as s:
        for sx in (-1, 1):
            s.crect(cx + sx * (H.PCA_L / 2 - 3.0), cz, 2.5, 15.2)
    p.extrude(y0 + H.PCA_T, y0 + H.PCA_T + 8.5, "I2C headers")
    p.colour(COL["pcb"])
    return p


def esp32(sw):
    p = sw.new_part("ESP32-S3")
    y0 = H.TUB_FLOOR + H.ESP_POST
    cx, cz = H.ESP_XZ
    with p.sketch("wXZ") as s:
        s.crect(cx, cz, H.ESP_L, H.ESP_W, 1.0)
    p.extrude(y0, y0 + H.ESP_T, "Board")
    with p.sketch("wXZ") as s:
        s.crect(cx - H.ESP_L / 2 + 13.0, cz, 18.0, 25.5)
    p.extrude(y0 + H.ESP_T, y0 + H.ESP_T + 3.1, "WROOM")
    with p.sketch("wXZ") as s:
        for dz in (-8.53, 8.53):
            s.crect(cx + H.ESP_L / 2 - 3.2, cz + dz, 7.3, 8.94, 1.0)
    p.extrude(y0 + H.ESP_T, y0 + H.ESP_T + 3.3, "USB-C")
    with p.sketch("wXZ") as s:
        for dz in (-13.4, 13.4):
            s.crect(cx, cz + dz, 58.4, 2.5)
    p.extrude(y0 - 8.5, y0, "Headers")
    p.colour(COL["pcb3"])
    return p


def amp(sw):
    p = sw.new_part("MAX98357A")
    y0 = H.TUB_FLOOR + H.AMP_POST
    cx, cz = H.AMP_XZ
    with p.sketch("wXZ") as s:
        s.crect(cx, cz, H.AMP_L, H.AMP_W, 1.0)
    p.extrude(y0, y0 + 1.6, "Board")
    with p.sketch("wXZ") as s:
        s.crect(cx, cz + 4.0, 10.0, 7.5)
    p.extrude(y0 + 1.6, y0 + 11.6, "Terminal")
    p.colour(COL["pcb2"])
    return p


def speaker(sw):
    p = sw.new_part("Speaker2040")
    z0 = H.SPK_Z0
    with p.sketch("wYZ") as s:
        s.crect(H.TUB_FLOOR + H.SPK_H / 2 + 0.3, z0 + H.SPK_L / 2, H.SPK_H, H.SPK_L, 3.0)
    p.extrude(H.SPK_X, H.SPK_X + H.SPK_T, "Driver")
    p.colour(COL["spk"])
    return p


def mic(sw):
    p = sw.new_part("INMP441")
    with p.sketch("wYZ") as s:
        s.circle(H.MIC_Y, 0, 7.5)
    p.extrude(H.MIC_X - 0.8, H.MIC_X + 0.8, "Board")
    p.colour(COL["pcb2"])
    return p


def bulkcap(sw):
    p = sw.new_part("Cap2200uF")
    with p.sketch("wXZ") as s:
        s.circle(H.CAP_XZ[0], H.CAP_XZ[1], H.CAP_D / 2)
    p.extrude(H.TUB_FLOOR + 0.5, H.TUB_FLOOR + 0.5 + H.CAP_H, "Can")
    p.colour("#1F3A6B")
    return p


def mx_switch(sw):
    p = sw.new_part("CherryMX")
    zc = H.MX_POD_Z
    top = H.MX_TOP
    with p.sketch("wXZ") as s:
        s.crect(0, zc, 15.6, 15.6, 0.6)
    p.extrude(top, top + 6.2, "Upper")
    with p.sketch("wXZ") as s:
        s.crect(0, zc, 13.9, 13.9, 0.4)
    p.extrude(top - H.MX_PLATE_T, top + OVL, "Neck")
    with p.sketch("wXZ") as s:
        s.crect(0, zc, 15.0, 15.0, 0.6)
    p.extrude(top - H.MX_PLATE_T - 6.5, top - H.MX_PLATE_T, "Body")
    with p.sketch("wXZ") as s:
        s.crect(0, zc, 7.2, 7.2, 0.4)
    p.extrude(top + 6.2, top + 9.8, "Stem")
    p.colour(COL["mx"])
    return p


def led_ring(sw):
    p = sw.new_part("WS2812-ring")
    y0 = H.SHADE_BACK + 2.0
    with p.sketch("wXZ") as s:
        s.ring(0, 0, H.RING_OD / 2, H.RING_ID / 2)
    p.extrude(y0, y0 + 1.6, "PCB")
    with p.sketch("wXZ") as s:
        # 5050 packages turned to follow the ring, the way they are placed
        rr = (H.RING_OD + H.RING_ID) / 4
        sq = [(-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5)]
        for k in range(12):
            s.poly(_rot([(x + rr, y) for x, y in sq], 30 * k))
    p.extrude(y0 + 1.6, y0 + 1.6 + 1.6, "LEDs")
    p.colour(COL["led"])
    return p


# ================================================================ printed ===
def tub(sw):
    p = sw.new_part("h3-tub")
    # The slew ring juts 16.5 mm in from the wall. With a flat underside it
    # printed in mid-air and the slicer filled the tub with tree supports
    # (the user's print, 2026-10-01: a mess, and hours of it). A 45 deg cone
    # under it grows the ring out of the wall instead -- no supports. The
    # top of the ring (what the disc and lip ring sit on) is unchanged. The
    # bottom edge chamfer is 50 deg for the same reason.
    drop = H.TUB_R - H.TUB_WALL - H.LEDGE_R0
    prof = [(0, 0), (H.TUB_R - 2.5, 0), (H.TUB_R, 3.0), (H.TUB_R, H.Y_T),
            (H.LEDGE_R0, H.Y_T), (H.LEDGE_R0, H.Y_T - H.LEDGE_H),
            (H.TUB_R - H.TUB_WALL, H.Y_T - H.LEDGE_H - drop),
            (H.TUB_R - H.TUB_WALL, H.TUB_FLOOR), (0, H.TUB_FLOOR)]
    with p.sketch("XY") as s:
        s.cline(0, -5, 0, 80)
        s.poly(prof)
    p.revolve(360, name="Shell + slew ring")

    # ---- pan servo pillars: M3 through each tab hole into a nut slot
    tab_under = H.PAN_CASE_TOP + H.SV_TAB_Z0                    # 30.2
    posts = [(-19.4, -10.4, -1), (31.1, 40.1, +1)]              # x0, x1, outward
    with p.sketch("wXZ") as s:
        for x0, x1, _o in posts:
            s.crect((x0 + x1) / 2, 0, x1 - x0, 22.0, 1.5)
    p.extrude(H.TUB_FLOOR - OVL, tab_under, "Pan servo pillars")
    holes = [(hx, hz) for hx in (-14.4, 35.1) for hz in (-5.0, 5.0)]
    with p.sketch("wXZ") as s:
        for hx, hz in holes:
            s.circle(hx, hz, H.M3_PILOT / 2)
    # M3x10 through the servo tab threads straight into the pillar (7.5 mm of
    # thread). The sideways nut slots these replace had ceilings open to one
    # side -- Bambu Studio called each one a floating cantilever.
    p.cut(tab_under - 13.0, tab_under + OVL, "Pillar pilot holes")
    with p.sketch("wXZ") as s:          # the lead leaves the far end, low
        s.crect(35.6, 0, 10.2, 9.0)
    p.cut(H.TUB_FLOOR - OVL, 13.0, "Servo lead notch")

    # ---- boards
    pcx, pcz = H.PCA_XZ
    with p.sketch("wXZ") as s:
        for hx, hz in H.PCA_HOLES:
            s.circle(pcx + hx, pcz + hz, 3.0)
    p.extrude(H.TUB_FLOOR - OVL, H.TUB_FLOOR + H.PCA_POST, "PCA9685 posts")
    with p.sketch("wXZ") as s:
        for hx, hz in H.PCA_HOLES:
            s.circle(pcx + hx, pcz + hz, 1.1)
    p.extrude(H.TUB_FLOOR + H.PCA_POST - OVL, H.TUB_FLOOR + H.PCA_POST + 1.9,
              "PCA9685 locating pins")
    ex, ez = H.ESP_XZ
    with p.sketch("wXZ") as s:
        for ox in (-29.5, 26.5):
            for oz in (-8.5, 8.5):
                s.circle(ex + ox, ez + oz, 2.7)
    p.extrude(H.TUB_FLOOR - OVL, H.TUB_FLOOR + H.ESP_POST, "ESP32 posts")
    with p.sketch("wXZ") as s:          # edge fences on the long sides only
        for sx in (-1, 1):
            for sz in (-1, 1):
                s.rect(ex + sx * (H.ESP_L / 2 - 6.0), ez + sz * (H.ESP_W / 2 + 0.4),
                       ex + sx * (H.ESP_L / 2 - 0.5), ez + sz * (H.ESP_W / 2 + 2.2))
    p.extrude(H.TUB_FLOOR - OVL, H.TUB_FLOOR + H.ESP_POST + 3.0, "ESP32 fences")
    ax, az = H.AMP_XZ
    with p.sketch("wXZ") as s:
        for ox in (-7.5, 7.5):
            for oz in (-2.5, 6.0):
                s.circle(ax + ox, az + oz, 2.7)
    p.extrude(H.TUB_FLOOR - OVL, H.TUB_FLOOR + H.AMP_POST, "Amp posts")
    with p.sketch("wXZ") as s:
        for sx in (-1, 1):
            for sz in (-1, 1):
                s.rect(ax + sx * (H.AMP_L / 2 + 0.4), az + sz * (H.AMP_W / 2 - 5.0),
                       ax + sx * (H.AMP_L / 2 + 2.2), az + sz * (H.AMP_W / 2))
    p.extrude(H.TUB_FLOOR - OVL, H.TUB_FLOOR + H.AMP_POST + 3.0, "Amp fences")
    # speaker: two grooved rails, the driver slides down between them
    z0, z1 = H.SPK_Z0, H.SPK_Z0 + H.SPK_L
    with p.sketch("wXZ") as s:
        s.rect(H.SPK_X - 3.0, z1 - 1.0, H.SPK_X + H.SPK_T + 3.0, z1 + 2.0)
        s.rect(H.SPK_X - 3.0, z0 - 2.0, H.SPK_X + H.SPK_T + 3.0, z0 + 1.0)
    p.extrude(H.TUB_FLOOR - OVL, H.TUB_FLOOR + 24.0, "Speaker rails")
    with p.sketch("wXZ") as s:
        s.rect(H.SPK_X - 0.3, z1 - 1.1, H.SPK_X + H.SPK_T + 0.3, z1 + 0.4)
        s.rect(H.SPK_X - 0.3, z0 - 0.4, H.SPK_X + H.SPK_T + 0.3, z0 + 1.1)
    p.cut(H.TUB_FLOOR + 1.0, H.TUB_FLOOR + 24.0 + OVL, "Speaker grooves")
    # mic: a block on the -X wall with a slot the round board drops into
    with p.sketch("wXZ") as s:
        s.rect(-79.5, -10.0, H.MIC_X + 5.0, 10.0)
    p.extrude(H.TUB_FLOOR - OVL, H.MIC_Y + 9.0, "Mic block")
    with p.sketch("wXZ") as s:
        s.rect(H.MIC_X - 0.95, -7.8, H.MIC_X + 0.95, 7.8)
    p.cut(H.MIC_Y - 8.0, H.MIC_Y + 9.0 + OVL, "Mic slot")
    with p.sketch("wYZ") as s:
        s.circle(H.MIC_Y, 0, 1.25)
    p.cut(-90.0, H.MIC_X, "Mic port")
    # bulk capacitor clip
    with p.sketch("wXZ") as s:
        s.ring(H.CAP_XZ[0], H.CAP_XZ[1], H.CAP_D / 2 + 1.8, H.CAP_D / 2 + 0.3)
    p.extrude(H.TUB_FLOOR - OVL, H.TUB_FLOOR + 12.0, "Cap clip")

    # ---- openings through the wall
    with p.sketch("wYZ") as s:          # speaker grille
        for k in range(7):
            zc = H.SPK_Z0 + 6.0 + k * (H.SPK_L - 12.0) / 6
            s.slot(7.0, zc, 21.0, zc, 2.2)
    p.cut(66.0, 90.0, "Speaker grille")
    with p.sketch("wYZ") as s:          # USB well
        # one opening per USB-C port, 13 wide (a plug's overmold), with a
        # 45 deg pointed top: a flat 27 mm top was a floating cantilever
        y0, y1 = H.TUB_FLOOR + H.ESP_POST - 1.0, H.TUB_FLOOR + H.ESP_POST + 10.0
        for dz in (-8.53, 8.53):
            zc = H.ESP_XZ[1] + dz
            s.poly([(y0, zc - 6.5), (y1, zc - 6.5), (y1 + 6.5, zc),
                    (y1, zc + 6.5), (y0, zc + 6.5)])
    p.cut(H.ESP_XZ[0] + H.ESP_L / 2 + 1.0, 90.0, "USB windows")
    with p.sketch("XY") as s:           # back vents
        for k in range(-5, 6):
            if k == 0:
                continue
            s.slot(k * 7.0, 9.0, k * 7.0, 34.0, 2.6)
    p.cut(-90.0, -60.0, "Back vents")

    # ---- lip-ring bolts: nuts pressed up into the rim band from inside
    pts = [(H.RING_BOLT_R * math.cos(math.radians(a)),
            H.RING_BOLT_R * math.sin(math.radians(a))) for a in H.RING_BOLT_ANG]
    with p.sketch("wXZ") as s:
        for x, z in pts:
            s.circle(x, z, H.M3_PILOT / 2)
    # The cone under the slew ring is solid PLA, so the lip ring's M3x12s
    # thread straight into it (6 mm of thread each). Same positions as the
    # lip ring's printed holes; no nuts inside the tub to fiddle with.
    p.cut(H.Y_T - 12.0, H.Y_T + OVL, "Ring pilot holes")
    with p.sketch("wXZ") as s:
        s.circle(0, H.MX_WIRE_Z, 3.5)
    p.cut(30.0, H.Y_T + OVL, "MX lead hole")
    p.colour(COL["shell"])
    return p


def lipring(sw):
    p = sw.new_part("h3-lipring")
    tip_r = H.DISC_R - 4.0 + 0.45
    prof = [(H.BAND_R0, H.Y_T), (H.TUB_R, H.Y_T), (H.TUB_R, H.Y_D),
            (tip_r, H.Y_D), (H.BAND_R0, H.Y_D - (H.BAND_R0 - tip_r))]
    with p.sketch("XY") as s:
        s.cline(0, 40, 0, 70)
        s.poly(prof)
    p.revolve(360, name="Lip ring")
    with p.sketch("wXZ") as s:
        s.crect(0, H.MX_POD_Z, 24.0, 21.0, 4.0)
    p.extrude(H.Y_T, H.MX_TOP, "MX pod")
    with p.sketch("wXZ") as s:
        s.crect(0, H.MX_POD_Z, H.MX_CUT, H.MX_CUT, 0.4)
    p.cut(H.MX_TOP - H.MX_PLATE_T, H.MX_TOP + OVL, "MX cutout")
    with p.sketch("wXZ") as s:
        s.crect(0, H.MX_POD_Z, 16.0, 16.0, 0.5)
    p.cut(H.Y_T + 2.0, H.MX_TOP - H.MX_PLATE_T, "MX relief")
    with p.sketch("wXZ") as s:
        s.circle(0, H.MX_WIRE_Z, 3.5)
    p.cut(H.Y_T - OVL, H.Y_T + 2.0 + OVL, "MX lead hole")
    with p.sketch("wXZ") as s:
        for a in H.RING_BOLT_ANG:
            s.circle(H.RING_BOLT_R * math.cos(math.radians(a)),
                     H.RING_BOLT_R * math.sin(math.radians(a)), H.M3_CLEAR / 2)
    p.cut(H.Y_T - OVL, H.Y_D + OVL, "Bolt holes")
    p.colour(COL["dark"])
    return p


def _kidney(s):
    r0, r1, a0, a1 = 34.0, 46.0, 220.0, 320.0
    P = lambda r, a: (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
    s.arc(0, 0, *P(r1, a0), *P(r1, a1), ccw=True)
    s.arc(*P(40.0, a1), *P(r1, a1), *P(r0, a1), ccw=True)
    s.arc(0, 0, *P(r0, a1), *P(r0, a0), ccw=False)
    s.arc(*P(40.0, a0), *P(r0, a0), *P(r1, a0), ccw=True)


def disc(sw):
    p = sw.new_part("h3-disc")
    prof = [(0, H.Y_T), (H.DISC_R, H.Y_T), (H.DISC_R, H.Y_T + 2.0),
            (H.DISC_R - 4.0, H.Y_D), (0, H.Y_D)]
    with p.sketch("XY") as s:
        s.cline(0, 40, 0, 70)
        s.poly(prof)
    p.revolve(360, name="Turntable")
    with p.sketch("wXZ") as s:
        a = (H.HORN_POCKET_L - H.HORN_POCKET_W) / 2
        s.slot(-a, 0, a, 0, H.HORN_POCKET_W)
    p.cut(H.Y_T - OVL, H.Y_T + H.PAN_POCKET_D, "Horn dog pocket")
    # the hub's top IS the arm's underside, flush with the disc's -- so the
    # disc sat ON the hub and the "torque only" drive carried the disc's
    # weight. 1.5 of relief restores the float the pocket was sized for.
    with p.sketch("wXZ") as s:
        s.circle(0, 0, H.HORN_HUB_D / 2 + 1.0)
    p.cut(H.Y_T - OVL, H.Y_T + 1.5, "Hub relief")
    with p.sketch("wXZ") as s:
        s.circle(0, 0, 4.0)
        _kidney(s)
    p.cut(H.Y_T - OVL, H.Y_D + OVL, "Centre + cable kidney")
    with p.sketch("wXZ") as s:
        for x, z in H.TOWER_BOLTS:
            s.circle(x, z, H.M3_CLEAR / 2)
    p.cut(H.Y_T - OVL, H.Y_D + OVL, "Tower bolt holes")
    with p.sketch("wXZ") as s:
        for x, z in H.TOWER_BOLTS:
            s.hexagon(x, z, H.NUT_POCKET_AF)
    p.cut(H.Y_T - OVL, H.Y_T + H.NUT_POCKET_H, "Tower nut pockets")
    p.colour(COL["shell"])
    return p


def tower(sw):
    p = sw.new_part("h3-tower")
    y0, y1 = H.TOWER_Y0, H.TOWER_Y1
    with p.sketch("wXZ") as s:
        pts = [(-36, H.TOWER_BACK_Z), (36, H.TOWER_BACK_Z), (36, H.TOWER_FRONT_Z),
               (-36, H.TOWER_FRONT_Z)]
        s.path(pts, [6, 6, 10, 10])
    p.extrude(y0, y1, "Base plate")
    with p.sketch("wXZ") as s:
        for x, z in H.TOWER_BOLTS:
            s.circle(x, z, H.M3_CLEAR / 2)
        _kidney(s)
    p.cut(y0 - OVL, y1 + OVL, "Bolt holes + cable kidney")

    # cheeks: a tapered post each side with a round boss on the pivot
    YP, ZP = H.Y_P, H.Z_P
    cheek = [(y1 - OVL, H.TOWER_BACK_Z), (y1 - OVL, H.TOWER_FRONT_Z - 2.0),
             (YP - 2.0, ZP + 14.0), (YP + 16.0, ZP - 4.0), (YP + 22.0, ZP - 30.0),
             (YP + 22.0, H.TOWER_BACK_Z)]
    for sx in (-1, 1):
        x0 = sx * H.CHEEK_X0
        x1 = sx * (H.CHEEK_X0 + H.CHEEK_T)
        with p.sketch("wYZ") as s:
            s.path(cheek, [0, 6, 10, 8, 6, 0])
        p.extrude(min(x0, x1), max(x0, x1), f"Cheek {'+X' if sx > 0 else '-X'}")
        with p.sketch("wYZ") as s:
            s.circle(YP, ZP, 17.0)
        p.extrude(min(x0, x1), max(x0, x1), f"Pivot boss {'+X' if sx > 0 else '-X'}")
    with p.sketch("wYZ") as s:
        s.circle(YP, ZP, H.PIN_D / 2 + H.RUN)
    p.cut(-H.CHEEK_X0 - H.CHEEK_T - 1, H.CHEEK_X0 + H.CHEEK_T + 1, "Journal bores")

    # back wall, notched where the pinion's teeth pass
    with p.sketch("XY") as s:
        s.rect(-36.0, y1 - OVL, 36.0, YP + 22.0)
    p.extrude(H.TOWER_BACK_Z, H.TOWER_BACK_Z + 4.0, "Back wall")
    with p.sketch("XY") as s:
        s.rect(H.PINION_X[0] - 0.8, H.SH_SHAFT_Y - 22.0, H.PINION_X[1] + 1.4, YP + 23.0)
    p.cut(H.TOWER_BACK_Z - 1.0, H.TOWER_BACK_Z + 5.0, "Pinion notch")

    # servo bulkhead: the tabs bear on its -X face, nuts on its +X face
    bx0, bx1 = H.SH_BULK_X
    zs = H.SH_SHAFT_Z
    win_y = (H.Y_P - H.SV_X1 - 0.4, H.Y_P - H.SV_X0 + 0.4)
    win_z = (zs - H.SV_W / 2 - 0.4, zs + H.SV_W / 2 + 0.4)
    with p.sketch("wYZ") as s:
        s.rect(y1 - OVL, H.TOWER_BACK_Z + 3.8, YP + 20.0, -22.0)
        s.rect(win_y[0], win_z[0], win_y[1], win_z[1])
    p.extrude(bx0, bx1, "Servo bulkhead")
    tabs = [(H.Y_P - hx, zs - hy) for hx, hy in H.SV_HOLES]
    with p.sketch("wYZ") as s:
        for y, z in tabs:
            s.circle(y, z, H.M3_CLEAR / 2)
    p.cut(bx0 - OVL, bx1 + OVL, "Tab bolt holes")
    with p.sketch("wYZ") as s:
        for y, z in tabs:
            s.hexagon(y, z, H.NUT_POCKET_AF)
    p.cut(bx1 - H.NUT_POCKET_H, bx1 + OVL, "Tab nut pockets")
    # link1 leaning back 30 deg swept its rear wall through this corner
    # (23.5 mm3 at the lean-back pose). The upper tab nut sits behind the
    # chamfer line with 3.9 to spare.
    with p.sketch("wYZ") as s:
        s.poly([(YP + 6.0, -21.8), (YP + 20.2, -21.8), (YP + 20.2, -30.0)])
    p.cut(bx0 - OVL, bx1 + OVL, "Bulkhead chamfer")
    p.colour(COL["shell"])
    return p


def pinion(sw):
    p = sw.new_part("h3-pinion")
    c = (H.SH_SHAFT_Y, H.SH_SHAFT_Z)
    g = H.gear_segments(H.GEAR_M, H.PINION_N, 90.0 - 180.0 / H.PINION_N, c)
    with p.sketch("wYZ") as s:                     # a tooth SPACE faces the gear
        s.segments(g)
    p.extrude(*H.PINION_X, name="Pinion 30T m1.25")
    with p.sketch("wYZ") as s:
        a = (H.HORN_TRIM_L + 1.0 - H.HORN_POCKET_W) / 2
        s.slot(c[0] - a, c[1], c[0] + a, c[1], H.HORN_POCKET_W)
    p.cut(H.PINION_X[0] - OVL, H.PINION_X[0] + H.PINION_POCKET, "Trimmed-horn pocket")
    with p.sketch("wYZ") as s:
        s.circle(*c, H.M3_CLEAR / 2)
    p.cut(H.PINION_X[0] - OVL, H.PINION_X[1] + OVL, "M3 to shaft")
    with p.sketch("wYZ") as s:
        s.circle(*c, H.M3_CB_D / 2)
    p.cut(H.PINION_X[1] - 2.5, H.PINION_X[1] + OVL, "Head counterbore")
    p.colour(COL["accent"])
    return p


# ------------------------------------------------------------------ links ---
def _wall_stations(s_list, t_in):
    return [(s, sg * 14.0) for s in s_list for sg in (-1, 1)]


def link1(sw):
    p = sw.new_part("h3-link1")
    xf = link_xf(H.Y_P)
    gr = H.gear_radii(H.GEAR_M, H.GEAR_N)
    ax0, ax1 = H.L1A
    with p.sketch("YZ", xf=xf) as s:
        s.segments(H.link_outline(H.L1, gr["root"] - 0.4, H.CAP_R, H.LINK_W))
        s.circle(H.L1, 0, 8.0)
    p.extrude(ax0, ax1, "Plate A")
    g = H.gear_segments(H.GEAR_M, H.GEAR_N, -90.0)   # a TOOTH faces the pinion
    with p.sketch("YZ", xf=xf) as s:
        s.segments(g)
    p.extrude(*H.GEAR_X, name="Shoulder gear 54T m1.25")
    wx = (H.L1B[1], ax0 + OVL)
    with p.sketch("YZ", xf=xf) as s:
        for sg in (-1, 1):
            s.rect(8.0, sg * H.WALL_T0, H.L1 + 8.0, sg * H.LINK_W / 2)
    p.extrude(*wx, name="Box walls")
    stations = [20.0, 55.0, 100.0]
    with p.sketch("YZ", xf=xf) as s:
        for st in stations:
            for sg in (-1, 1):
                s.rect(st - 5.0, sg * 11.0, st + 5.0, sg * H.WALL_T0)
    p.extrude(*wx, name="Wall bosses")
    # elbow servo tab ledges
    lx0 = H.EL_CASE_TOP + H.SV_TAB_Z1
    ledges = [H.L1 - 35.1, H.L1 + 14.4]
    with p.sketch("YZ", xf=xf) as s:
        for sc in ledges:
            s.rect(sc - 4.0, -10.5, sc + 4.0, 10.5)
    p.extrude(lx0, ax0 + OVL, "Elbow servo ledges")

    # cuts
    with p.sketch("YZ", xf=xf) as s:
        s.circle(0, 0, H.PIN_D / 2 + 0.25)
    p.cut(ax1 - 1.0, ax1 + OVL, "Shoulder pin seat")
    with p.sketch("YZ", xf=xf) as s:
        s.circle(0, 0, H.M3_CLEAR / 2)
    p.cut(H.GEAR_X[0] - OVL, ax1 + OVL, "Shoulder pin bolt")
    with p.sketch("YZ", xf=xf) as s:
        s.hexagon(0, 0, H.NUT_POCKET_AF)
    p.cut(H.GEAR_X[0] - OVL, H.GEAR_X[0] + H.NUT_POCKET_H, "Shoulder pin nut")
    with p.sketch("YZ", xf=xf) as s:
        s.circle(H.L1, 0, 29.5)
    p.cut(ax1 - 1.0, ax1 + OVL, "Elbow horn clearance")
    with p.sketch("YZ", xf=xf) as s:
        for st, t in _wall_stations(stations, 14.0):
            s.circle(st, t, H.M3_CLEAR / 2)
    p.cut(H.L1B[1] - OVL, H.L1B[1] + 10.0, "Cover bolt holes")
    with p.sketch("YZ", xf=xf) as s:
        for st, t in _wall_stations(stations, 14.0):
            nut_slot(s, (st, t), H.NUT_POCKET_AF, (0, -math.copysign(1, t)), 3.4)
    p.cut(-18.4, -15.7, "Cover nut slots")
    with p.sketch("YZ", xf=xf) as s:
        for sc in ledges:
            for t in (-5.0, 5.0):
                s.circle(sc, t, H.M3_CLEAR / 2)
    p.cut(lx0 - OVL, lx0 + 9.0, "Tab bolt holes")
    with p.sketch("YZ", xf=xf) as s:
        for sc, d in zip(ledges, (-1, 1)):
            for t in (-5.0, 5.0):
                nut_slot(s, (sc, t), H.NUT_POCKET_AF, (d, 0), 4.5)
    p.cut(lx0 + 3.5, lx0 + 3.5 + H.NUT_POCKET_H, "Tab nut slots")
    p.colour(COL["arm"])
    return p


def link1_cover(sw):
    p = sw.new_part("h3-link1-cover")
    xf = link_xf(H.Y_P)
    bx0, bx1 = H.L1B
    with p.sketch("YZ", xf=xf) as s:
        s.segments(H.link_outline(H.L1, H.CAP_R, H.CAP_R, H.LINK_W))
    p.extrude(bx0, bx1, "Plate B")
    with p.sketch("YZ", xf=xf) as s:
        for sc in (0.0, H.L1):
            s.circle(sc, 0, H.PIN_D / 2 + 0.25)
    p.cut(bx0 - OVL, bx0 + 1.0, "Pin seats")
    with p.sketch("YZ", xf=xf) as s:
        for sc in (0.0, H.L1):
            s.circle(sc, 0, H.M3_CLEAR / 2)
        for st, t in _wall_stations([20.0, 55.0, 100.0], 14.0):
            s.circle(st, t, H.M3_CLEAR / 2)
    p.cut(bx0 - OVL, bx1 + OVL, "Bolt holes")
    with p.sketch("YZ", xf=xf) as s:
        for sc in (0.0, H.L1):
            s.hexagon(sc, 0, H.NUT_POCKET_AF)
    p.cut(bx1 - H.NUT_POCKET_H, bx1 + OVL, "Pin nuts")
    with p.sketch("YZ", xf=xf) as s:
        for st, t in _wall_stations([20.0, 55.0, 100.0], 14.0):
            s.circle(st, t, H.M3_CB_D / 2)
    p.cut(bx0 - OVL, bx0 + H.M3_HEAD_H, "Counterbores")
    p.colour(COL["arm"])
    return p


L2_STATIONS = [42.0, 58.0]   # walls start at 36: at 31 they grazed link1 at 120 deg
HEAD_MOUNT = [(H.L2 - 18.0, -7.0), (H.L2 - 18.0, 7.0)]


def link2(sw):
    p = sw.new_part("h3-link2")
    xf = link_xf(H.Y_E)
    ax0, ax1 = H.L2A
    with p.sketch("YZ", xf=xf) as s:
        s.segments(link_outline_open(H.L2, H.CAP_R, H.LINK_W))
    p.extrude(ax0, ax1, "Plate D")
    wx = (H.L2B[1], ax0 + OVL)
    with p.sketch("YZ", xf=xf) as s:
        for sg in (-1, 1):
            s.rect(36.0, sg * H.WALL_T0, H.L2 - 26.0, sg * H.LINK_W / 2)
    p.extrude(*wx, name="Box walls")
    with p.sketch("YZ", xf=xf) as s:
        for st in L2_STATIONS:
            for sg in (-1, 1):
                s.rect(st - 5.0, sg * 11.0, st + 5.0, sg * H.WALL_T0)
    p.extrude(*wx, name="Wall bosses")
    horn_face = H.EL_CASE_TOP + H.HORN_TOP
    with p.sketch("YZ", xf=xf) as s:
        a = (H.HORN_POCKET_L - H.HORN_POCKET_W) / 2
        s.slot(-a, 0, a, 0, H.HORN_POCKET_W)
    p.cut(ax0 - OVL, horn_face, "Horn pocket")
    with p.sketch("YZ", xf=xf) as s:
        s.circle(0, 0, H.M3_CLEAR / 2)
        for sc, t in HEAD_MOUNT:
            s.circle(sc, t, H.M3_CLEAR / 2)
    p.cut(ax0 - OVL, ax1 + H.L2A_BOSS + OVL, "Bolt holes")
    with p.sketch("YZ", xf=xf) as s:
        s.circle(0, 0, H.M3_CB_D / 2)
    p.cut(ax1 + H.L2A_BOSS - 2.5, ax1 + H.L2A_BOSS + OVL, "Hub counterbore")
    with p.sketch("YZ", xf=xf) as s:
        for sc, t in HEAD_MOUNT:
            s.circle(sc, t, H.M3_CB_D / 2)
    p.cut(ax1 - H.M3_HEAD_H, ax1 + OVL, "Head-mount counterbores")
    with p.sketch("YZ", xf=xf) as s:
        for st, t in _wall_stations(L2_STATIONS, 14.0):
            s.circle(st, t, H.M3_CLEAR / 2)
    p.cut(H.L2B[1] - OVL, H.L2B[1] + 10.0, "Cover bolt holes")
    with p.sketch("YZ", xf=xf) as s:
        for st, t in _wall_stations(L2_STATIONS, 14.0):
            nut_slot(s, (st, t), H.NUT_POCKET_AF, (0, -math.copysign(1, t)), 3.4)
    p.cut(-24.9, -22.2, "Cover nut slots")
    p.colour(COL["arm"])
    return p


def link2_cover(sw):
    p = sw.new_part("h3-link2-cover")
    xf = link_xf(H.Y_E)
    bx0, bx1 = H.L2B
    with p.sketch("YZ", xf=xf) as s:
        s.segments(link_outline_open(H.L2, H.CAP_R, H.LINK_W))
        s.circle(0, 0, H.PIN_D / 2 + H.RUN)
    p.extrude(bx0, bx1, "Plate I")
    pts = _wall_stations(L2_STATIONS, 14.0) + HEAD_MOUNT
    with p.sketch("YZ", xf=xf) as s:
        for st, t in pts:
            s.circle(st, t, H.M3_CLEAR / 2)
    p.cut(bx0 - OVL, bx1 + OVL, "Bolt holes")
    with p.sketch("YZ", xf=xf) as s:
        for st, t in pts:
            s.circle(st, t, H.M3_CB_D / 2)
    p.cut(bx0 - OVL, bx0 + H.M3_HEAD_H, "Counterbores")
    p.colour(COL["arm"])
    return p


# ------------------------------------------------------------------- head ---
NOSE_TOP = 21.5
NOSE_BOT = -38.2
TAIL = (-52.0, -38.0)
NOSE_HALF_T = 13.3


def head(sw):
    p = sw.new_part("h3-head")
    xf = link_xf(H.Y_H)                     # (s_h, t), s_h = 0 on the tilt axis
    a0, a1 = H.NOSE_A
    tail_x = (H.NOSE_B[1], H.L2A[0] - 0.3)
    with p.sketch("YZ", xf=xf) as s:
        s.crect((TAIL[0] + TAIL[1]) / 2, 0, TAIL[1] - TAIL[0], 25.0, 3.0)
    p.extrude(*tail_x, name="Tail block")
    with p.sketch("YZ", xf=xf) as s:
        s.path([(NOSE_BOT, -NOSE_HALF_T), (NOSE_TOP, -NOSE_HALF_T),
                (NOSE_TOP, NOSE_HALF_T), (NOSE_BOT, NOSE_HALF_T)], [0, 6, 6, 0])
        s.circle(0, 0, 8.0)
    p.extrude(a0, a1, "Nose wall +X")
    with p.sketch("YZ", xf=xf) as s:
        s.path([(NOSE_BOT, -NOSE_HALF_T), (NOSE_TOP, -NOSE_HALF_T),
                (NOSE_TOP, NOSE_HALF_T), (NOSE_BOT, NOSE_HALF_T),
                (NOSE_BOT, 10.3), (17.5, 10.3), (17.5, -10.3), (NOSE_BOT, -10.3)],
               [0, 6, 6, 0, 0, 1, 1, 0])
    p.extrude(H.NOSE_B[1], a0 + OVL, "Nose walls")
    lx0 = H.HD_CASE_TOP + H.SV_TAB_Z1
    ledges = [14.4, -35.1]
    with p.sketch("YZ", xf=xf) as s:
        for sc in ledges:
            s.rect(sc - 4.0, -10.3 - OVL, sc + 4.0, 10.3 + OVL)
    p.extrude(lx0, a0 + OVL, "Head servo ledges")
    # cuts
    with p.sketch("YZ", xf=xf) as s:
        for sc in ledges:
            for t in (-5.0, 5.0):
                s.circle(sc, t, H.M3_CLEAR / 2)
    p.cut(lx0 - OVL, lx0 + 9.0, "Tab bolt holes")
    with p.sketch("YZ", xf=xf) as s:
        for sc, d in zip(ledges, (-1, 1)):
            for t in (-5.0, 5.0):
                nut_slot(s, (sc, t), H.NUT_POCKET_AF, (d, 0), 4.5)
    p.cut(lx0 + 3.5, lx0 + 3.5 + H.NUT_POCKET_H, "Tab nut slots")
    tail_bolts = [(-46.0, -7.0), (-46.0, 7.0)]
    with p.sketch("YZ", xf=xf) as s:
        for sc, t in tail_bolts:
            s.circle(sc, t, H.M3_CLEAR / 2)
    p.cut(tail_x[0] - OVL, tail_x[1] + OVL, "Mount bolt holes")
    with p.sketch("YZ", xf=xf) as s:
        s.circle(19.5, 0.0, H.M3_CLEAR / 2)
    p.cut(tail_x[0] - OVL, -8.0, "Cover bolt hole")
    with p.sketch("YZ", xf=xf) as s:
        for sc, t in tail_bolts:
            nut_slot(s, (sc, t), H.NUT_POCKET_AF, (0, math.copysign(1, t)), 6.0)
    p.cut(18.9, 21.6, "D-side nut slots")
    with p.sketch("YZ", xf=xf) as s:
        for sc, t in tail_bolts:
            nut_slot(s, (sc, t), H.NUT_POCKET_AF, (0, math.copysign(1, t)), 6.0)
    p.cut(-17.5, -14.8, "I-side nut slots")
    with p.sketch("YZ", xf=xf) as s:
        nut_slot(s, (19.5, 0.0), H.NUT_POCKET_AF, (1, 0), 2.5)
    p.cut(-16.4, -13.7, "Cover nut slot")
    with p.sketch("YZ", xf=xf) as s:     # head servo lead down into link2
        s.crect(-45.0, 0, 14.6, 8.0)
    p.cut(H.NOSE_B[1] - OVL, -8.0, "Lead channel")
    p.colour(COL["arm"])
    return p


def head_cover(sw):
    p = sw.new_part("h3-head-cover")
    xf = link_xf(H.Y_H)
    b0, b1 = H.NOSE_B
    with p.sketch("YZ", xf=xf) as s:
        s.path([(TAIL[0], -12.5), (NOSE_TOP, -12.5), (NOSE_TOP, 12.5), (TAIL[0], 12.5)],
               [3, 6, 6, 3])
    p.extrude(b0, b1, "Cover plate")
    with p.sketch("YZ", xf=xf) as s:
        s.crect((TAIL[0] + TAIL[1]) / 2 - 0.25, 0, TAIL[1] - TAIL[0] - 0.5, 25.0, 3.0)
    p.extrude(H.L2B[1] + 0.3, b0 + OVL, "Tail pad")
    with p.sketch("YZ", xf=xf) as s:
        s.circle(0, 0, H.PIN_D / 2 + 0.25)
    p.cut(b0 - OVL, b0 + 1.0, "Pin seat")
    pts = [(0.0, 0.0), (-46.0, -7.0), (-46.0, 7.0), (19.5, 0.0)]
    with p.sketch("YZ", xf=xf) as s:
        for sc, t in pts:
            s.circle(sc, t, H.M3_CLEAR / 2)
    p.cut(H.L2B[1], b1 + OVL, "Bolt holes")
    with p.sketch("YZ", xf=xf) as s:
        s.hexagon(0, 0, H.NUT_POCKET_AF)
    p.cut(b1 - H.NUT_POCKET_H, b1 + OVL, "Pin nut")
    with p.sketch("YZ", xf=xf) as s:
        s.circle(19.5, 0.0, H.M3_CB_D / 2)
    p.cut(b0 - OVL, b0 + H.M3_HEAD_H, "Counterbore")
    p.colour(COL["arm"])
    return p


# ------------------------------------------------------------------ shade ---
def shade(sw):
    """Local frame: tilt axis = X axis through the origin, cone axis +Y."""
    p = sw.new_part("h3-shade")
    B, Ls = H.SHADE_BACK, H.SHADE_LEN
    Rb, Rm = H.SHADE_BACK_D / 2, H.SHADE_MOUTH_D / 2
    k = (Rm - Rb) / Ls
    wt = H.SHADE_WALL * math.hypot(1, k)
    prof = [(3.5, B), (Rb, B), (Rm, B + Ls), (Rm - wt, B + Ls),
            (Rb + 2 * k - wt, B + 2.0), (3.5, B + 2.0)]
    with p.sketch("XY") as s:
        s.cline(0, 0, 0, 100)
        s.poly(prof)
    p.revolve(360, name="Cone")
    for x0, x1, nm in ((H.YOKE_A[0], H.YOKE_A[1], "Drive arm"),
                       (H.YOKE_B[0], H.YOKE_B[1], "Idler arm")):
        with p.sketch("wYZ") as s:
            s.slot(0, 0, B + 16.0, 0, 26.0)
        p.extrude(x0, x1, nm)
    # re-hollow the cone: the arms' inner faces reach inside it
    with p.sketch("XY") as s:
        s.cline(0, 0, 0, 100)
        # the cut's cone must be COLLINEAR with the shade's inner surface: a
        # line to (Rm - wt, B + Ls + 1) tilts 0.1 mm inward and left a sliver
        # of each arm standing on the inner wall
        s.poly([(0, B + 2.0), (Rb + 2 * k - wt, B + 2.0), (Rm - wt + k, B + Ls + 1.0),
                (0, B + Ls + 1.0)])
    p.revolve(360, cut=True, name="Hollow")
    horn_face = H.HD_CASE_TOP + H.HORN_TOP
    with p.sketch("wYZ") as s:
        a = (H.HORN_TRIM_L + 1.0 - H.HORN_POCKET_W) / 2
        s.slot(-a, 0, a, 0, H.HORN_POCKET_W)
    p.cut(H.YOKE_A[0] - OVL, horn_face, "Horn pocket")
    # the horn's O14 hub stands 1.0 proud of the drive arm's inner face
    # (interference check, every pose: 19.7 mm3) -- give it a relief
    with p.sketch("wYZ") as s:
        s.circle(0, 0, H.HORN_HUB_D / 2 + 0.6)
    p.cut(H.YOKE_A[0] - OVL, H.HD_CASE_TOP + H.HORN_TOP - H.HORN_T + 0.4, "Hub relief")
    with p.sketch("wYZ") as s:
        s.circle(0, 0, H.M3_CLEAR / 2)
    p.cut(H.YOKE_A[0] - OVL, H.YOKE_A[1] + OVL, "Hub bolt")
    with p.sketch("wYZ") as s:
        s.circle(0, 0, H.M3_CB_D / 2)
    p.cut(H.YOKE_A[1] - 2.5, H.YOKE_A[1] + OVL, "Hub counterbore")
    with p.sketch("wYZ") as s:
        s.circle(0, 0, H.PIN_D / 2 + H.RUN)
    p.cut(H.YOKE_B[0] - OVL, H.YOKE_B[1] + OVL, "Idler bore")
    p.colour(COL["arm"])
    return p


# ------------------------------------------------------------------- pins ---
def pin(sw, shaft, name):
    """Journal pin, local frame: axis +X, shaft x 0..shaft, flange beyond."""
    p = sw.new_part(name)
    r, rf = H.PIN_D / 2, H.PIN_FLANGE_D / 2
    t = H.PIN_FLANGE_T
    prof = [(0, 1.7), (0, r), (shaft, r), (shaft, rf), (shaft + t, rf),
            (shaft + t, H.M3_CB_D / 2), (shaft + 0.5, H.M3_CB_D / 2), (shaft + 0.5, 1.7)]
    with p.sketch("XY") as s:
        s.cline(-5, 0, shaft + 10, 0)
        s.poly(prof)
    p.revolve(360, name="Pin")
    p.colour(COL["dark"])
    return p


def ledclamp(sw):
    """Clamps the WS2812 ring against the shade's back plate. A collar
    printed on the shade itself would hang off the back plate into air with
    the shade printed mouth-down, so this is a flat ring instead: pushed
    down the cone, its tapered rim wedges against the cone wall just above
    the LEDs and its inner edge bears on the PCB rim. Shade local frame."""
    p = sw.new_part("h3-ledclamp")
    B, Ls = H.SHADE_BACK, H.SHADE_LEN
    Rb, Rm = H.SHADE_BACK_D / 2, H.SHADE_MOUTH_D / 2
    k = (Rm - Rb) / Ls
    wt = H.SHADE_WALL * math.hypot(1, k)
    r_in = lambda y: Rb + k * (y - B) - wt          # the cone's inner surface
    y0 = B + 2.0 + 1.6 + 0.2                        # just clear of the LEDs' base
    y1 = y0 + 2.0
    # modelled line-to-line with the cone: it is PUSHED until it wedges, so
    # its seated position is where the two surfaces meet
    prof = [(22.0, y0), (r_in(y0), y0), (r_in(y1), y1), (22.0, y1)]
    with p.sketch("XY") as s:
        s.cline(0, 0, 0, 100)
        s.poly(prof)
    p.revolve(360, name="Taper clamp ring")
    p.colour(COL["dark"])
    return p


def keycap(sw):
    p = sw.new_part("h3-keycap")
    y0 = H.MX_TOP + 6.2 + 0.8
    with p.sketch("wXZ") as s:
        s.crect(0, H.MX_POD_Z, 17.0, 17.0, 3.0)
    p.extrude(y0, y0 + 6.0, "Cap")
    with p.sketch("wXZ") as s:
        s.crect(0, H.MX_POD_Z, 7.4, 7.4, 0.4)
    p.cut(y0 - OVL, y0 + 3.0, "Stem socket")
    p.colour(COL["accent"])
    return p


# ============================================================= hand fixes ===
# Cuts made BY HAND on parts that were already printed (2026-10-12: the user
# chose a saw over a reprint). h3_build exports a part's print STL before
# its hand fix runs -- the STL stays exactly what was printed -- and saves
# the SolidWorks part after it, so the assembly checks the lamp as built.

KEEPER_Y0 = H.Y_P - H.SV_X0 + 0.4                                    # 112.9, window top
KEEPER_Z = (H.TOWER_BACK_Z + 4.0, H.SH_SHAFT_Z + H.SV_W / 2 + 0.4)   # -50.0 .. -28.35


def tower_handfix(p):
    """The shoulder servo cannot get into the printed tower: its case top
    has to pass 13.8 mm through the bulkhead window, the -X cheek leaves
    1.45 mm behind the case, and the bar over the window stops it dropping
    in from above. Two saw cuts free that bar -- one flush down the back
    wall, one straight up from the window's front edge -- and the servo
    drops in. The bar keeps the two upper tab-bolt holes and nut pockets it
    was printed with, so it goes back in as a keeper (tower_keeper)."""
    bx0, bx1 = H.SH_BULK_X
    with p.sketch("wYZ") as s:
        s.rect(KEEPER_Y0 - OVL, KEEPER_Z[0], H.Y_P + 24.0, KEEPER_Z[1])
    p.cut(bx0 - OVL, bx1 + OVL, "HAND FIX - bar over the servo window sawn out")


def tower_keeper(sw):
    """The bar tower_handfix saws out, modelled line-to-line with its slot
    (the saw kerf only makes it looser). Same features it had in the tower."""
    p = sw.new_part("h3-tower-keeper")
    bx0, bx1 = H.SH_BULK_X
    YP, zs = H.Y_P, H.SH_SHAFT_Z
    with p.sketch("wYZ") as s:
        s.rect(KEEPER_Y0, KEEPER_Z[0], YP + 20.0, KEEPER_Z[1])
    p.extrude(bx0, bx1, "Keeper")
    with p.sketch("wYZ") as s:
        s.poly([(YP + 6.0, -21.8), (YP + 20.2, -21.8), (YP + 20.2, -30.0)])
    p.cut(bx0 - OVL, bx1 + OVL, "Bulkhead chamfer")
    tabs = [(YP - hx, zs - hy) for hx, hy in H.SV_HOLES if YP - hx > KEEPER_Y0]
    with p.sketch("wYZ") as s:
        for y, z in tabs:
            s.circle(y, z, H.M3_CLEAR / 2)
    p.cut(bx0 - OVL, bx1 + OVL, "Tab bolt holes")
    with p.sketch("wYZ") as s:
        for y, z in tabs:
            s.hexagon(y, z, H.NUT_POCKET_AF)
    p.cut(bx1 - H.NUT_POCKET_H, bx1 + OVL, "Tab nut pockets")
    p.colour(COL["shell"])
    return p


def tub_handfix(p):
    """The slew-ring cone (the support-free fix) overhangs two things that
    go in from above. The speaker stands right under the ledge, so it has
    to come in tilted from the middle: its rails are snipped to 10 mm. The
    cone also roofs the mic slot 1.7 mm above the board, so the slot's -Z
    end wall is nibbled out and the board slides in sideways (the +Z side
    would send it into the amp). The third cut, a groove for the pan
    servo's cable boot, is older trouble: the old 8 mm audit steps missed it."""
    z0, z1 = H.SPK_Z0, H.SPK_Z0 + H.SPK_L
    with p.sketch("wXZ") as s:
        s.rect(H.SPK_X - 3.0, z1 - 1.0, H.SPK_X + H.SPK_T + 3.0, z1 + 2.0)
        s.rect(H.SPK_X - 3.0, z0 - 2.0, H.SPK_X + H.SPK_T + 3.0, z0 + 1.0)
    p.cut(H.TUB_FLOOR + 10.0, H.TUB_FLOOR + 24.0 + OVL, "HAND FIX - speaker rails snipped to 10 mm")
    with p.sketch("wXZ") as s:
        s.rect(H.MIC_X - 1.1, -10.0 - OVL, H.MIC_X + 1.1, -7.8 + OVL)
    p.cut(H.MIC_Y - 8.0, H.MIC_Y + 9.0 + OVL, "HAND FIX - mic slot end wall nibbled out")
    # The pan servo's case end sits 0.4 mm off the +X pillar, and its cable
    # boot sticks out of that end near the bottom, below the pillar's lead
    # notch -- lowering the servo drags the boot down the pillar face. A
    # groove from the notch to the top lets it through. 2.5 deep x 6.4 wide
    # leaves 1.2 mm round each M3 pilot hole (x 35.1, z +-5).
    tab_under = H.PAN_CASE_TOP + H.SV_TAB_Z0
    x0 = 31.1                                           # pillar inner face
    with p.sketch("wXZ") as s:
        s.rect(x0 - 0.5, -3.2, x0 + 2.5, 3.2)
    p.cut(13.0 - OVL, tab_under + OVL, "HAND FIX - groove for the pan servo's cable boot")


# =============================================================== hardware ===
def bolt(sw, length):
    """ISO 4762 M3 socket head. Local frame: axis +Z, the head's underside
    on z = 0, the shank (modelled at the 3.0 major diameter) to z = length."""
    p = sw.new_part(f"M3x{length:g}")
    with p.sketch("XY") as s:
        s.circle(0, 0, H.M3_HEAD_D / 2)
    p.extrude(-H.M3_HEAD_H, 0, "Head")
    with p.sketch("XY") as s:
        s.circle(0, 0, 1.5)
    p.extrude(0, length, "Shank")
    with p.sketch("XY") as s:
        s.hexagon(0, 0, 2.5)
    p.cut(-H.M3_HEAD_H - OVL, -H.M3_HEAD_H + 1.6, "Hex socket")
    p.colour(COL["metal"])
    return p


def nut(sw):
    """ISO 4032 M3. Local frame: axis +Z, centred on z = 0, corners at
    30 + 60k deg -- the same as Sketch.hexagon's pockets at rot 0."""
    p = sw.new_part("M3-nut")
    with p.sketch("XY") as s:
        s.hexagon(0, 0, H.M3_NUT_AF)
        s.circle(0, 0, 1.5)
    p.extrude(-H.M3_NUT_H / 2, H.M3_NUT_H / 2, "Nut")
    p.colour(COL["metal"])
    return p


HANDFIX = {"h3-tower": tower_handfix, "h3-tub": tub_handfix}
HANDCUT = [("h3-tower-keeper", tower_keeper)]
HARDWARE = [(f"M3x{n}", (lambda sw, n=n: bolt(sw, n))) for n in (8, 10, 12, 16)] + [("M3-nut", nut)]


PRINTED = [("h3-tub", tub), ("h3-lipring", lipring), ("h3-disc", disc),
           ("h3-tower", tower), ("h3-pinion", pinion), ("h3-link1", link1),
           ("h3-link1-cover", link1_cover), ("h3-link2", link2),
           ("h3-link2-cover", link2_cover), ("h3-head", head),
           ("h3-head-cover", head_cover), ("h3-shade", shade),
           ("h3-ledclamp", ledclamp), ("h3-keycap", keycap)]

PINS = {"h3-pin-shoulder": H.SHOULDER_PIN, "h3-pin-elbow": H.ELBOW_PIN,
        "h3-pin-head": H.HEAD_PIN}

BOUGHT = [("MG996R", servo), ("Horn", horn), ("HornTrim", None), ("PCA9685", pca9685),
          ("ESP32-S3", esp32), ("MAX98357A", amp), ("Speaker2040", speaker),
          ("INMP441", mic), ("Cap2200uF", bulkcap), ("CherryMX", mx_switch),
          ("WS2812-ring", led_ring)]
