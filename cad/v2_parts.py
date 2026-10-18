"""v2_parts.py — HOTARU 2.0. v1 is deprecated.

What changed and why, from the printed v1 and the reference mechanism the
user supplied (a Chinese "robot arm 3.18.1" 3MF, measured, plus photos of a
flat-link desk arm):

  pan        v1 had no pan at all; the arm sat on a fixed lid. 2.0 puts a
             MG996R dead centre in the tub, spline up, driving a turntable
             disc. 180 degrees in the plane, 1:1, no gears. The reference
             gears a micro servo into a ring gear because a 9 g servo is
             weak; an MG996R direct-drives the same load and a 180 servo
             through a reduction cannot reach 180 at the table, so its
             gearing is exactly what we do not copy.
  bearing    the disc rides the tub RIM (r72..75), not the servo shaft. The
             one good idea in the reference at our scale: a big slew surface
             takes the arm's tipping moment; the spline only takes torque.
  links      flat sandwich plates with round joint caps, like the reference
             photos, instead of v1's printed tubes. Each link is two plates;
             the next joint's servo body IS the structure between them.
  bed        printed on a P1S: 256 x 256, so the tub, disc and links plate
             together instead of nine A1-mini plates.

Drive interfaces are v1's, unchanged, because they are printed and proven:
cross-recess horn pocket (HORN_*), threaded stub axle + yoke-screw idler
(AXLE_*, SCREW_*), servo tab clamp bars (cup-cap concept). The v1 cone shade
mounts on the head end UNMODIFIED -- the head block copies v1's housing
interface dimensions (H_HX half-width, SG90 spline on axis, AXLE_D stub).
"""
from __future__ import annotations

import math
import os
import sys

from shapely import affinity
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import components as CO
import params as P
import partlib as pl

_GRILLE_SLOTS = 0     # set when the tub builds its vent banks
OVL = pl.OVL
BED = 256.0                     # P1S

# ---------------------------------------------------------------- layout ----
TUB_OD = 150.0
TUB_WALL = 3.0
TUB_H = 46.0                    # rim top = slew surface
FLOOR = 2.4

DISC_T = 4.5
DISC_Z0 = TUB_H                 # underside rides the rim
SKIRT_ID = TUB_OD + 0.6         # 0.3 per side round the rim
SKIRT_T = 2.4
SKIRT_DROP = 6.5
# The face reaches PAST the skirt: the first cut had the skirt at r75.3..77.7
# under a face that ended at r74 -- a ring attached to nothing. Each shell
# was watertight on its own, which is exactly why per-part validate() said
# nothing; the over-air audit is what caught it.
DISC_OD = SKIRT_ID + 2 * SKIRT_T

# pan servo: standing on the floor, spline tip at 2.4 + 47.6 = 50.0, which is
# 0.5 below the disc top face (50.5) -- the M3 goes down the centre into it
PAN_TAB_Z = FLOOR + P.MG_TAB_Z

# shoulder tower
TWR_W = 51.3                    # |-25.85| + 25.45, the derived cheek span
TWR_AXIS_Z = 45.0               # above disc top
TWR_BASE = (74.0, 44.0, 4.0)
TWR_H = TWR_AXIS_Z + 14.0

# links: axis to axis
L1 = 100.0                      # shoulder -> elbow
L2 = 90.0                       # elbow -> head
PLATE_T = 4.0
# 58, not 46: the stock horn is 54 tip to tip and a 46 cap cannot hold its
# pocket -- the slot broke out of both edges and left 1.1 mm of plate
# bridging a joint that carries the arm. 58 puts a 2 mm rim round the horn.
CAP_D = 58.0                    # round joint ends
LINK_W = 34.0                   # plate width between caps
GAP_FIT = 0.4                   # per side, yoke over tower / link over link

# ---- the joint stack, derived once and audited, never transcribed ----
# An MG996R crosses a joint along its HEIGHT: 42.9 of case plus 4.7 of boss
# and spline, 47.6 spline tip to tail. The first cut sized the sandwich to
# the servo's WIDTH (19.7) and only placing the real servo meshes exposed
# it. Every face below is a number with a reason:
#
#   case centred: top +21.45, tail -21.45           (42.9 case)
#   drive cheek  [21.45, 25.45]   4 thick, web 3.0 at its inner face and a
#                                 O20 x 1.0 counterbore at its outer, so the
#                                 boss pokes 1.0 into the counterbore and the
#                                 horn socket takes 1.7 of spline -- v1's
#                                 printed numbers exactly
#   idler cheek  [-25.85,-21.85]  4 thick, threaded stub outward, len 5
#   link1 inner faces: drive +25.75 (cheek outer +0.3), idler -26.15
#   shoulder horn: back 24.45, face 27.25 -> 1.5 into the 25.75 recess
#   elbow mirrors it inside link1 (drive -X): case top on link1-out's inner
#   face -26.15, horn face -31.95, link2-in inner -30.45 -> 1.5 engagement
#   elbow case tail +16.75: fully INSIDE the sandwich. No window, no proud
#   tail, no swept annulus -- the truncated idler plate is no longer needed
#   for clearance but stays for weight.
CASE_TOP = 21.45
DRIVE_CHEEK = (21.45, 25.45)
IDLER_CHEEK = (-25.85, -21.85)
L1_IN_HALF = 25.75
L1_OUT_HALF = 26.15
L2_IN_HALF = 30.45
L2_OUT_HALF = 30.05
STUB_LEN = 5.0
BOSS_WALL = 3.0
LINK1_HALF = L1_IN_HALF          # kept as an alias for the audit
LINK2_HALF = L2_IN_HALF
LINK2_OUT_HALF = L2_OUT_HALF
LINK2_OUT_START = 32.0
HUB_BORE = P.HORN_HUB_D + P.HORN_FIT
ELBOW_TAIL = -26.15 + P.MG_H     # +16.75, must stay inside L1_IN_HALF

# head: the tilt joint is an MG996R too, so the robot is ONE servo part
# number in four places -- one spare covers every joint, one horn fits
# everywhere, and the head no longer runs a 29.5 case where a 42.9 one has
# to reach. The nose half-width is the servo's own reach: case bottom to
# spline tip is MG_SHAFT_TOP, measured from the case CENTRE that is
# 26.15 -- the identical number link1's outer plate already sits at.
HEAD_SHAFT_Y = 22.0                      # nose-local station of the tilt axis
HEAD_SHAFT_Z = 15.0
# The nose's drive face is the DRIVE CHEEK plane, not the spline tip. Putting
# it at the tip looks right and is wrong: the horn's socket is bored from its
# cross face, so a horn seated flush on the tip engages ZERO spline. The
# shoulder gets this right by standing its cheek 0.7 short of the tip and
# counterboring 1.0 into it, which buys 1.7 of spline in the socket and 1.5
# of cross in the plate's recess. The head now uses the identical stack --
# same faces, same numbers, same horn.
HEAD_HALF = DRIVE_CHEEK[1]                             # 25.45, the drive face
HEAD_SPLINE_TIP = P.MG_SHAFT_TOP - P.MG_H / 2          # 26.15, 0.7 proud
HEAD_CBORE_D = 20.0
HEAD_CBORE_T = 1.0
HEAD_TAIL_W = L2_IN_HALF + L2_OUT_HALF - 2 * GAP_FIT   # 59.7
# The tail is as wide as the sandwich, so anywhere it reaches, the shade's
# yoke cannot. It stops below the yoke's lowest sweep instead of running the
# full nose depth.
HEAD_TAIL_TOP = 8.0
# 24.5, not 26: the enlarged shade's apex-side ring (O44.6) closes over
# the head at world 297.5, and a 26 nose topped out at 298.5 -- the cone's
# open end bit 1.0 into the nose's upper corners.
HEAD_NOSE_LEN = 24.5
HEAD_BLOCK_H = 30.0

# Standoff and ledge positions, shared by the plates (through-holes), the
# standoff parts (inserts) and the audit's hole-exists probe.
def l1_spots():
    # x=32: the shoulder servo's upper tab sits at +16.45 with its clamp
    # bar above it; a post at 20 sat straight on the tab
    return [(32.0, -LINK_W / 2 + 6), (32.0, LINK_W / 2 - 6)]

def l1_ledge_spots():
    # ledge centres sit just BEYOND the tab tips, so the 9-wide posts clear
    # the 40.7 case with margin while the tabs still land on them
    bc = L1 - (P.MG_L / 2.0 - P.MG_SHAFT_OFF)        # elbow body centre
    return [(bc - P.MG_TAB_SPAN / 2 - 1.8, 0.0),
            (bc + P.MG_TAB_SPAN / 2 + 1.8, 0.0)]

def l2_spots():
    return [(LINK2_OUT_START + 8, -LINK_W / 2 + 6),
            (LINK2_OUT_START + 8, LINK_W / 2 - 6), (47.0, 0.0)]

# ---- all-printed fastening: there is NO metal on this robot ----
# Every M3-insert bore becomes a printed P6 thread (the v1 yoke screw's
# proven M6x2 form), every M3 becomes a printed thumb-bolt, and the two
# board-screw stations become locating pins under gravity. The user has no
# screws and wants to buy none; v1's printed thread is the one fastener
# this project has already manufactured and load-tested.
PB_CLEAR = P.SCREW_MAJOR + 0.6      # thumb-bolt clearance hole
PB_BORE = P.SCREW_MAJOR + 0.8       # host bore the threaded sleeve fuses in
PB_SLEEVE = P.SCREW_MAJOR + 2.4     # sleeve outer: 0.8 fuse ring per side
PB_HEAD = 11.0

def _psleeve(x, y, z0, z1):
    # seg 20 / per_turn 8, not the 64/24 default: a O6 printed thread is
    # resolved by a 0.4 nozzle far below either number, and at default
    # tessellation thirteen sleeves plus the bolt strip pushed the
    # assembled GLB to 113 MB -- past GitHub's 100 MB hard limit.
    b = pl.threaded_bore(PB_SLEEVE, P.SCREW_MAJOR, P.SCREW_PITCH,
                         0.0, z1 - z0, clearance=P.SCREW_FIT / 2,
                         seg=20, per_turn=8)
    b.translate(dx=x, dy=y, dz=z0)
    return b

# Board stations: one definition, used by the tub's posts AND by the
# assembly that drops the real components onto them.
# The ESP32's USB connectors live on the +X SHORT edge of the board, and
# the old charge window sat in the FRONT wall -- pointed at the board's
# side, where no plug could ever reach a port. Rotating the board to face
# the window does not fit either: wall to pan-servo collar is 57 and the
# board is 64. So the board stays long-axis-X, slides +16 toward the right
# wall, and the charge opening moves to the +X wall as a recessed well in
# line with the connectors. Reach from wall surface to port face is 12.7.
# y=33, not 40: the +X+Y corner of the 64 x 30 board reaches r 67.9 there,
# 1.3 inside the pebble wall at PCB height -- at 40 it was r 72.9, in the
# wall. The -Y edge at 18 still clears the pan clamp bosses, which top out
# at y 17.15.
# x 13, not 16: the PCB corner at (48, 50.5) crossed the tub's inner wall
# arc, which reaches in to r~69 there. The board is squeezed between the
# pan clamp bosses at y 20.2 and that arc, and 3 mm inboard is where a
# 64 x 30 board actually clears both.
ESP_XY = (13.0, 35.5)
ESP_ROT = 0.0
ESP_POST = 10.0          # > 8.5 of ESP32-S3 pin
AMP_XY = (-40.0, -34.0)
AMP_POST = 7.5           # > 6.0 of amp pin
# y=-6: the charge well's walls own y 16..50 on this side; at y=0 the
# speaker's +Y edge (20) sat 4 inside them. x=54: the pebble foot pulls the
# wall in to r 65.7 at z=4, and at x=58 the shifted frame's -Y corner
# reached r 68.6 -- inside the foot. 54 keeps the whole footprint at
# r <= 64.9.
SPK_XY = (54.0, -6.0)
SPK_Z = 14.0             # 20 mm tall on edge; keeps it off the 2.4 floor
# x=-63.5: the wall's bulge leans IN over the old -66 pocket (inner face
# r72 at the pocket mouth vs a board edge at r73.5), so the mic could never
# be lowered into it. 2.5 inboard clears the lean the whole way down.
MIC_XY = (-63.5, 0.0)
MIC_Z = 26.0
FACET_FLAT = 71.0        # y of the front/back flats

# The MX button rides the TURRET: a solid pod grown out of the rear wall and
# crown, with the switch dropped into its TOP face, stem up. The button is
# part of the FIXED base -- the disc turns, the button does not -- which is
# the only arrangement where "press the top of the base" works at every pan
# angle. The old front-wall mount is gone.
MXC = (0.0, -87.25)      # switch centre, in the turret top
TR_R0, TR_R1 = 77.5, 97.0
TR_A0, TR_A1 = 256.0, 284.0
TR_TOP = 56.0            # same height as the crown cap
MX_PLATE_Z = TR_TOP - P.MX_PLATE_T     # 54.5, the 1.5 clip plate
SEG = 96

COLORS = {
    "v2-tub": "#E9E9EE", "v2-disc": "#E9E9EE", "v2-tower": "#E9E9EE",
    "v2-link1-in": "#E9E9EE", "v2-link1-out": "#E9E9EE",
    "v2-link2-in": "#E9E9EE", "v2-link2-out": "#E9E9EE",
    "v2-cap": "#26262B", "v2-clamp": "#26262B", "v2-accent": "#4D17F5",
}


def _stadium(length, w, cap_d):
    """Sculpted link outline, per the reference: full cap discs at the
    joints, a TAPERED body between them, everything filleted. A parallel
    stadium is what read as stale."""
    a = pl.circle(cap_d, SEG)
    b = affinity.translate(pl.circle(cap_d, SEG), length, 0)
    body = Polygon([(4, -w / 2 - 2), (length - 4, -w / 2 + 4),
                    (length - 4, w / 2 - 4), (4, w / 2 + 2)])
    return pl.smooth(unary_union([a, body, b]), 6.0)


def _skeleton(length, keepout):
    """The truss cutouts and glow slots that make the reference read as a
    machine instead of a plank. Cut candidates are clipped to the plate's
    safe interior and then MINUS the functional keepout, so styling can
    never eat a bore, a recess, a standoff seat or the servo window."""
    tris = []
    x0, x1 = CAP_D / 2 + 8.0, length - CAP_D / 2 - 8.0
    if x1 - x0 > 24:
        xm = (x0 + x1) / 2
        tris.append(Polygon([(x0, -LINK_W / 2 + 7), (xm - 3, -2),
                             (x0, LINK_W / 2 - 7)]))
        tris.append(Polygon([(x1, -LINK_W / 2 + 7), (xm + 3, -2),
                             (x1, LINK_W / 2 - 7)]))
    tris = [pl.smooth(t, 3.0) for t in tris if t.area > 40]
    slots = [affinity.translate(pl.slot(13.0, 4.2), length - 37.0, dy)
             for dy in (-9.0, 0.0, 9.0)]
    cuts = unary_union(tris + slots)
    return cuts.difference(keepout.buffer(3.0))


def _horn_recess_openings(plate, x_axis, z0, t):
    """Pocket for the STOCK MG996R double horn, cut from the driven face.

    The printed horn's recess was a CROSS, because a printed horn could be
    whatever shape suited us. The moulded one is a single straight arm, so
    the pocket is a single slot -- and a cross pocket would leave the real
    horn free to rattle a quarter turn round into the empty arm.

    Torque goes through the slot's two long SIDES, which is why the ends
    can be generous. The centre bore is cut clear through so the hub nests
    and a driver reaches the shaft screw behind it."""
    w = P.SHORN_W + 2 * P.SHORN_FIT
    L = P.SHORN_L + 2 * P.SHORN_FIT
    hub = P.SHORN_HUB_D + 2 * P.SHORN_FIT
    rec_d = P.SHORN_T + P.SHORN_FIT
    cx, cy = x_axis
    slot = affinity.translate(pl.rounded_rect(L, w, w / 2.0), cx, cy)
    hubc = affinity.translate(pl.circle(hub, 48), cx, cy)
    # Two screws per arm, through the horn's OWN holes into the driven
    # plate. That is what those holes are for, and it is what now retains
    # the joint axially: the printed trim cups that used to do it were
    # shaped round a O13 printed hub and cannot cover a 54 mm moulded bar.
    # The screws are the small self-tappers in the servo bag.
    scr = unary_union([affinity.translate(pl.circle(P.SHORN_SCREW, 16),
                                          cx + sx * r, cy)
                       for sx in (-1, 1) for r in P.SHORN_SCREW_R])
    return [
        (slot, z0 - OVL, z0 + rec_d),
        (hubc, z0 - OVL, z0 + t + OVL),
        (scr, z0 - OVL, z0 + t + OVL),
    ]


# ------------------------------------------------------------------ tub ----
def tub():
    R = TUB_OD / 2.0
    m = pl.Mesh()
    # Pebble profile: the wall blends out of the desk through a 12 mm fillet
    # and bulges 3 past the rim diameter through the middle -- the reference
    # base's soft soap-bar read -- then lands back exactly on O150 at the
    # rim, because the rim is the slew surface and the slew is not styling.
    import math as _math
    # Base blend is a straight 40-degree chamfer, not a sine: the sine's
    # tangent at z=0 sat at 46 degrees and the audit flagged the first two
    # millimetres of wall as over air. A 16-over-19 line never exceeds 40.
    # The bulge window still ends at 36 -- the disc's skirt hangs to 39.5 at
    # r75.3, and the first curve peaked at r76.1 inside it.
    def od(z):
        if z < 19.0:
            return TUB_OD - 16.0 + 16.0 * (z / 19.0)
        if z < 36.0:
            t = (z - 19.0) / 17.0
            return TUB_OD + 6.0 * _math.sin(t * _math.pi)
        return TUB_OD
    # The wall is a stack of banded rings, not a revolve_shell: this kernel
    # has no 3D CSG, so a lofted shell cannot take a window afterwards. The
    # rings can. Two FLAT FACETS are shaved into the profile (front -Y and
    # back +Y) so the MX switch and the USB port get clean rectangular
    # openings in flat wall instead of ragged holes in a curve -- the
    # reference base does exactly this for its side port.
    # A facet is a CHORD, not a guillotine. The first version differenced
    # everything beyond y = -66 out of the finished ring -- but the wall
    # lives at r 74.5..77.5, so the whole front wall is beyond 66 and the
    # cut deleted it, leaving an open mouth with the MX frame floating in
    # front of nothing. Each shell stayed watertight, so nothing complained.
    # Chord the OUTER at F and the INNER at F - wall, and the flat keeps a
    # full-thickness wall behind it.
    FACET_Y = 71.0
    from shapely.geometry import box as _box
    def _flat(prof, f):
        return prof.difference(_box(-95, -95, 95, -f)).difference(
                               _box(-95, f, 95, 95))

    # wall openings, all expressed in XY + a z band:
    # MX: the switch clips into a plate EXACTLY MX_PLATE_T thick, so the
    # front flat is thinned to 1.5 over the switch footprint and the 14.1
    # cutout goes through that. Behind it the wall opens to MX_BODY_SQ so
    # the latches have somewhere to spring.
    # USB charge well in the +X wall, in line with the board's connector
    # edge; spans BOTH ports (26 across) plus clearance
    usb_win = _box(48.9, ESP_XY[1] - 15.0, 95.0, ESP_XY[1] + 15.0)
    # MX lead: from under the turret pocket, through rear wall and bulge
    wire_win = _box(-6.0, -95, 6.0, -66.0)
    mic_hole = affinity.translate(pl.circle(P.MIC_PORT_D, 24), -74.0, 0.0)
    mic_hole = mic_hole.union(_box(-92, -P.MIC_PORT_D / 2, -70, P.MIC_PORT_D / 2))
    # Ventilation, sized by v2_margins rather than by eye. The tub is a
    # CLOSED box holding the ESP32 and the pan servo, and 7 slots over 36
    # degrees gave 277 mm2 -- about a sixth of what free convection needs to
    # carry ~1.6 W at a 20 K rise. Two banks now:
    #   +X  exhaust side, beside the speaker, already reads as a grille
    #   -X  intake, stopping short of the crown's 195-degree foot
    # Positive angles 20..35 are skipped: at r70 those land in the USB
    # window's own y band and would leave the wall there more air than wall.
    # The disc's kidney slot is the high outlet, so this convects.
    _gang = [a for a in range(-45, 20, 5)] + [40, 45]
    _gang += [a for a in range(140, 191, 5) if a != 180]   # 180 is the mic
    grille = unary_union([
        affinity.rotate(_box(60.0, -1.5, 92.0, 1.5), a, origin=(0, 0))
        for a in _gang])
    globals()["_GRILLE_SLOTS"] = len(_gang)

    USB_Z0 = FLOOR + ESP_POST + 1.4 - 2.4          # below the port slab
    WALL_OPEN = [
        (usb_win, USB_Z0, USB_Z0 + P.USB_PLUG_H + 1.6),
        (mic_hole, 26.0, 26.0 + P.MIC_PORT_D),
        (grille,   5.0, 33.0),
        (wire_win, 24.0, 36.0),
    ]
    steps = 40
    for k in range(steps):
        za = TUB_H * k / steps
        zb0 = TUB_H * (k + 1) / steps            # the band's TRUE top
        zb = zb0 + (OVL if k < steps - 1 else 0.0)   # stretched, to fuse
        o = od(za)
        ring = _flat(pl.circle(o, 160), FACET_Y).difference(
               _flat(pl.circle(o - 2 * TUB_WALL, 160), FACET_Y - TUB_WALL))
        # against zb0, NOT the stretched zb: a band is stretched by OVL so
        # its shell fuses with its neighbour, and testing the opening
        # against that inflated top drops any opening whose ceiling lands
        # on a band edge -- the whole band then prints solid. The opening
        # covers the band it is asked about, not the overlap.
        cuts = unary_union([g for g, zl, zh in WALL_OPEN
                            if zl <= za + 0.01 and zh >= zb0 - 0.01] or
                           [Polygon()])
        ring = ring.difference(cuts)
        if not ring.is_empty:
            m += pl.prism(ring, za, zb)
    m += pl.prism(_flat(pl.circle(od(0.4), 160), FACET_Y), 0.0, FLOOR)  # floor

    # ---- the rear crown: the bulged / non-bulged split the pan RANGE asks
    # for. The MG996R sweeps 180 degrees, all of it over the +Y front, so
    # the front rim stays LOW (the arm and shade fold down past it) and the
    # rear rises into a shroud around the disc, the reference base's tall
    # back. Sector 195..345 degrees, wall 78.2..81.2 -- 0.5 outside the
    # skirt's 77.7 so the disc still drops in from above -- grounded to the
    # floor through its own 40-degree foot, capped flat at 56.
    from shapely.geometry import Polygon as _Poly
    def _sector(a0, a1):
        # steps scale with the arc: the crown now sweeps 332 degrees, and a
        # fixed 40 would have chorded it into a visible polygon once the
        # ring was intersected against 160-segment circles.
        n = max(40, int(abs(a1 - a0) / 1.5))
        pts = [(0.0, 0.0)] + [
            (200.0 * math.cos(math.radians(a0 + t * (a1 - a0) / n)),
             200.0 * math.sin(math.radians(a0 + t * (a1 - a0) / n)))
            for t in range(n + 1)]
        return _Poly(pts)
    # The crown parts at the turret: its ring through 256..284 would run
    # straight across the pod's switch relief (795 probe points inside
    # crown and MX body at once). The pod carries that sector's wall.
    CR_IR, CR_OR = 78.2, 81.2
    # FULL RING, not a 150-degree arc. An arc has two free ends, and a wall
    # 3 mm thick standing 56 tall on two free ends is a lever waiting to
    # snap -- which is exactly what the user said when they held the
    # printed one. Closing it into a ring removes the free ends entirely:
    # every section is braced by the two beside it, and hoop stress does
    # the work instead of a cantilever. Only the turret's own sector is
    # left out, because the pod fills it.
    # The crown runs INTO the turret pod by 4 degrees at each end instead
    # of stopping exactly where the pod starts. Butted end-to-end the two
    # only shared a plane, which is precisely the seam an arc levers open
    # at; overlapped, they interpenetrate by ~5.6 mm of arc at r80 and the
    # ring is continuous material the whole way round, crown into pod into
    # crown. A closed ring has no free end to lever.
    sec = _sector(TR_A1 - 4.0, TR_A0 + 360.0 + 4.0)
    # Disc retention, fully printed: at 210 and 330 degrees the crown grows
    # an outboard channel boss, and a WINDOW through its inner wall at
    # 38.0..39.4 -- just under the skirt's bottom edge (39.5). A printed key
    # drops down the channel after the disc is in; its toe pokes through the
    # window beneath the skirt. Lift the disc and the skirt meets two toes
    # whose upward load path is the window ceiling, solid crown. The keys
    # go in AFTER the disc, which is the whole reason the crown itself
    # cannot carry these toes (a fixed overhang would block the disc drop
    # -- the closure audit is what forced this into a separate part).
    KEY_ANGS = (210.0, 330.0)
    KEY_W = 10.0
    def _key_boxes(ang, r0, r1, w):
        bb = box(r0, -w / 2, r1, w / 2)
        return affinity.rotate(bb, ang, origin=(0, 0))
    # The boss is a rotated BOX, and a box aimed along a radius carries its
    # corners PAST the nominal outer radius (hypot(84.4, 8.2) = 84.8). The
    # key's flange starts exactly at 84.4, so those corners buried
    # themselves 0.4 into it. Trimming the boss to the true r=84.4 arc
    # makes the face the flange lands on actually flat-at-radius.
    key_boss = unary_union([_key_boxes(a, CR_OR - OVL, CR_OR + 3.2, KEY_W + 6.4)
                            for a in KEY_ANGS]).intersection(
                                pl.circle(2 * (CR_OR + 3.2), 160))
    key_win = unary_union([_key_boxes(a, 75.2, CR_OR + 3.2 + OVL, KEY_W + 0.7)
                           for a in KEY_ANGS])
    # window floor at 37.0, not 38.0: prism bands stretch their ceilings by
    # OVL (0.2), so a band ending AT the tongue's z buried its bottom 0.05.
    # With the floor a full millimetre down, the stretch lands in void; the
    # ceiling stays sharp at 39.4 because stretches ADD material and the
    # solid band above already owns everything past 39.4.
    crown_marks = sorted({0.0, 2.0, 4.0, 6.0, 8.0, 30.0, 37.0, 39.4, 56.0,
                          5.0, 24.0, 26.0, 33.0, 36.0, 26.0 + P.MIC_PORT_D,
                          USB_Z0, USB_Z0 + P.USB_PLUG_H + 1.6})
    for za, zb0 in zip(crown_marks[:-1], crown_marks[1:]):
        zb = zb0 + (OVL if zb0 < 56.0 else 0.0)
        ro = CR_OR if za >= 8.0 else CR_OR - (8.0 - za) * 1.1
        # Below the skirt the crown's INNER radius reaches the wall, which
        # is the only thing joining it to the tub. At a constant 78.2 it
        # stood 3.2 clear of a wall ending at 75: a free-floating ring,
        # watertight and fully audited, that came off the plate as a loose
        # arc in the user's hand. Above 37 it steps back out so the disc's
        # skirt still drops past it.
        ri = CR_IR if za >= 37.0 else TUB_OD / 2 - TUB_WALL
        ring = pl.ring2d(pl.circle(2 * ro, 160), pl.circle(2 * ri, 160))
        ring = ring.intersection(sec)
        # Grounding the crown to the wall filled r72..81.2 all the way
        # round below 37 -- and every opening in that wall (speaker grille,
        # USB well, mic port, the MX lead window) sits inside that annulus.
        # They were all bricked up behind the crown: the wall had a hole,
        # the crown behind it did not, so from outside there was no hole at
        # all. Whatever the wall opens, the crown opens too.
        thru = [g for g, zl, zh in WALL_OPEN
                if zl <= za + 1e-6 and zh >= zb0 - 1e-6]
        if thru:
            ring = ring.difference(unary_union(thru))
        if za >= 30.0:                        # boss stiffens the wall
            ring = ring.union(key_boss.intersection(sec))
            if za >= 37.0 - 1e-6 and zb0 <= 39.4 + 1e-6:   # the key window
                ring = ring.difference(key_win)
        if not ring.is_empty:
            m += pl.prism(ring, za, zb)

    # ---- the button turret: the reference base's rear pod, load-bearing ----
    # A solid sector pod fused across crown and wall bulge, flat on top at
    # the crown's own 56. The MX switch drops into that top face exactly the
    # way it used to clip into the front flat -- same 1.5 plate, same 14.1
    # cutout, same 16 latch relief -- but rotated flat, so the key faces UP
    # from the fixed part of the base. Its lead runs down a bore inside the
    # pod and through the rear wall at 26..34, under the skirt's lowest
    # point (39.5), so the path never crosses anything that rotates.
    tsec = _sector(TR_A0, TR_A1)
    mx_sq = _box(MXC[0] - P.MX_CUT / 2, MXC[1] - P.MX_CUT / 2,
                 MXC[0] + P.MX_CUT / 2, MXC[1] + P.MX_CUT / 2)
    mx_re = _box(MXC[0] - P.MX_BODY_SQ / 2, MXC[1] - P.MX_BODY_SQ / 2,
                 MXC[0] + P.MX_BODY_SQ / 2, MXC[1] + P.MX_BODY_SQ / 2)
    vwire = affinity.translate(pl.circle(10.0, 24), *MXC)
    # ACCESS PORT through the pod's back, at the terminal height.
    #
    # Without it the switch's two solder tags sit at the bottom of a blind
    # 16 x 16 pocket, 10 mm deep, whose only other exit is the lead bore --
    # no soldering iron reaches that, and no finger does either. The switch
    # could be dropped in and then never wired. There is only 1.75 mm of
    # wall behind the pocket (it spans r 79.2..95.2 inside a pod that ends
    # at 97), so cutting it through costs nothing structurally and turns
    # the pocket into an open port: clip the switch in from the top, solder
    # both tags from behind, drop the leads down the bore. It faces the
    # back of the lamp, and it is the only way this switch is serviceable.
    # The access slot faces INWARD, into the tub, not out the visible face.
    # Cut outward it was the first thing you saw on the back of the lamp;
    # cut inward it opens above the rim where the disc later hides it, and
    # you still reach the tags with the disc off -- which is when you are
    # soldering anyway. The 1.75 mm of outer wall behind the pocket stays,
    # so from outside the pod is unbroken.
    mx_inner = affinity.translate(pl.rounded_rect(16.0, 22.0, 3.0),
                                  MXC[0], MXC[1] + 8.0)
    # ONE opening in the pod's back, not two. The access port and the lead
    # window were separate holes at 44..54.5 and 26..34, which read from
    # outside as two unexplained cavities stacked on the same face. They do
    # one job between them -- get to the tags, get the wire out -- so they
    # are one slot from 26 up to the plate: solder at the top of it, the
    # lead falls down the same opening and through the tub wall.
    T_OPEN = [
        (mx_sq, MX_PLATE_Z, TR_TOP + OVL),
        (mx_re, 44.0, MX_PLATE_Z),
        (mx_inner, 44.0, MX_PLATE_Z),
        (vwire, 28.0, 44.0 + OVL),
        (wire_win, 24.0, 36.0),
    ]
    # Bands are split at every opening edge. A fixed 2 mm grid missed the
    # 54.5 plate line: the 54..56 band carried no active opening, and the
    # mesh probe found the "cutout" solid.
    marks = sorted({0.0, 2.0, 4.0, 6.0, 8.0, 24.0, 28.0, 36.0, 37.0, 44.0,
                    MX_PLATE_Z, TR_TOP})
    for za, zb in zip(marks[:-1], marks[1:]):
        ro = TR_R1 if za >= 8.0 else TR_R1 - (8.0 - za) * 1.1
        ri_t = TR_R0 if za >= 37.0 else TUB_OD / 2 - TUB_WALL
        ring = pl.ring2d(pl.circle(2 * ro, 160), pl.circle(2 * ri_t, 160))
        ring = ring.intersection(tsec)
        cuts = unary_union([g for g, zl, zh in T_OPEN
                            if zl <= za + 0.01 and zh >= zb - 0.01] or
                           [Polygon()])
        ring = ring.difference(cuts)
        if not ring.is_empty:
            m += pl.prism(ring, za, zb + (OVL if zb < TR_TOP else 0.0))

    # USB well: a solid land from just past the board edge into the +X
    # wall, with the plug cavity cut through it in line with the ports. The
    # wall opening (usb_win) opens the curved shell; this well walls the
    # passage so the plug cannot wander into the tub.
    USB_Z0 = FLOOR + ESP_POST + 1.4 - 2.4
    tun = _box(49.6, ESP_XY[1] - 17.0, 64.0, ESP_XY[1] + 17.0)
    cav = _box(48.9, ESP_XY[1] - 14.5, 80.0, ESP_XY[1] + 14.5)
    m += pl.banded(tun, FLOOR - OVL, USB_Z0 + P.USB_PLUG_H + 1.6 + 2.4,
                   [(cav, USB_Z0, USB_Z0 + P.USB_PLUG_H + 1.6)])
    # mic cradle behind its port
    # The INMP441 breakout is a 15 x 12.6 CARD, so the cradle is rectangular.
    # A round pocket sized on MIC_D held the board's midline and let all four
    # corners bury themselves in the boss wall.
    mic_l, mic_w = P.MIC_D + 2 * P.MIC_FIT, 12.6 + 2 * P.MIC_FIT
    mcx = MIC_XY[0]
    mic_pock = _box(mcx - mic_l / 2, -mic_w / 2, mcx + mic_l / 2, mic_w / 2)
    mic_boss = _box(mcx - mic_l / 2 - 2.4, -mic_w / 2 - 2.4,
                    mcx + mic_l / 2 + 2.4, mic_w / 2 + 2.4)
    # the pocket floor IS the seat: the breakout's bottom face lands on it at
    # MIC_Z, and its pins -- 6 long -- drop through a relief below that, which
    # a 22.0 floor did not have (they buried 2 mm into solid boss).
    mic_pin = affinity.translate(pl.circle(7.0, 24), mcx, 0.0)
    m += pl.banded(mic_boss.difference(_box(-95, -20, -73, 20)), FLOOR - OVL, 38.0,
                   [(mic_pock, MIC_Z, 38.0 + OVL),
                    (mic_pin, MIC_Z - 8.0, MIC_Z + OVL)])

    # Disc retention is the pan horn's centre M3, full stop. A clip system
    # was tried and produced three unsolvable conflicts (a radial screw
    # would have to cross the ROTATING skirt). The load says it was never
    # needed: 3 N at 200 mm of reach is 0.6 Nm, which is ~8 N of uplift at
    # the centre against a steel M3 in the spline. Documented, not hidden.

    # ---- pan servo mount: pocket walls + two tab pillars ----
    # Body centre is +10.35 from the spline, not -10.35: the component mesh
    # was probed (body spans spline -10 .. +30.7) after the world sweep put
    # 2409 points inside pillar and body at once. Probe the mesh, never
    # transcribe the datasheet's sign convention.
    bx = P.MG_L / 2.0 - P.MG_SHAFT_OFF
    body = box(bx - P.MG_L / 2 - 0.4, -P.MG_W / 2 - 0.4,
               bx + P.MG_L / 2 + 0.4, P.MG_W / 2 + 0.4)
    collar = body.buffer(2.4, join_style=2).difference(body)
    m += pl.prism(collar, FLOOR - OVL, 16.0)                    # lateral collar
    for sx in (-1, 1):
        px = bx + sx * (P.MG_TAB_SPAN / 2 - 3.2)
        pil = box(px - 5.0, -P.MG_W / 2 - 0.4, px + 5.0, P.MG_W / 2 + 0.4)
        pil = pil.difference(body)
        # the lead leaves the case at the tail face; the tail-side pillar
        # gets a notch (a 16-wide bridge, anchored on its y-legs)
        wire_w = box(px - 5.0 - OVL, -8.0, px + 5.0 + OVL, 8.0)
        m += pl.banded(pil, FLOOR - OVL, PAN_TAB_Z,
                       [(wire_w, 12.0, 22.0)] if sx < 0 else [])
        # M3 insert bosses beside the tab, for the clamp bar
        for sy in (-1, 1):
            # +5.35 not +3.6: the boss grew from O7.4 to O10 to swallow the
            # printed-thread sleeve, and at the old offset its flank clipped
            # the servo tab corners by 1.4 (the audit counted 47 points)
            bpos = (px, sy * (P.MG_W / 2 + 5.35))
            bos = affinity.translate(pl.circle(PB_SLEEVE + 1.6, 32), *bpos)
            bor = affinity.translate(pl.circle(PB_BORE, 24), *bpos)
            m += pl.prism(bos.difference(bor), FLOOR - OVL, PAN_TAB_Z + 6.0)
            m += _psleeve(*bpos, PAN_TAB_Z - 4.0, PAN_TAB_Z + 6.0)

    # ---- electronics, on the floor, v1 footprints ----
    def corner_tabs(cx, cy, L, W, h_post, rise=3.0, fit=0.4, sides="xy"):
        """Four L-tabs just OUTSIDE the board outline, rising past its top
        face. They locate the PCB by its EDGES.

        Pins through the mounting holes were the first idea and they were
        wrong twice: the ESP's -X pair landed under the WROOM can and the
        amp's +Y pair landed under the solder-pad strip. Neither board has
        a hole where the standoff geometry wanted one. An edge fence needs
        no hole at all, so it cannot be wrong about where the holes are.

        `sides` exists because a dev board is wider than its PCB: the
        ESP32-S3's WROOM can hangs 5 mm past the -X edge and its USB shell
        1.2 mm past the +X edge, so X-face tabs sit exactly where the
        overhang is. That board gets Y-face tabs only, and the charge well
        and wall hold it in X."""
        m2 = pl.Mesh()
        hx, hy = L / 2 + fit, W / 2 + fit
        for sx in (-1, 1):
            for sy in (-1, 1):
                arms = []
                if "x" in sides:
                    arms.append(box(cx + sx * hx, cy + sy * (hy - 5.0),
                                    cx + sx * (hx + 1.8), cy + sy * hy))
                if "y" in sides:
                    arms.append(box(cx + sx * (hx - 6.0), cy + sy * hy,
                                    cx + sx * hx, cy + sy * (hy + 1.8)))
                m2 += pl.prism(unary_union(arms),
                               FLOOR - OVL, FLOOR + h_post + rise)
        return m2

    def posts(cx, cy, offs, h_post, pin_offs=None):
        """Standoffs tall enough that the board's THROUGH-HOLE PINS clear the
        floor. An ESP32-S3's pins hang 8.5 below its PCB and the amp's 6.0;
        5 mm posts would have driven both straight into the tub floor.

        The offsets are given per board rather than as a symmetric inset,
        because a rectangle inset from the PCB edge lands ON the header
        strips: the ESP32's two rows own y +/-12.1..14.7 for the board's
        whole length, and the amp's single row owns y -8.7..-6.1. A 5.4 post
        under either one holds the board up by its connectors."""
        pp = pl.Mesh()
        pin_offs = offs if pin_offs is None else pin_offs
        for ox, oy in offs:
            p = (cx + ox, cy + oy)
            b = affinity.translate(pl.circle(5.4, 24), *p)
            pp += pl.prism(b, FLOOR - OVL, FLOOR + h_post)
            if (ox, oy) not in pin_offs:
                continue
            # locating PIN up through the board's corner hole -- the screw
            # is gone. The tub is sealed by the disc, so gravity plus pins
            # is the whole mounting story, and it costs nothing. Pins only
            # where the REAL board has a hole: the ESP's +X posts sit at
            # +26.5 (the charge well pushed them inboard), where the board
            # is solid copper -- a pin there punches the PCB, and the audit
            # counted it. Two pins fix position; four posts carry weight.
            # Pin top stops 0.05 BELOW the board's top face: anything proud
            # found the parts that overhang the corner holes.
            pin = affinity.translate(pl.circle(1.8, 16), *p)
            pp += pl.prism(pin, FLOOR + h_post - OVL, FLOOR + h_post + 1.55)
        return pp
    # ESP32-S3: inboard of the header rows (|y| <= 12.1 - 2.7 post radius)
    # +X pair sits at +26.5, not +29.5: the charge well's land begins at
    # 49.6 and a post at 45.5 reached into it
    m += posts(*ESP_XY, [(ox, sy * 8.5) for ox in (-29.5, 26.5)
                         for sy in (-1, 1)], ESP_POST, pin_offs=[])
    m += corner_tabs(*ESP_XY, P.ESP_L, P.ESP_W, ESP_POST, sides="y")
    # MAX98357A: its one row is along -Y, so both pairs sit above it.
    # NO pins: the +Y pair lands at local (+/-7.5, 6.0), which on the real
    # breakout is under the solder-pad strip, not a mounting hole -- a pin
    # there presses on pads. The board is 3 g on a shelf inside a closed
    # tub; four posts and gravity are the whole mounting.
    m += posts(*AMP_XY, [(sx * 7.5, oy) for sx in (-1, 1)
                         for oy in (6.0, -2.5)], AMP_POST, pin_offs=[])
    m += corner_tabs(*AMP_XY, P.AMP_L, P.AMP_W, AMP_POST)

    # speaker against +X wall: two rails + open back grille (v1 lesson: the
    # pocket stays open behind the driver)
    # Speaker seat, INBOARD at x=58: the pebble chamfer pulls the wall away
    # from under the old wall-hugging position, and the rails' outboard ends
    # stood on the void (58 mm2, found at r70 by the over-air audit). On the
    # flat floor there is nothing to hang over. The wall grille lands in the
    # electronics pass, aimed at this seat.
    sx0 = SPK_XY[0]
    for sy in (-1, 1):
        y_in = SPK_XY[1] + sy * (P.SPK_L / 2 + 0.4)
        rail = box(sx0 - 3.0, min(y_in, y_in + sy * 3.0),
                   sx0 + P.SPK_T + 1.2, max(y_in, y_in + sy * 3.0))
        m += pl.prism(rail, FLOOR - OVL, FLOOR + 26.0)

    # mic port + USB window + MX plate would go through the wall here; kept
    # for the v2 electronics pass so this file stays reviewable
    return [("v2-tub", m, COLORS["v2-tub"])]


# ----------------------------------------------------------------- disc ----
def disc():
    m = pl.Mesh()
    face = pl.circle(DISC_OD, 160)

    # kidney slot for the arm harness: +Y side, 70 degrees of arc
    ring = pl.ring2d(pl.circle(64.0, 96), pl.circle(48.0, 96))
    wedge = Polygon([(0, 0)] + [(90 * math.cos(math.radians(a)),
                                 90 * math.sin(math.radians(a)))
                                for a in [55 + k * (70 / 24) for k in range(25)]])
    kidney = ring.intersection(wedge)

    # centre: bolt counterbore + clearance, and the horn cross recess below.
    #
    # This was the LAST M3 hole in the robot -- O3.4 through a O6.8
    # counterbore, both sized for a metal M3 -- and it survived the
    # all-printed conversion because the sweep that replaced the others
    # matched on a wrapper this line does not have. The fastener that goes
    # here is now a printed P6 thumb-bolt: O6 thread, O6.6 clearance, O11
    # head. It could not go in, which is exactly what the user found.
    # M3 into the servo shaft's own tapped hole -- the screw that comes in
    # the servo bag with the horn. Counterbored so its head sits below the
    # disc face and the tower can seat flat on top of it.
    cb = pl.circle(P.M3_HEAD_D + 1.0, 40)
    thr = pl.circle(P.M3_CLEAR, 24)

    openings = _horn_recess_openings(face, (0.0, 0.0), DISC_Z0, DISC_T)
    # ANNULAR relief for the pan servo's secondary case hump. The hump is
    # fixed to the case while the disc turns over it, so a local pocket
    # would only clear at one angle -- the sweep audit found the disc
    # grounding on it at every step except 0 and +/-90. A groove at the
    # hump's radius clears it through the whole rotation.
    # r 10..28, not 12..26: the pan servo's boss cleared the rotating disc
    # by 0.15 mm, which is not a running fit on an FDM part -- it is a rub.
    # The relief is a groove the disc sweeps over, so widening it costs
    # nothing but a little plastic.
    HUMP_R0, HUMP_R1, HUMP_TOP = 10.0, 28.0, 48.4
    openings.append((pl.ring2d(pl.circle(2 * HUMP_R1, 96),
                               pl.circle(2 * HUMP_R0, 96)),
                     DISC_Z0 - OVL, HUMP_TOP))
    # The pan servo's boss pokes up into the disc's hub bore, and the disc
    # TURNS over it: O13.3 round a O13 boss is 0.15 a side, which is a rub
    # on an FDM part, not a running fit. The bore is stepped out to O14 for
    # the 3.1 the boss actually occupies -- 0.5 a side -- and stays 13.3
    # above that, where the horn's hub needs it. The cross arms run out to
    # r24 so they lose nothing that grips.
    # to 49.8, past the boss's own top at 49.3 -- a counterbore that stops
    # BELOW the thing it is clearing clears nothing, which a 3.0 ceiling
    # proved by leaving the gap at 0.15 exactly where it started.
    openings.append((pl.circle(14.0, 48), DISC_Z0 - OVL, DISC_Z0 + 3.8))
    openings += [
        (kidney, DISC_Z0 - OVL, DISC_Z0 + DISC_T + OVL),
        (thr, DISC_Z0 - OVL, DISC_Z0 + DISC_T + OVL),
        (cb, DISC_Z0 + DISC_T - 2.9, DISC_Z0 + DISC_T + OVL),
    ]
    # 4 insert bores for the tower flange
    for (ix, iy) in _tower_bolts():
        openings.append((affinity.translate(pl.circle(DISC_BOLT_TAP, 24), ix, iy),
                         DISC_Z0 - OVL, DISC_Z0 + DISC_T + OVL))
    # No arc vents. They were styling on the ONE face that has to stay a
    # clean bearing surface and a clean top, and they read as busy rather
    # than machined. The platter is plain; the crown and the link skeletons
    # carry the visual language instead.
    m += pl.banded(face, DISC_Z0, DISC_Z0 + DISC_T, openings)
    # no printed-thread sleeves here any more: an M3 self-taps straight into
    # the 2.9 pilot through the disc's own 4.5 of plate

    # rim skirt: hides the joint and locates the disc round the rim
    sk = pl.ring2d(pl.circle(SKIRT_ID + 2 * SKIRT_T, 160), pl.circle(SKIRT_ID, 160))
    m += pl.prism(sk, DISC_Z0 - SKIRT_DROP, DISC_Z0 + OVL)
    return [("v2-disc", m, COLORS["v2-disc"])]


# The turntable's fasteners are M3, not printed. A printed P6 thumb-bolt
# is fine where it is turned by hand into printed plastic, but the disc is
# the one joint that takes the whole arm's tipping moment through four
# bolts, and the user's verdict on the printed ones there was simply that
# they are not good.
DISC_BOLT_CLEAR = P.M3_CLEAR          # 3.4 through the tower flange
DISC_BOLT_HEAD = P.M3_HEAD_D + 0.6    # counterbore in the flange
DISC_BOLT_TAP = 2.9                   # self-tapping pilot in the disc


def _tower_bolts():
    return [(sx * 28.0, sy * 16.0) for sx in (-1, 1) for sy in (-1, 1)]


# ---------------------------------------------------------------- tower ----
def tower():
    """Shoulder mount. Prints on its back. Bolts to the disc, 4x M3.
    Cheek faces come from the derived joint stack at the top of this file;
    the servo hangs between them on two tab posts, clamped by printed bars."""
    z0 = DISC_Z0 + DISC_T
    m = pl.Mesh()
    bw, bd, bt = TWR_BASE
    base = pl.rounded_rect(bw, bd, 6.0)
    holes = unary_union([affinity.translate(pl.circle(DISC_BOLT_CLEAR, 24), x, y)
                         for x, y in _tower_bolts()])
    heads = unary_union([affinity.translate(pl.circle(DISC_BOLT_HEAD, 24), x, y)
                         for x, y in _tower_bolts()])
    m += pl.banded(base, z0, z0 + bt, [
        (holes, z0 - OVL, z0 + bt + OVL),
        (heads, z0 + bt - 1.6, z0 + bt + OVL)])

    ax_z = z0 + TWR_AXIS_Z
    top = ax_z + 22.0

    # drive cheek +X: web 3.0 at the inner face, counterbore at the outer
    x0, x1 = DRIVE_CHEEK
    prof = box(x0, -bd / 2 + 2, x1, bd / 2 - 2)
    bore = [(box(x0 - OVL, -w, x1 + OVL, w), zl, zh)
            for zl, zh, w in _slices(P.MG_BOSS_D + 1.0, ax_z)]
    cb = [(box(x0 + BOSS_WALL, -w, x1 + OVL, w), zl, zh)
          for zl, zh, w in _slices(P.MG_BOSS_D + 7.0, ax_z)]
    # the case's four screw bosses stand ~0.5 proud of its top face; relieve
    # the cheek 1.0 where they land (probed at 16..23 below the axis)
    relief = [(box(x0 - OVL, -10.4, x0 + 1.0, 10.4), ax_z - 31.2, ax_z - 11.0),
              # the case's secondary hump (probed O7 x 2.0 at axis-20) gets a
              # clear window; the cheek is wide, the hole is not structural
              (box(x0 - OVL, -4.9, x1 + OVL, 4.9), ax_z - 24.9, ax_z - 15.1)]
    m += pl.banded(prof, z0 + bt - OVL, top, bore + cb + relief)

    # idler cheek -X: with the threaded stub outward on the axis, and a
    # notch where the servo's lead leaves the case tail -- the world sweep
    # put 1660 probe points inside cheek and wire at once, which is the
    # audit's way of saying the wire had nowhere to exist.
    x0, x1 = IDLER_CHEEK
    body_c0 = (DISC_Z0 + DISC_T + TWR_AXIS_Z) - (P.MG_L / 2.0 - P.MG_SHAFT_OFF)
    wire = box(x0 - OVL, -8.0, x1 + OVL, 8.0)
    m += pl.banded(box(x0, -bd / 2 + 2, x1, bd / 2 - 2), z0 + bt - OVL, top,
                   [(wire, body_c0 - 26.0, body_c0 - 12.0)])
    ax = pl.threaded_bore(P.AXLE_D, P.SCREW_MAJOR, P.SCREW_PITCH,
                          0.0, P.SCREW_ENGAGE, clearance=P.SCREW_FIT / 2)
    ax.rotate_y(-90.0)
    ax.translate(dx=x0 + OVL, dz=ax_z)
    m += ax

    # tab posts: the servo's tabs sit at case-bottom + 26.6, which is
    # x = +5.15, at z beyond both ends of the 40.7 case. Post face at the tab
    # underside; the clamp bar and an M3 through the drive cheek close it.
    body_c = ax_z - (P.MG_L / 2.0 - P.MG_SHAFT_OFF)
    for tz in (body_c - P.MG_TAB_SPAN / 2 - 1.8, body_c + P.MG_TAB_SPAN / 2 + 1.8):
        pm = pl.prism(affinity.translate(pl.rounded_rect(
            (CASE_TOP - P.MG_TAB_Z) - IDLER_CHEEK[1], 12.0, 2.0),
            ((CASE_TOP - P.MG_TAB_Z) + IDLER_CHEEK[1]) / 2.0, 0.0), tz - 4.5, tz + 4.5)
        m += pm
    return [("v2-tower", m, COLORS["v2-tower"])]


def _slices(d, cz, seg=24):
    """NB: callers get d + 0.8 of slack folded in -- ten slabs approximate
    the bore as a polygon whose steps bite inward, and the shoulder boss
    interfered with exactly those steps in the world sweep."""
    """Horizontal slabs approximating a Y-axis... X-axis bore of diameter d
    centred at height cz -- v1's _disc_slices, reimplemented for banded()."""
    out = []
    r = (d + 0.8) / 2.0
    n = 10
    for k in range(n):
        z1 = cz - r + (2 * r) * k / n
        z2 = cz - r + (2 * r) * (k + 1) / n
        zm = (z1 + z2) / 2.0
        half = math.sqrt(max(r * r - (zm - cz) ** 2, 0.0))
        out.append((z1, z2, half))
    return out


# ---------------------------------------------------------------- links ----
def _link_plate(name, length, colour, near, far, spots=(), holes_at=()):
    """One flat plate, lying in XY at z 0..PLATE_T. near/far feature menu:
    recess | idler | boss3 | window | steadybore | mount | none.
    `spots` get M3 through-holes (standoff screws); `holes_at` likewise (the
    servo-tab clamp screws). Both also become styling keepouts."""
    prof = _stadium(length, LINK_W, CAP_D)
    openings = []
    keep = [affinity.translate(pl.circle(36.0, 48), x0, 0) for x0 in (0.0, length)]
    for x0, feat in ((0.0, near), (length, far)):
        at = lambda g: affinity.translate(g, x0, 0)
        if feat == "recess":
            openings += _horn_recess_openings(prof, (x0, 0.0), 0.0, PLATE_T)
        elif feat == "idler":
            openings.append((at(pl.circle(P.AXLE_D + P.AXLE_FIT, 48)),
                             -OVL, PLATE_T + OVL))
        elif feat == "boss3":
            openings.append((at(pl.circle(P.MG_BOSS_D + 1.0, 48)),
                             -OVL, PLATE_T + OVL))
            openings.append((at(pl.circle(P.MG_BOSS_D + 7.0, 48)),
                             BOSS_WALL, PLATE_T + OVL))
            # relief for the case screw bosses, minus a contact ring at the
            # axis so the case still registers on the plate
            rel = box(x0 - 20.8, -10.4, x0 + 20.8, 10.4).difference(
                at(pl.circle(26.0, 48)))
            openings.append((rel, -OVL, 1.0))
            openings.append((affinity.translate(
                pl.circle(9.8, 32), x0 - 19.7, 0.0), -OVL, PLATE_T + OVL))
        elif feat == "window":
            bc = x0 - (P.MG_L / 2.0 - P.MG_SHAFT_OFF)
            w = box(bc - (P.MG_L + 0.8) / 2, -(P.MG_W + 0.8) / 2,
                    bc + (P.MG_L + 0.8) / 2, (P.MG_W + 0.8) / 2)
            openings.append((w, -OVL, PLATE_T + OVL))
            keep.append(w.buffer(2.0))
        elif feat == "mount":
            for dy in (-10.0, 10.0):
                openings.append((affinity.translate(pl.circle(PB_CLEAR, 24), x0, dy),
                                 -OVL, PLATE_T + OVL))
    if "stub" in (near, far):
        x0 = length if far == "stub" else 0.0
        keep.append(affinity.translate(pl.circle(30.0, 48), x0, 0))
    for (cx, cy) in tuple(spots) + tuple(holes_at):
        openings.append((affinity.translate(pl.circle(PB_CLEAR, 24), cx, cy),
                         -OVL, PLATE_T + OVL))
        keep.append(affinity.translate(pl.rounded_rect(13.0, 12.0, 2.0), cx, cy))
    cuts = _skeleton(length, unary_union(keep))
    if not cuts.is_empty:
        openings.append((cuts, -OVL, PLATE_T + OVL))
    m = pl.banded(prof, 0.0, PLATE_T, openings)
    if "stub" in (near, far):
        # threaded stub on the OUTER face, so the next link's idler plate
        # rides it and a yoke-screw traps it -- v1's proven pair
        x0 = length if far == "stub" else 0.0
        boss = pl.prism(pl.circle(P.AXLE_D, 48), PLATE_T - OVL, PLATE_T + 0.6)
        boss.translate(dx=x0)
        ax = pl.threaded_bore(P.AXLE_D, P.SCREW_MAJOR, P.SCREW_PITCH,
                              0.0, P.SCREW_ENGAGE, clearance=P.SCREW_FIT / 2)
        ax.translate(dx=x0, dz=PLATE_T + 0.6 - OVL)
        m += boss
        m += ax
    return (name, m, colour)


def _ledge_posts(name, spots, h, colour):
    """Tab ledges whose top 9 mm is two prongs: the servo lead exits at the
    case bottom right where a solid post face would pin it. The interference
    sweep found 1772 probe points inside ledge and wire at once."""
    m = pl.Mesh()
    for (cx, cy) in spots:
        b = affinity.translate(pl.rounded_rect(10.0, 12.0, 2.0), cx, cy)
        hh = affinity.translate(pl.circle(PB_BORE, 24), cx, cy)
        m += pl.prism(b.difference(hh), 0.0, h - 9.0)
        slot = affinity.translate(box(-5.5, -3.2, 5.5, 3.2), cx, cy)
        m += pl.prism(b.difference(hh).difference(slot), h - 9.0 - OVL, h)
        m += _psleeve(cx, cy, 0.0, h - 9.0)      # thread in the solid zone
    return (name, m, colour)


def _standoffs(name, spots, h, colour):
    """M3-insert standoff posts, flat on the bed."""
    m = pl.Mesh()
    for (cx, cy) in spots:
        b = affinity.translate(pl.rounded_rect(10.0, 9.0, 2.0), cx, cy)
        hh = affinity.translate(pl.circle(PB_BORE, 24), cx, cy)
        m += pl.prism(b.difference(hh), 0.0, h)
        m += _psleeve(cx, cy, 0.0, h)
    return (name, m, colour)


def link1():
    c = COLORS["v2-link1-in"]
    return [
        # drive side +X at the shoulder; elbow flips drive to -X. The elbow
        # servo lives ENTIRELY inside the sandwich (tail at +16.75 against an
        # inner face at +25.75), so there is no window and nothing sweeps.
        # far end grows the elbow's idler stub: with the elbow servo now
        # fully inside the sandwich there is nothing proud to dodge, so the
        # elbow is a real two-sided yoke instead of the single-sided
        # compromise -- which is why it read as a missing cap.
        _link_plate("v2-link1-in", L1, c, near="recess", far="stub",
                    spots=l1_spots(), holes_at=l1_ledge_spots()),
        _link_plate("v2-link1-out", L1, c, near="idler", far="boss3",
                    spots=l1_spots()),
        _standoffs("v2-link1-spacers", l1_spots(),
                   L1_IN_HALF + L1_OUT_HALF - 0.4, COLORS["v2-clamp"]),
        # ledge height: from link1-in's inner face down to the tab underside,
        # tail + MG_TAB_Z above the case bottom. Derived, because the first
        # value was transcribed from the wrong end of the servo.
        _ledge_posts("v2-link1-ledges", l1_ledge_spots(),
                     L1_IN_HALF - (ELBOW_TAIL - P.MG_TAB_Z) - P.MG_TAB_T,
                     COLORS["v2-clamp"]),
    ]


def _truncated_plate(name, length, start, colour, far, spots=()):
    """A plate that begins `start` from the near axis: no near cap at all.
    Same sculpt and skeleton treatment as the full plates."""
    prof = pl.smooth(unary_union([
        Polygon([(start, -LINK_W / 2 + 3), (length - 4, -LINK_W / 2),
                 (length - 4, LINK_W / 2), (start, LINK_W / 2 - 3)]),
        affinity.translate(pl.circle(CAP_D, SEG), length, 0)]), 6.0)
    openings = []
    keep = [affinity.translate(pl.circle(36.0, 48), length, 0),
            box(start - 2, -LINK_W / 2, start + 8, LINK_W / 2)]
    if far == "mount":
        for dy in (-10.0, 10.0):
            openings.append((affinity.translate(pl.circle(PB_CLEAR, 24), length, dy),
                             -OVL, PLATE_T + OVL))
    for (cx, cy) in spots:
        openings.append((affinity.translate(pl.circle(PB_CLEAR, 24), cx, cy),
                         -OVL, PLATE_T + OVL))
        keep.append(affinity.translate(pl.rounded_rect(13.0, 12.0, 2.0), cx, cy))
    cuts = _skeleton(length, unary_union(keep)).intersection(
        prof.buffer(-6.0))
    if not cuts.is_empty:
        openings.append((cuts, -OVL, PLATE_T + OVL))
    m = pl.banded(prof, 0.0, PLATE_T, openings)
    return (name, m, colour)


def link2():
    c = COLORS["v2-link2-in"]
    return [
        _link_plate("v2-link2-in", L2, c, near="recess", far="none",
                    spots=l2_spots(),
                    holes_at=[(L2 - 22.0, -10.0), (L2 - 22.0, 10.0)]),
        _link_plate("v2-link2-out", L2, c, near="idler", far="none",
                    spots=list(l2_spots())
                        + [(L2 - 22.0, -10.0), (L2 - 22.0, 10.0)]),
        _standoffs("v2-link2-spacers", l2_spots(),
                   L2_IN_HALF + L2_OUT_HALF - 0.4, COLORS["v2-clamp"]),
    ]


# ----------------------------------------------------------------- head ----
def head_pockets(fit=P.MG_FIT):
    """The MG996R's cavity in the head, taken from the SERVO MESH.

    Every other servo pocket in this file is a hand-derived box list, and
    each one cost a rebuild when a face was transcribed instead of derived
    -- the first head pocket had the SG90's length and width swapped and put
    1218 probe points inside servo and nose at once. So this reads the real
    component's own sub-part bounds and maps them into the head's frame.

    The servo lies height-along-X, the one orientation whose spline can span
    the nose wall. Servo-local (x, y, z) -> head-local:

        head x = z - MG_SHAFT_TOP + HEAD_HALF     (spline tip on the +X face)
        head y = (HEAD_SHAFT_Y + SPL_X) - x       (length along -Y)
        head z = y + HEAD_SHAFT_Z                 (width along Z)

    Returns (openings, floor_z). Every pocket is cut OPEN TO THE TOP: the
    servo is dropped straight down into the nose, and a boss channel bored
    blind through the +X wall would have to be threaded on sideways with the
    case already seated, which is not a motion a hand can make. The wall at
    the boss is 0.7 thick anyway -- it was never carrying load.
    """
    spl_x = -(P.MG_L / 2.0 - P.MG_SHAFT_OFF)      # -10.35, spline centre
    out, floor = [], None
    for n, mm, _c in CO.mg996r():
        if n.endswith("-label"):
            continue                              # printed sticker, not solid
        b = mm.bounds()
        hx0 = b[2] - P.MG_SHAFT_TOP + HEAD_SPLINE_TIP
        hx1 = b[5] - P.MG_SHAFT_TOP + HEAD_SPLINE_TIP
        hy0 = (HEAD_SHAFT_Y + spl_x) - b[3]
        hy1 = (HEAD_SHAFT_Y + spl_x) - b[0]
        hz0 = b[1] + HEAD_SHAFT_Z
        floor = hz0 - fit if floor is None else min(floor, hz0 - fit)
        out.append(box(hx0 - fit, hy0 - fit, hx1 + fit, hy1 + fit))
    return out, floor


def head_block():
    """Nose carries the shade on the tilt axis; tail bolts between link2's
    plates.

    Local frame: X width, Y depth (tail y -13..13, nose y 9..35), Z height
    0..30. In world it is rotated x+90, so local (x,y,z) lands at
    (x, 15-z, z_head-22+y). The tilt axis is local (., 22, 15).

    Tail screws: link2's plates bolt through at world (y=+/-10, z_head-22),
    which is local (y=0, z=5) and (z=25). Those are 2.8 x 3.2 tunnels cut
    clear through the tail width; M3s self-thread into the printed flats,
    the same way v1's M2 pilots worked."""
    m = pl.Mesh()
    # HEAD_TAIL_W already nets out the running fit; subtracting it again
    # here left the tail rattling 0.8 per side (the contact row caught it)
    tail_w = HEAD_TAIL_W
    D = HEAD_NOSE_LEN
    pockets, floor = head_pockets()
    # the servo's lower length end reaches local y < 13, i.e. INTO the tail
    # band, so the tail carries the same cuts as the nose. The lead's own
    # pocket is one of them, and it is run OUT through the +Y wall: a servo
    # whose cable has nowhere to go does not seat, however well the case fits.
    cuts = [(g, floor, HEAD_BLOCK_H + OVL) for g in pockets]
    cuts.append((box(-9.0, D - 4.0 + 1.0, -3.0, D + 10.0), 11.0,
                 HEAD_BLOCK_H + OVL))

    tail_d = HEAD_TAIL_TOP + D / 2.0
    tail = affinity.translate(pl.rounded_rect(tail_w, tail_d, 5.0),
                              0.0, HEAD_TAIL_TOP - tail_d / 2.0)
    tun = box(-tail_w / 2 - OVL, -1.4, tail_w / 2 + OVL, 1.4)
    m += pl.banded(tail, 0.0, HEAD_BLOCK_H,
                   [(tun, 3.4, 6.6), (tun, 23.4, 26.6)] + cuts)

    # nose: the drive face carries the horn's counterbore, exactly as the
    # shoulder cheek does, so the cross sits 1.0 in and the socket swallows
    # 1.7 of spline
    nose = affinity.translate(pl.rounded_rect(2 * HEAD_HALF, D + 5.0, 5.0),
                              0, D - 6.5)
    cb = affinity.translate(pl.circle(HEAD_CBORE_D, 48),
                            HEAD_HALF - HEAD_CBORE_T, HEAD_SHAFT_Y)
    cb = cb.intersection(box(HEAD_HALF - HEAD_CBORE_T, -99, 99, 99))
    m += pl.banded(nose, 0.0, HEAD_BLOCK_H, cuts + [
        (cb, HEAD_SHAFT_Z - HEAD_CBORE_D / 2, HEAD_SHAFT_Z + HEAD_CBORE_D / 2)])

    # idler stub: 4 long, a plain printed axle -- and NO screw. The shade
    # is one rigid piece: its drive plate is axially locked (horn in the
    # drop-in recess, capped by the glued cup), so the idler plate cannot
    # walk off this stub without the whole cone walking, which the drive
    # side forbids. The M3-and-washer this stub used to carry was v1
    # thinking; there is no outboard room for any head here anyway
    # (link2-in stands 0.5 away), which the interference audit proved.
    ax = pl.prism(pl.circle(P.AXLE_D, 48), 0.0, 4.0 + OVL)
    ax.rotate_y(-90.0)
    ax.translate(dx=-HEAD_HALF + OVL, dy=HEAD_SHAFT_Y, dz=HEAD_SHAFT_Z)
    m += ax
    return [("v2-head", m, COLORS["v2-tower"])]


def _slot_head(d):
    """A coin-slot disc: the visual signature of the reference's joints.
    The slot goes clear through -- it prints clean in any orientation and
    reads as machined from both sides."""
    return pl.circle(d, 96).difference(box(-d * 0.36, -1.7, d * 0.36, 1.7))


def v2_disckey_one():
    """ONE disc-retention tongue, y-centred on its own axis. The world
    assembly places two of these; the print strip is two side by side.
    (The first cut placed the whole 2-tongue strip at each window, so every
    key carried a phantom twin 16 mm away buried in the crown wall -- the
    interference audit found the twins.)"""
    # The TONGUE reaches 0.5 back into the flange, rather than the flange
    # reaching forward into the tongue. Both give one solid part -- sharing
    # only the plane x=0 gave two loose bits -- but the flange's front face
    # is what lands on the crown boss at r 84.4, so growing it forward put
    # 0.5 of key inside the boss. Growing the tongue backwards costs
    # nothing: behind the flange is air.
    m = pl.prism(box(-0.5, -9.3 / 2, 8.9, 9.3 / 2), 0.0, 1.2)
    m += pl.prism(box(-1.55, -12.9 / 2, 0.0, 12.9 / 2), 0.0, 3.2)
    return m


def v2_disckeys():
    """The two disc-retention tongues as one printed strip. Each slides
    RADIALLY through its crown window after the disc is seated; the tip
    rides 0.15 under the skirt's bottom edge, so the disc cannot lift, and
    the fixed crown never has to overhang the disc's drop-in path (a fixed
    overhang is exactly what the closure audit rejects). The flange stands
    on the boss face as the stop and the fingernail grip."""
    out = pl.Mesh()
    for i in range(2):
        q = v2_disckey_one().copy()
        q.translate(dy=i * 16.0)
        out += q
    return [("v2-disckeys", out, COLORS["v2-accent"])]


def v2_bolts():
    """22 printed P6 thumb-bolts -- every fastener on the robot, printed.

    4 tower->disc, 4 link1 spots (both plates), 2 link1 ledges, 6 link2
    spots (both plates), 6 servo clamp bars. Head 11 with the coin slot,
    minor-diameter neck through the 4.0 plate, then 6.5 of the proven M6x2
    printed thread into the sleeve on the far side."""
    m = pl.Mesh()
    for i in range(22):
        q = v2_bolt_one()
        q.translate(dx=(i % 6) * 14.0, dy=(i // 6) * 16.0)
        m += q
    return [("v2-bolts", m, COLORS["v2-accent"])]


# Pitch the tray grids use. The head is the widest feature at PB_HEAD, so
# 14 x 16 leaves 3 mm between heads in X and 5 in Y -- enough that a knife
# gets in beside a head without touching its neighbour.
BOLT_PITCH = (14.0, 16.0)


def v2_bolt_one():
    """ONE P6 thumb-bolt, at the origin, head down on the plate.

    Factored out of v2_bolts so the spares tray prints the SAME bolt rather
    than a second copy of the recipe that can drift from it."""
    m = pl.Mesh()
    neck_d = P.SCREW_MAJOR - 2 * pl.THREAD_RAMP * P.SCREW_PITCH
    m += pl.prism(_slot_head(PB_HEAD), 0.0, 2.6)
    m += pl.prism(pl.circle(neck_d, 32), 2.6 - OVL, 7.0)
    m += pl.thread(P.SCREW_MAJOR, P.SCREW_PITCH, 7.0 - OVL, 13.5,
                   seg=20, per_turn=8)
    return m


def v2_screws_strip():
    """The two yoke screws as one printed strip: shoulder and elbow. The
    head joint needs none -- see head_block's stub comment."""
    m = pl.Mesh()
    for i in range(2):
        q = v2_screw()[0][1].copy()
        q.translate(dx=i * 30.0)
        m += q
    return [("v2-screws", m, COLORS["v2-accent"])]


def v2_screw():
    """The shoulder idler's retaining screw, v1's proven M6x2 printed thread
    under a big slotted head. Turned with a coin or a flat blade."""
    m = pl.Mesh()
    m += pl.prism(_slot_head(26.0), 0.0, 3.2)
    m += pl.prism(pl.circle(P.SCREW_MAJOR - 2 * pl.THREAD_RAMP * P.SCREW_PITCH, 48),
                  3.2 - OVL, 4.2)
    m += pl.thread(P.SCREW_MAJOR, P.SCREW_PITCH, 4.2 - OVL, 4.2 + P.SCREW_ENGAGE)
    return [("v2-screw", m, COLORS["v2-accent"])]


def v2_trimcap():
    """The elbow's matching cap: same slotted face, a plug that glues into
    the steady bore after the horn's M3 is home. Cosmetic, and honest about
    it -- the slot turns nothing here.

    No plug at all: the horn's hub end stands 2.0 PROUD of the plate's
    outer face (probed, not assumed -- the first cap was modelled against
    an imagined recessed hub and drove 4 mm into the real one). The cap is
    a shallow CUP: its rim glues to the plate around the bore, and the
    proud hub end lives inside the pocket with 0.3 all round."""
    m = pl.Mesh()
    head = _slot_head(32.0)
    pocket = pl.circle(P.SHORN_HUB_D + 1.0, 48)
    m += pl.banded(head, 0.0, 6.6, [(pocket, 0.9, 6.6 + OVL)])
    return [("v2-trimcap", m, COLORS["v2-accent"])]


def v2_caphead():
    """The head hub's blue cap. Same family as the elbow cup, different
    constraints: link2-out stands 1.3 outboard of its face, so it cannot
    arrive along the axis; above, the cone's flare roofs the corridor at
    +16, so it cannot drop from the top either (both walked by the closure
    audit). It SLIDES IN FROM THE SIDE, along Y, between the drive plate
    and link2-out, and its keyhole slot -- pointing along +Y -- passes over
    the hub on the way. Glued to the hub, its flange overlaps the drive
    plate's drop-in slot rails, which is what keeps the shade from sliding
    back off the joint; and because the cap's own slot runs along Y while
    the shade's escape runs along Z, the two can never line up.

    O26, not 32: it must fit the corridor and still cover the slot rails
    (13.3 slot under a 26 flange leaves 6.3 of overlap each side)."""
    m = pl.Mesh()
    head = _slot_head(26.0)
    pocket = pl.circle(P.SHORN_HUB_D + 1.0, 48)
    slot = box(-(P.SHORN_HUB_D + 1.0) / 2, 0.0, (P.SHORN_HUB_D + 1.0) / 2, 14.0)
    m += pl.banded(head, 0.0, 6.6, [(pocket.union(slot), 0.9, 6.6 + OVL)])
    return [("v2-caphead", m, COLORS["v2-accent"])]


def v2_keycap():
    """The MX keycap, printed: a 17 square hat. Inner stem pocket grips the
    7.2 stem boss; a wider skirt relief clears the 15.6 upper housing so
    the cap can bottom out without ever touching it. Printed top-face-down,
    exactly as modelled."""
    m = pl.Mesh()
    cap = pl.rounded_rect(17.0, 17.0, 3.0)
    stem = pl.rounded_rect(P.MX_STEM_SQ + 0.2, P.MX_STEM_SQ + 0.2, 0.4)
    skirt = pl.rounded_rect(P.MX_UPPER_SQ + 0.6, P.MX_UPPER_SQ + 0.6, 0.8)
    m += pl.banded(cap, 0.0, 6.0, [
        (stem, 1.5, 4.6),
        (skirt, 4.6, 6.0 + OVL),
    ])
    return [("v2-keycap", m, COLORS["v2-accent"])]


def v2_horns():
    """Four printed MG996R horns on one strip -- pan, shoulder, elbow, head.
    They were never on a plate: every joint's drive assumed a printed horn
    that the zip did not contain."""
    import part_horn
    horn = {n: mm for n, mm, _c in part_horn.build()}["horn-mg996r"]
    m = pl.Mesh()
    for i in range(4):
        q = horn.copy()
        q.translate(dx=(i % 2) * 52.0, dy=(i // 2) * 52.0)
        m += q
    return [("v2-horns", m, COLORS["v2-accent"])]


def clamp_bars():
    """Servo tab clamp bars: pan (2) + shoulder (2) + elbow (2)."""
    m = pl.Mesh()
    for i in range(6):
        b = affinity.translate(pl.rounded_rect(12.0, 8.0, 2.0), i * 16.0, 0)
        h = affinity.translate(pl.circle(PB_CLEAR, 24), i * 16.0, 0)
        m += pl.prism(b.difference(h), 0.0, 4.0)
    return [("v2-clamps", m, COLORS["v2-clamp"])]
