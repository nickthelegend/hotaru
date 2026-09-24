"""swlib.py -- drive SolidWorks 2026 from Python, early-bound through pywin32.

Every length in this module's API is in MILLIMETRES; SolidWorks wants metres,
so each coordinate goes through mm() exactly once, at the COM boundary.

    sw = SW()                         # attach to (or start) SolidWorks
    p = sw.new_part("link1")
    with p.sketch("XY") as s:         # Front plane, normal +Z
        s.circle(0, 0, 10)
    p.extrude(0, 5)                   # z 0..5 along the plane normal
    p.save(r"D:\\...\\link1.SLDPRT"); p.stl(r"D:\\...\\link1.stl")

Why early binding: late-bound pywin32 cannot tell SolidWorks properties from
methods (sw.RevisionNumber() raised "str is not callable") and gets ByRef
arguments wrong. makepy wrappers of sldworks.tlb carry the real signatures,
but they call by DISPID, so every object must first be QueryInterface'd to
the interface whose DISPIDs we are using -- that is what cast() is for.
"""
from __future__ import annotations

import math
import os
import subprocess
import time
from contextlib import contextmanager

import pythoncom
import win32com.client
from win32com.client import gencache

SW_DIR = r"C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS"
TEMPLATES = r"C:\ProgramData\SOLIDWORKS\SOLIDWORKS 2026\templates"


def _typelib(name):
    lib = pythoncom.LoadTypeLib(os.path.join(SW_DIR, name))
    a = lib.GetLibAttr()
    return gencache.EnsureModule(str(a[0]), a[1], a[3], a[4])


SWM = _typelib("sldworks.tlb")
SWC = _typelib("swconst.tlb").constants


def mm(v):
    return v / 1000.0


def cast(obj, iface):
    """QueryInterface `obj` to SolidWorks interface `iface` and wrap it."""
    if obj is None:
        return None
    cls = getattr(SWM, iface)
    o = obj._oleobj_ if hasattr(obj, "_oleobj_") else obj
    return cls(o.QueryInterface(cls.CLSID, pythoncom.IID_IDispatch))


# ------------------------------------------------------------------ planes --
# name, and how a sketch point (u, v) on it maps to model (x, y, z). Verified
# by sw/selftest.py against sketch-to-model transforms, not assumed.
PLANES = {
    "XY": ("Front Plane", lambda u, v, w: (u, v, w)),
    "XZ": ("Top Plane",   lambda u, v, w: (u, w, -v)),    # normal +Y
    "YZ": ("Right Plane", lambda u, v, w: (w, v, -u)),    # normal +X
}


class Sketch:
    """Collects 2D entities while a sketch is open. Coordinates are sketch
    (u, v) in mm. Closed loops are what extrude/cut consume."""

    def __init__(self, part, xf=None):
        """`xf` = (a, b, c, d, e, f) maps the caller's 2D frame (p, q) to
        sketch (u, v) = (a p + b q + c, d p + e q + f). A mirroring map
        (the Top plane seen as world x-z is one) reverses every arc, so arc
        directions are flipped here and callers never think about it."""
        self.p = part
        self.skm = part.skm
        self.n = 0
        self.xf = xf or (1, 0, 0, 0, 1, 0)
        a, b, _c, d, e, _f = self.xf
        self.mirror = (a * e - b * d) < 0

    def _m(self, p, q):
        a, b, c, d, e, f = self.xf
        return mm(a * p + b * q + c), mm(d * p + e * q + f)

    # primitives -------------------------------------------------------
    def line(self, u0, v0, u1, v1):
        self.n += 1
        (x0, y0), (x1, y1) = self._m(u0, v0), self._m(u1, v1)
        return self.skm.CreateLine(x0, y0, 0, x1, y1, 0)

    def cline(self, u0, v0, u1, v1):
        self.n += 1
        (x0, y0), (x1, y1) = self._m(u0, v0), self._m(u1, v1)
        return self.skm.CreateCenterLine(x0, y0, 0, x1, y1, 0)

    def circle(self, uc, vc, r):
        self.n += 1
        x, y = self._m(uc, vc)
        return self.skm.CreateCircleByRadius(x, y, 0, mm(r))

    def arc(self, uc, vc, u0, v0, u1, v1, ccw=True):
        self.n += 1
        (xc, yc), (x0, y0), (x1, y1) = (self._m(uc, vc), self._m(u0, v0),
                                         self._m(u1, v1))
        if self.mirror:
            ccw = not ccw
        return self.skm.CreateArc(xc, yc, 0, x0, y0, 0, x1, y1, 0,
                                  1 if ccw else -1)

    def spline(self, pts):
        from win32com.client import VARIANT
        self.n += 1
        flat = []
        for (u, v) in pts:
            x, y = self._m(u, v)
            flat += [x, y, 0.0]
        return self.skm.CreateSpline2(
            VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, flat), True)

    def segments(self, segs):
        """Draw ('line', p0, p1) / ('arc', c, p0, p1, ccw) / ('spline', pts)."""
        for sg in segs:
            if sg[0] == "line":
                self.line(sg[1][0], sg[1][1], sg[2][0], sg[2][1])
            elif sg[0] == "spline":
                self.spline(sg[1])
            else:
                c, p0, p1, ccw = sg[1], sg[2], sg[3], sg[4]
                self.arc(c[0], c[1], p0[0], p0[1], p1[0], p1[1], ccw)

    def ring(self, uc, vc, r_out, r_in):
        self.circle(uc, vc, r_out)
        self.circle(uc, vc, r_in)

    def rect(self, u0, v0, u1, v1):
        self.poly([(u0, v0), (u1, v0), (u1, v1), (u0, v1)])

    def crect(self, uc, vc, w, h, r=0.0):
        """Centred rectangle, optional corner radius."""
        pts = [(uc - w / 2, vc - h / 2), (uc + w / 2, vc - h / 2),
               (uc + w / 2, vc + h / 2), (uc - w / 2, vc + h / 2)]
        self.path(pts, r)

    def poly(self, pts):
        for a, b in zip(pts, pts[1:] + pts[:1]):
            self.line(a[0], a[1], b[0], b[1])

    def hexagon(self, uc, vc, af, rot=0.0):
        """Regular hexagon by ACROSS-FLATS -- how a nut is specified."""
        r = af / 2 / math.cos(math.pi / 6)
        pts = [(uc + r * math.cos(math.radians(rot + 30 + 60 * k)),
                vc + r * math.sin(math.radians(rot + 30 + 60 * k)))
               for k in range(6)]
        self.poly(pts)

    def slot(self, u0, v0, u1, v1, w):
        """Straight slot (stadium) between two centres, width w."""
        r = w / 2
        dx, dy = u1 - u0, v1 - v0
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return self.circle(u0, v0, r)
        nx, ny = -dy / L * r, dx / L * r
        self.line(u0 + nx, v0 + ny, u1 + nx, v1 + ny)
        self.arc(u1, v1, u1 + nx, v1 + ny, u1 - nx, v1 - ny, ccw=False)
        self.line(u1 - nx, v1 - ny, u0 - nx, v0 - ny)
        self.arc(u0, v0, u0 - nx, v0 - ny, u0 + nx, v0 + ny, ccw=False)

    def path(self, pts, radii=0.0):
        """Closed polygon with a fillet radius per vertex (scalar or list).
        Fillets are solved here and drawn as true arcs, so the sketch reads
        as designed geometry, not a tessellation."""
        n = len(pts)
        rs = radii if isinstance(radii, (list, tuple)) else [radii] * n
        segs = []   # per vertex: (tangent-in point, tangent-out point, centre, ccw)
        for i in range(n):
            p0, p1, p2 = pts[i - 1], pts[i], pts[(i + 1) % n]
            r = rs[i]
            if r <= 0:
                segs.append((p1, p1, None, None))
                continue
            a = _unit(p0[0] - p1[0], p0[1] - p1[1])
            b = _unit(p2[0] - p1[0], p2[1] - p1[1])
            cosang = max(-1.0, min(1.0, a[0] * b[0] + a[1] * b[1]))
            ang = math.acos(cosang)
            t = r / math.tan(ang / 2)
            bis = _unit(a[0] + b[0], a[1] + b[1])
            d = r / math.sin(ang / 2)
            c = (p1[0] + bis[0] * d, p1[1] + bis[1] * d)
            tin = (p1[0] + a[0] * t, p1[1] + a[1] * t)
            tout = (p1[0] + b[0] * t, p1[1] + b[1] * t)
            cross = (p1[0] - p0[0]) * (p2[1] - p1[1]) - (p1[1] - p0[1]) * (p2[0] - p1[0])
            segs.append((tin, tout, c, cross > 0))
        for i in range(n):
            tin, tout, c, ccw = segs[i]
            if c is not None:
                self.arc(c[0], c[1], tin[0], tin[1], tout[0], tout[1], ccw=ccw)
            nxt = segs[(i + 1) % n][0]
            if math.hypot(nxt[0] - tout[0], nxt[1] - tout[1]) > 1e-4:
                self.line(tout[0], tout[1], nxt[0], nxt[1])

    def polyline_closed(self, pts):
        """Dense closed polyline (gear teeth): straight segments.

        Drawn with the view's graphics updates OFF. A 54-tooth gear is ~850
        lines, and redrawing the growing sketch after each one crashed
        SolidWorks twice (slddtlu.dll access violation mid-CreateLine)."""
        v = None
        try:
            v = cast(self.p.doc.ActiveView, "IModelView")
            v.EnableGraphicsUpdate = False
        except Exception:
            v = None
        try:
            self.poly(pts)
        finally:
            if v is not None:
                v.EnableGraphicsUpdate = True

    def circles(self, pts, r):
        for (u, v) in pts:
            self.circle(u, v, r)


def _unit(x, y):
    L = math.hypot(x, y)
    return (x / L, y / L)


class Part:
    def __init__(self, sw, doc, name):
        self.sw = sw
        self.doc = cast(doc, "IModelDoc2")
        self.ext = cast(self.doc.Extension, "IModelDocExtension")
        self.skm = cast(self.doc.SketchManager, "ISketchManager")
        self.fm = cast(self.doc.FeatureManager, "IFeatureManager")
        self.name = name
        self._plane = "XY"
        self.feats = []

    # --------------------------------------------------------- sketching --
    @contextmanager
    def sketch(self, plane="XY", name=None, xf=None):
        """Open a sketch. plane is XY / XZ / YZ, or one of the named
        conventions: 'wXZ' takes world (x, z), 'wYZ' takes world (y, z)."""
        self.doc.ClearSelection2(True)
        if plane == "wXZ":
            plane, xf = "XZ", (1, 0, 0, 0, -1, 0)
        elif plane == "wYZ":
            plane, xf = "YZ", (0, -1, 0, 1, 0, 0) if xf is None else xf
        pname, _f = PLANES[plane]
        ok = self.ext.SelectByID2(pname, "PLANE", 0, 0, 0, False, 0, None, 0)
        if not ok:
            raise RuntimeError(f"could not select {pname}")
        self.skm.InsertSketch(True)
        self.skm.AddToDB = True
        s = Sketch(self, xf)
        try:
            yield s
        finally:
            self.skm.AddToDB = False
            self.skm.InsertSketch(True)          # close
            f = cast(self.doc.FeatureByPositionReverse(0), "IFeature")
            if name:
                f.Name = name
            self._last_sketch = f
            self._plane = plane
            self.sw.frame()

    def _select_last_sketch(self):
        self.doc.ClearSelection2(True)
        self._last_sketch.Select2(False, 0)

    def _done(self, feat, what, name=None):
        if feat is None:
            raise RuntimeError(f"{self.name}: {what} failed "
                               f"(sketch {self._last_sketch.Name})")
        f = cast(feat, "IFeature")
        if name:
            f.Name = name
        self.feats.append(f.Name)
        self.doc.ClearSelection2(True)
        self.doc.ViewZoomtofit2()          # keep the growing part framed
        self.doc.GraphicsRedraw2()
        self.sw.tick()
        self.sw.frame()
        return f

    # ---------------------------------------------------------- features --
    def extrude(self, z0, z1, name=None, merge=True):
        """Boss from plane-offset z0 to z1 along the sketch plane normal."""
        return self._ex(False, z0, z1, name, merge)

    def cut(self, z0, z1, name=None):
        return self._ex(True, z0, z1, name, True)

    def cut_all(self, name=None):
        """Through-all both ways from the sketch plane."""
        self._select_last_sketch()
        f = self.fm.FeatureCut4(False, False, False, 9, 9, 0.01, 0.01,
                                False, False, False, False, 0, 0,
                                False, False, False, False, False, True, True,
                                True, True, False, 0, 0, False, False)
        return self._done(f, "cut-through-all", name)

    def _ex(self, is_cut, z0, z1, name, merge):
        lo, hi = min(z0, z1), max(z0, z1)
        depth = hi - lo
        self._select_last_sketch()
        t0 = 0 if abs(lo) < 1e-9 else 3               # sketch plane | offset
        flip_start = lo < 0
        # Probed (see selftest): an extrude's Dir=False runs ALONG the plane
        # normal, a cut's Dir=False runs AGAINST it. Both are asked for +n.
        if is_cut:
            f = self.fm.FeatureCut4(True, False, True, 0, 0, mm(depth), 0,
                                    False, False, False, False, 0, 0,
                                    False, False, False, False, False, True, True,
                                    True, True, False, t0, mm(abs(lo)), flip_start, False)
        else:
            f = self.fm.FeatureExtrusion3(True, False, False, 0, 0, mm(depth), 0,
                                          False, False, False, False, 0, 0,
                                          False, False, False, False, merge, True, True,
                                          t0, mm(abs(lo)), flip_start)
        return self._done(f, "cut" if is_cut else "extrude", name)

    def revolve(self, angle=360.0, cut=False, name=None):
        """Revolve the last sketch about its (single) centreline."""
        self._select_last_sketch()
        f = self.fm.FeatureRevolve2(True, True, False, cut, False, False,
                                    0, 0, math.radians(angle), 0,
                                    False, False, 0, 0, 0, 0, 0,
                                    True, True, True)
        return self._done(f, "revolve", name)

    # ---------------------------------------------------------- finishing --
    def colour(self, hexrgb, shine=0.3):
        """Part colour. The values MUST go over as a typed VT_R8 array:
        assigned as a plain list, pywin32 marshals them so that SolidWorks
        reads (0, r, 0, garbage, g, ...) and the part renders see-through."""
        from win32com.client import VARIANT
        r, g, b = (int(hexrgb[i:i + 2], 16) / 255 for i in (1, 3, 5))
        vals = [r, g, b, 1.0, 1.0, shine, 0.3, 0.0, 0.0]
        self.ext.SetMaterialPropertyValues(
            VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, vals), 2, None)

    def material(self, name="PLA"):
        pd = cast(self.doc, "IPartDoc")
        for lib in ("SOLIDWORKS Materials", ""):
            try:
                pd.SetMaterialPropertyName2("", lib, name)
            except Exception:
                continue
            if pd.GetMaterialPropertyName2("", "")[0] if isinstance(
                    pd.GetMaterialPropertyName2("", ""), tuple) else True:
                return

    def mass(self):
        """(mass g, volume mm3, centre of mass mm) from SolidWorks' own
        mass properties -- solid part, no infill discount."""
        mp = cast(self.ext.CreateMassProperty(), "IMassProperty")
        c = mp.CenterOfMass
        return mp.Mass * 1000.0, mp.Volume * 1e9, tuple(v * 1000 for v in c)

    def view(self, iso=True):
        if iso:
            self.doc.ShowNamedView2("*Isometric", 7)
        self.doc.ViewZoomtofit2()
        self.doc.GraphicsRedraw2()

    def save(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        r = self.ext.SaveAs3(path, 0, 1, None, None, 0, 0)
        ok = r[0] if isinstance(r, tuple) else r
        if not ok:
            raise RuntimeError(f"save failed: {path} -> {r}")
        return path

    def stl(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.sw.app.SetUserPreferenceIntegerValue(SWC.swExportStlUnits, SWC.swMM)
        self.sw.app.SetUserPreferenceIntegerValue(SWC.swSTLQuality, SWC.swSTLQuality_Fine)
        self.sw.app.SetUserPreferenceToggle(SWC.swSTLBinaryFormat, True)
        self.sw.app.SetUserPreferenceToggle(SWC.swSTLDontTranslateToPositive, True)
        r = self.ext.SaveAs3(path, 0, 1 | 2, None, None, 0, 0)
        ok = r[0] if isinstance(r, tuple) else r
        if not ok:
            raise RuntimeError(f"stl export failed: {path} -> {r}")
        return path

    def close(self):
        self.sw.app.CloseDoc(self.doc.GetTitle())


PID_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".sw_pid")


def _rot_instance(pid):
    """The SolidWorks session with this process id, from the Running Object
    Table (every SOLIDWORKS process registers itself there as
    'SolidWorks_PID_<pid>'), or None."""
    rot = pythoncom.GetRunningObjectTable()
    ctx = pythoncom.CreateBindCtx(0)
    for mk in rot.EnumRunning():
        try:
            name = mk.GetDisplayName(ctx, None)
        except pythoncom.com_error:
            continue
        if name.strip("!") == f"SolidWorks_PID_{pid}":
            return rot.GetObject(mk).QueryInterface(pythoncom.IID_IDispatch)
    return None


def _alive(pid):
    import ctypes
    h = ctypes.windll.kernel32.OpenProcess(0x1000, False, pid)   # QUERY_LIMITED
    if not h:
        return False
    code = ctypes.c_ulong()
    ctypes.windll.kernel32.GetExitCodeProcess(h, ctypes.byref(code))
    ctypes.windll.kernel32.CloseHandle(h)
    return code.value == 259                                    # STILL_ACTIVE


def dedicated_session(timeout=240):
    """A SolidWorks process that belongs to THIS project alone.

    Dispatch("SldWorks.Application") attaches to whatever SolidWorks is
    already running -- and another automation session on this machine was
    driving that same instance, each side closing the other's documents
    (every RPC_E_DISCONNECTED in the build logs traces back to it). So this
    project starts its own SLDWORKS.exe, remembers its pid, and always
    reattaches to exactly that process through the ROT."""
    if os.path.exists(PID_FILE):
        pid = int(open(PID_FILE).read().strip() or 0)
        if pid and _alive(pid):
            d = _rot_instance(pid)
            if d is not None:
                return d, pid
    proc = subprocess.Popen([os.path.join(SW_DIR, "SLDWORKS.exe")])
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = _rot_instance(proc.pid)
        if d is not None:
            open(PID_FILE, "w").write(str(proc.pid))
            return d, proc.pid
        time.sleep(2)
    raise RuntimeError("dedicated SolidWorks did not register in the ROT")


class SW:
    def __init__(self, pace=0.0):
        raw, self.pid = dedicated_session()
        self.app = cast(raw, "ISldWorks")
        self.app.Visible = True
        self.app.UserControl = True
        self.pace = pace      # seconds to hold after each feature (timelapse)
        self.frames = None    # directory: when set, every step is a frame
        self._fn = 0
        # Scripted modelling settings. View animation MUST be off: zooming
        # after every feature while SolidWorks was still animating the last
        # view change crashed it (slddtlu.dll access violation, 23:08).
        self.app.SetUserPreferenceDoubleValue(SWC.swViewAnimationSpeed, 0.0)
        self.app.SetUserPreferenceToggle(SWC.swSketchInference, False)
        self.app.SetUserPreferenceToggle(SWC.swSketchAutomaticRelations, False)
        self.app.SetUserPreferenceToggle(SWC.swInputDimValOnCreate, False)

    def tick(self):
        if self.pace:
            time.sleep(self.pace)

    def start_frames(self, directory):
        """Timelapse by single frames. A CONTINUOUS window capture running
        while the API sketches knocked SolidWorks' COM objects offline
        (RPC_E_DISCONNECTED, five builds in a row; the same builds pass
        with it off). One frame per design step, grabbed while SolidWorks
        is idle between calls, is stable -- and a step per frame is what a
        timelapse wants anyway."""
        os.makedirs(directory, exist_ok=True)
        self.frames = directory
        self._fn = len([f for f in os.listdir(directory) if f.endswith(".png")])

    def frame(self):
        if not self.frames:
            return
        try:
            snap(self.hwnd(), os.path.join(self.frames, f"{self._fn:05d}.png"), 1920)
            self._fn += 1
        except Exception:
            pass

    def to_front(self):
        """Maximise SolidWorks and bring it in front of everything else, so
        the person watching sees the build happen."""
        import ctypes
        h = self.hwnd()
        u = ctypes.windll.user32
        u.ShowWindow(h, 3)                               # SW_MAXIMIZE
        fg = u.GetForegroundWindow()
        t_fg = u.GetWindowThreadProcessId(fg, None)
        t_me = ctypes.windll.kernel32.GetCurrentThreadId()
        u.AttachThreadInput(t_me, t_fg, True)
        u.BringWindowToTop(h)
        u.SetForegroundWindow(h)
        u.AttachThreadInput(t_me, t_fg, False)

    def hwnd(self):
        fr = cast(self.app.Frame(), "IFrame")
        return int(fr.GetHWndx64())

    def new_part(self, name):
        doc = self.app.NewDocument(os.path.join(TEMPLATES, "Part.PRTDOT"), 0, 0, 0)
        if doc is None:
            raise RuntimeError("NewDocument(part) failed")
        p = Part(self, doc, name)
        # millimetres in the UI too, so the feature tree reads in mm
        p.doc.SetUserPreferenceIntegerValue(SWC.swUnitsLinear, SWC.swMM)
        p.doc.ShowNamedView2("*Isometric", 7)
        return p

    def new_assembly(self):
        doc = self.app.NewDocument(os.path.join(TEMPLATES, "Assembly.ASMDOT"), 0, 0, 0)
        return cast(doc, "IModelDoc2")

    def close_all(self):
        self.app.CloseAllDocuments(True)


# --------------------------------------------------------------- recorder --
class Recorder:
    """Record ONE window (SolidWorks) with Windows Graphics Capture.

    gfxcapture reads the window's own composition surface, so it keeps
    recording when other windows cover SolidWorks -- and it cannot capture
    anything that is not that window, which is the point: nothing else on
    the desktop ends up in the timelapse."""

    def __init__(self, hwnd, path, fps=10):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path = path
        src = (f"gfxcapture=hwnd={hwnd}:max_framerate={fps}:capture_cursor=0,"
               f"hwdownload,format=bgra,format=yuv420p")
        self.proc = subprocess.Popen(
            ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
             "-filter_complex", src, "-c:v", "libx264", "-preset", "veryfast",
             "-crf", "22", "-fps_mode", "vfr", path],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    def stop(self):
        if self.proc.poll() is None:
            try:
                self.proc.communicate(b"q", timeout=15)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        return self.path


def snap(hwnd, path, width=1280):
    """One frame of the SolidWorks window (only that window) to a PNG."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    src = (f"gfxcapture=hwnd={hwnd}:max_framerate=5:capture_cursor=0,"
           f"hwdownload,format=bgra,scale={width}:-2")
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                    "-filter_complex", src, "-frames:v", "1", path],
                   check=True, timeout=30)
    return path


def timelapse(frames_dir, out_path, fps=6):
    """Stitch the step frames into an H.264 timelapse."""
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                    "-framerate", str(fps), "-i", os.path.join(frames_dir, "%05d.png"),
                    "-vf", "scale=1920:-2:flags=lanczos,format=yuv420p",
                    "-c:v", "libx264", "-crf", "18", "-preset", "slow", out_path],
                   check=True)
    return out_path
