"""h3_replay.py -- the SolidWorks timelapse, recorded AFTER the build.

    python sw/h3_replay.py          # -> recordings/hotaru3-solidworks.mp4

Why a replay and not a live recording: a Windows Graphics Capture session
on the SolidWorks window, running while the API was building, repeatedly
knocked SolidWorks' COM objects offline (RPC_E_DISCONNECTED) -- five builds
in a row failed with it on and passed with it off. So the parts are built
with nothing watching, and this script then REPLAYS each one from its own
feature tree: roll the tree back to empty, and roll it forward a feature at
a time, exactly the order it was modelled in. Then the assembly is driven
through its poses.

The recording is Desktop Duplication (ddagrab) of the screen area the
maximised SolidWorks window covers. That never touches the SolidWorks
process. SolidWorks is held TOPMOST for the length of the replay so no
other window can land in the recording, and released afterwards.
"""
from __future__ import annotations

import ctypes
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3_assembly as A
import h3_build as B
import swlib as L

u = ctypes.windll.user32
ORDER = ["h3-tub", "h3-lipring", "h3-disc", "h3-tower", "h3-pinion", "h3-link1",
         "h3-link1-cover", "h3-link2", "h3-link2-cover", "h3-head", "h3-head-cover",
         "h3-shade", "h3-pin-shoulder", "h3-keycap"]
STEP = 0.45            # seconds per feature on screen


def topmost(h, on):
    u.SetWindowPos(h, -1 if on else -2, 0, 0, 0, 0, 0x0001 | 0x0002 | 0x0040)


def screen_rect(h):
    r = (ctypes.c_long * 4)()
    u.GetWindowRect(h, r)
    x0, y0 = max(r[0], 0), max(r[1], 0)
    w, hh = (r[2] - x0) // 2 * 2, (r[3] - y0) // 2 * 2
    return x0, y0, w, hh


def start_capture(rect, path):
    x, y, w, h = rect
    src = (f"ddagrab=output_idx=0:framerate=20:draw_mouse=0:offset_x={x}:offset_y={y}"
           f":video_size={w}x{h},hwdownload,format=bgra,format=yuv420p")
    return subprocess.Popen(
        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-filter_complex", src,
         "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", path],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)


def solid_features(doc):
    """Model features in tree order, skipping sketches and the folders."""
    skip = {"ProfileFeature", "OriginProfileFeature", "RefPlane", "MaterialFolder",
            "HistoryFolder", "SensorFolder", "DocsFolder", "DetailCabinet",
            "SurfaceBodyFolder", "SolidBodyFolder", "CommentsFolder", "FavoriteFolder",
            "SelectionSetFolder", "EqnFolder", "InkMarkupFolder", "EnvFolder",
            "BlockFolder", "Attribute", "MateReferenceGroupFolder", "3DSketch"}
    out = []
    f = L.cast(doc.FirstFeature(), "IFeature")
    while f is not None:
        if f.GetTypeName2() not in skip:
            out.append(f.Name)
        nf = f.GetNextFeature()
        f = L.cast(nf, "IFeature") if nf is not None else None
    return out


def replay_part(sw, name):
    r = sw.app.OpenDoc6(os.path.join(B.PARTS, name + ".SLDPRT"), 1, 1, "", 0, 0)
    doc = L.cast(r[0], "IModelDoc2")
    fm = L.cast(doc.FeatureManager, "IFeatureManager")
    doc.ShowNamedView2("*Isometric", 7)
    doc.ViewZoomtofit2()                      # frame the FINISHED part, then
    feats = solid_features(doc)               # hold that framing throughout
    fm.EditRollback(3, feats[0])              # before the first feature
    doc.GraphicsRedraw2()
    time.sleep(STEP * 2)
    for f in feats:
        fm.EditRollback(4, f)                 # after this feature
        doc.GraphicsRedraw2()
        time.sleep(STEP)
    fm.EditRollback(1, "")
    time.sleep(STEP * 3)
    sw.app.CloseDoc(os.path.basename(doc.GetPathName()))


def replay_assembly(sw):
    lamp = A.Lamp(sw)
    order = ["neutral", "look-up", "reach", "full-reach", "curious", "pan-left",
             "pan-right", "shy", "lean-back", "fold", "rest"]
    lamp.pose(A.POSES["neutral"])
    time.sleep(1.0)
    prev = A.POSES["neutral"]
    for name in order:
        tgt = A.POSES[name]
        for k in range(1, 21):                # glide, don't jump
            a = tuple(p + (t - p) * k / 20 for p, t in zip(prev, tgt))
            lamp.pose(a, redraw=False)
            lamp.doc.GraphicsRedraw2()
            time.sleep(0.05)
        lamp.doc.ViewZoomtofit2()
        time.sleep(1.2)
        prev = tgt


def main():
    sw = L.SW()
    sw.close_all()
    sw.to_front()
    h = sw.hwnd()
    time.sleep(1.0)
    out = os.path.join(B.REC, "hotaru3-solidworks.mp4")
    os.makedirs(B.REC, exist_ok=True)
    topmost(h, True)
    cap = start_capture(screen_rect(h), out)
    try:
        time.sleep(1.0)
        for n in ORDER:
            print("replay", n, flush=True)
            replay_part(sw, n)
        print("replay assembly", flush=True)
        replay_assembly(sw)
        time.sleep(1.5)
    finally:
        try:
            cap.communicate(b"q", timeout=30)
        except subprocess.TimeoutExpired:
            cap.kill()
        topmost(h, False)
    print("video:", out)
    return out


if __name__ == "__main__":
    main()
