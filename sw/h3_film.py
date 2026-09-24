"""h3_film.py -- film HOTARU 3 being built in SolidWorks, for the timelapse.

    python sw/h3_film.py      # -> recordings/hotaru3-build-live.mp4 (real time)
                              #    recordings/hotaru3-timelapse.mp4  (sped up)

Records ONLY the SolidWorks window (Windows Graphics Capture by window
handle), while every part is built from scratch, then the finished lamp
glides through its poses. Runs on this project's own SolidWorks process
(swlib.dedicated_session), never on a session another tool is using.
"""
from __future__ import annotations

import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3_build as B
import h3_replay as R
import swlib as L


def speed_up(src, dst, factor=4.0):
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-i", src,
                    "-vf", f"setpts=PTS/{factor},fps=30,scale=1920:-2:flags=lanczos,format=yuv420p",
                    "-an", "-c:v", "libx264", "-crf", "20", "-preset", "slow", dst], check=True)
    return dst


def main():
    sw = L.SW(pace=0.35)
    sw.close_all()
    sw.to_front()
    time.sleep(1.0)
    raw = os.path.join(B.REC, "hotaru3-build-live.mp4")
    rec = L.Recorder(sw.hwnd(), raw, fps=15)
    time.sleep(1.5)
    try:
        B.build(list(B.catalogue().keys()), record=False)
        sw.to_front()
        R.replay_assembly(sw)
        time.sleep(2.0)
    finally:
        print("raw recording:", rec.stop(), flush=True)
    out = speed_up(raw, os.path.join(B.REC, "hotaru3-timelapse.mp4"))
    print("timelapse:", out)


if __name__ == "__main__":
    main()
