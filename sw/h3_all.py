"""h3_all.py -- the whole HOTARU 3 run, start to finish, one command.

    python sw/h3_all.py

  1. build every part in SolidWorks (a timelapse frame per design step)
  2. assemble, drive every named pose, SolidWorks interference at each
  3. worst-case servo torque from SolidWorks mass properties
  4. the firmware's pose guard against SolidWorks, on random poses
  5. stitch the frames into recordings/hotaru3-timelapse.mp4
"""
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3_assembly
import h3_build
import h3_guard
import h3_torque
import swlib as L

if __name__ == "__main__":
    frames = h3_build.FRAMES
    if "--keep-frames" not in sys.argv and os.path.isdir(frames):
        shutil.rmtree(frames)
    print("== 1. build", flush=True)
    rec = "--frames" in sys.argv       # off by default: see swlib.SW.start_frames
    h3_build.build(list(h3_build.catalogue().keys()), record=rec)
    print("== 2. assemble + poses", flush=True)
    h3_assembly.main(record=rec)
    print("== 3. torque", flush=True)
    h3_torque.main()
    print("== 4. guard vs SolidWorks", flush=True)
    missed = h3_guard.validate(40, record=rec)
    print("== 4b. pan sweep: the base through its whole range", flush=True)
    import h3_pan_sweep
    missed = h3_pan_sweep.main() or missed
    if rec:
        print("== 5. timelapse", flush=True)
        out = L.timelapse(frames, os.path.join(h3_build.REC, "hotaru3-timelapse.mp4"))
        print("timelapse:", out)
    sys.exit(1 if missed else 0)
