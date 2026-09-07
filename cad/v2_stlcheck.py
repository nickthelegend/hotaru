"""v2_stlcheck.py -- can a slicer actually READ what we shipped?

Every other audit in this repo reasons about the MESH: is it one piece, is
it watertight, does it fit the bed. Not one of them opens the file that
leaves the building. So when a project rename pushed the binary STL header
from 80 bytes to 82, all ten audits still passed and Bambu Studio said

    "The file does not contain any geometry data."

because the triangle count was being read two bytes late. The geometry was
perfect. The container was not.

A binary STL is exactly: 80 bytes of header, a uint32 triangle count, then
count * 50 bytes. That is checkable arithmetic, so check it.

    .venv/bin/python cad/v2_stlcheck.py
"""
from __future__ import annotations

import os
import struct
import sys

EXP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "exports")


def read_stl(path):
    """Return (n_declared, n_by_size, size). Raises on a short file."""
    size = os.path.getsize(path)
    with open(path, "rb") as fh:
        head = fh.read(80)
        if len(head) < 80:
            raise ValueError(f"shorter than an STL header: {len(head)} bytes")
        (n,) = struct.unpack("<I", fh.read(4))
    # what the file's own length says the count must be
    by_size = (size - 84) / 50.0
    return n, by_size, size


def check(path):
    n, by_size, size = read_stl(path)
    ok = by_size == int(by_size) and int(by_size) == n and n > 0
    return ok, n, by_size, size


def main():
    roots = [os.path.join(EXP, "v2"), EXP]
    files = []
    for r in roots:
        if not os.path.isdir(r):
            continue
        for f in sorted(os.listdir(r)):
            if f.endswith(".stl"):
                files.append(os.path.join(r, f))

    print("HOTARU 2.0 -- is every shipped STL readable?")
    print("=" * 62)
    print(f"{'file':34s} {'triangles':>10s}  verdict")
    bad = []
    for p in files:
        try:
            ok, n, by_size, _size = check(p)
        except Exception as e:                       # noqa: BLE001
            print(f"  FAIL {os.path.basename(p):28s} {'--':>10s}  {e}")
            bad.append(p)
            continue
        if ok:
            print(f"  ok   {os.path.basename(p):28s} {n:10d}")
        else:
            print(f"  FAIL {os.path.basename(p):28s} {n:10d}  "
                  f"file length implies {by_size:.2f} triangles")
            bad.append(p)

    # ---- self-test: the check must FAIL on the exact bug it exists for ----
    # 82-byte header, which is what "aibo" -> "hotaru" did to it.
    import tempfile
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as t:
        t.write(b"hotaru partlib" + b"\0" * 68)      # 82, not 80
        t.write(struct.pack("<I", 1))
        t.write(b"\0" * 50)
        probe = t.name
    try:
        caught = not check(probe)[0]
    except Exception:                                # noqa: BLE001
        caught = True
    os.unlink(probe)
    print("=" * 62)
    if not caught:
        print("SELF-TEST FAILED: an 82-byte header slipped through, so a PASS")
        print("from this check means nothing. Fix the check before trusting it.")
        return 1
    print("[self-test] an 82-byte header is rejected -- the check has teeth")

    if bad:
        print(f"\n{len(bad)} unreadable STL(s) -- a slicer will refuse these")
        return 1
    print(f"\nall {len(files)} STLs parse: header 80, count matches file length")
    return 0


if __name__ == "__main__":
    sys.exit(main())
