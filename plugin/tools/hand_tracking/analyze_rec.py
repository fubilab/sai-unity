"""Analyze a recording made by the sai-unity harness (color.raw, vio.jsonl, hands_live.jsonl)."""
import json
import struct
import sys
from pathlib import Path

import numpy as np

rec = Path(sys.argv[1])
HDR = struct.Struct("<qddiiii")  # seq, deviceTs, hostTime, w, h, dataSize, reserved


def read_frames(path):
    """Yield (header dict, offset) without loading pixel data."""
    out = []
    with open(path, "rb") as f:
        f.seek(0, 2)
        size = f.tell()
        f.seek(0)
        while f.tell() + HDR.size <= size:
            raw = f.read(HDR.size)
            seq, dts, host, w, h, n, _ = HDR.unpack(raw)
            off = f.tell()
            if off + n > size:
                break
            out.append(dict(seq=seq, dev=dts, host=host, w=w, h=h, n=n, off=off))
            f.seek(n, 1)
    return out


frames = read_frames(rec / "color.raw")
fh = np.array([f["host"] for f in frames])
fseq = np.array([f["seq"] for f in frames])
print(f"color frames: {len(frames)}  {frames[0]['w']}x{frames[0]['h']}  "
      f"span {fh[-1] - fh[0]:.1f}s  mean fps {len(frames) / (fh[-1] - fh[0]):.1f}")
print(f"  seq range {fseq[0]}..{fseq[-1]}  missing {fseq[-1] - fseq[0] + 1 - len(frames)}")
dts = np.diff(fh) * 1000
print(f"  host inter-frame ms: median {np.median(dts):.1f} p95 {np.percentile(dts, 95):.1f} max {dts.max():.1f}")

vio = [json.loads(l) for l in open(rec / "vio.jsonl") if l.strip()]
vh = np.array([v["host"] for v in vio])
print(f"vio outputs: {len(vio)}  rate {len(vio) / (vh[-1] - vh[0]):.1f} Hz  "
      f"status counts { {s: sum(1 for v in vio if v['status'] == s) for s in set(v['status'] for v in vio)} }")
intr = next((v["rgb_intrinsics"] for v in vio if "rgb_intrinsics" in v), None)
print("rgb intrinsics:", intr)

hands = [json.loads(l) for l in open(rec / "hands_live.jsonl") if l.strip()]
print(f"hand outputs: {len(hands)}  rate {len(hands) / (hands[-1]['host'] - hands[0]['host']):.1f} Hz")

# Latency of the live (old) native tracker: output host time - arrival host time of its source frame.
arrival = {f["seq"]: f["host"] for f in frames}
lat = [(h["host"] - arrival[h["seq"]]) * 1000 for h in hands if h["seq"] in arrival]
lag = [h["latest_color_seq"] - h["seq"] for h in hands if h["latest_color_seq"] >= 0]
print(f"live latency ms (frame arrival -> hand output): median {np.median(lat):.0f} p95 {np.percentile(lat, 95):.0f} max {np.max(lat):.0f}")
print(f"live seq lag (frames): median {np.median(lag):.0f} p95 {np.percentile(lag, 95):.0f}")

# Per-phase counts. Timeline offsets are relative to the first color frame.
phases = [(0, 6, "out of view"), (6, 16, "one hand moving"), (16, 20, "hold ~40cm"), (20, 24, "hold ~70cm"),
          (24, 28, "two hands open"), (28, 32, "fists"), (32, 36, "peace"), (36, 44, "fast motion"),
          (44, 52, "edges in/out")]
t0 = fh[0]
print("\nper-phase (live old tracker): outputs, mean #hands, frac with >=1, frac with 2")
for a, b, name in phases:
    sel = [h for h in hands if a <= h["host"] - t0 < b]
    if not sel:
        print(f"  {name:16s} no outputs")
        continue
    n = np.array([len(h["hands"]) for h in sel])
    print(f"  {name:16s} outputs {len(sel):3d}  mean_hands {n.mean():.2f}  >=1: {np.mean(n >= 1):.2f}  ==2: {np.mean(n == 2):.2f}")

# XYZ availability
xyz = [(h["host"] - t0, d["xyz_mm"]) for h in hands for d in h["hands"] if d["has_xyz"]]
print(f"\nhands with spatial xyz: {len(xyz)} of {sum(len(h['hands']) for h in hands)}")
if xyz:
    z = np.array([x[1][2] for x in xyz])
    print(f"  z mm median {np.median(z):.0f} p5 {np.percentile(z, 5):.0f} p95 {np.percentile(z, 95):.0f}")
