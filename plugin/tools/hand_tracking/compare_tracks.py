"""Compare two hand tracking result logs (jsonl with seq + hands[].landmarks normalized x,y,z) per frame.

usage: compare_tracks.py REF.jsonl TEST.jsonl [--w 640 --h 360]
Hands are matched per frame by wrist distance. Reports detection agreement and landmark error in pixels,
split by the recording phases of run_recording.ps1.
"""
import argparse
import json

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("ref")
ap.add_argument("test")
ap.add_argument("--w", type=int, default=640)
ap.add_argument("--h", type=int, default=360)
ap.add_argument("--fps", type=float, default=30.0)
args = ap.parse_args()


def load(path):
    rows = {}
    for line in open(path):
        if line.strip():
            r = json.loads(line)
            rows[r["seq"]] = [np.array(h["landmarks"]).reshape(21, 3)[:, :2] * [args.w, args.h] for h in r["hands"]]
    return rows


ref, test = load(args.ref), load(args.test)
seqs = sorted(set(ref) & set(test))
t0 = seqs[0]
phases = [(0, 6, "out of view"), (6, 16, "one hand moving"), (16, 24, "hold 40/70cm"), (24, 36, "two hands poses"),
          (36, 52, "fast / edges")]
print(f"frames: ref {len(ref)} test {len(test)} common {len(seqs)}")

# Phase by seq: recording seq ~ 30 per second
def phase_of(seq):
    t = (seq - t0) / args.fps
    for a, b, n in phases:
        if a <= t < b:
            return n
    return "end"


stats = {}
for s in seqs:
    r, t = ref[s], test[s]
    p = phase_of(s)
    st = stats.setdefault(p, dict(frames=0, ref_hands=0, test_hands=0, matched=0, errs=[], wrist=[]))
    st["frames"] += 1
    st["ref_hands"] += len(r)
    st["test_hands"] += len(t)
    used = set()
    for hr in r:
        best, bd = None, 1e9
        for j, ht in enumerate(t):
            if j in used:
                continue
            d = np.linalg.norm(hr[0] - ht[0])
            if d < bd:
                best, bd = j, d
        if best is not None and bd < 60:
            used.add(best)
            st["matched"] += 1
            st["errs"].append(np.linalg.norm(hr - t[best], axis=1).mean())
            st["wrist"].append(bd)

print(f"{'phase':18s} {'frames':>6s} {'ref':>5s} {'test':>5s} {'match':>5s} {'recall':>6s} {'prec':>6s} {'mean_lm_err_px':>14s} {'p95':>6s}")
for _, _, name in phases + [(0, 0, "end")]:
    if name not in stats:
        continue
    st = stats[name]
    e = np.array(st["errs"]) if st["errs"] else np.array([np.nan])
    rec = st["matched"] / max(1, st["ref_hands"])
    prec = st["matched"] / max(1, st["test_hands"])
    print(f"{name:18s} {st['frames']:6d} {st['ref_hands']:5d} {st['test_hands']:5d} {st['matched']:5d} {rec:6.2f} {prec:6.2f} {np.nanmean(e):14.2f} {np.nanpercentile(e, 95):6.2f}")
