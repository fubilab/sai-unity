"""Stationary-camera VIO noise/drift statistics from vio.jsonl files.  usage: vio_stats.py DIR [DIR...]"""
import json
import sys
from pathlib import Path

import numpy as np

for d in sys.argv[1:]:
    rows = [json.loads(l) for l in open(Path(d) / "vio.jsonl") if l.strip()]
    if len(rows) < 50:
        print(d, "too few rows", len(rows))
        continue
    t = np.array([r["t"] for r in rows])
    p = np.array([r["p"] for r in rows])
    st = np.array([r["status"] for r in rows])
    q = np.array([r["q"] for r in rows])
    # skip the initialization period
    keep = (t - t[0]) > 6.0
    t, p, st, q = t[keep], p[keep], st[keep], q[keep]
    dp = p - p.mean(0)
    # high frequency jitter: difference to a 1 s moving average
    k = 20
    sm = np.stack([np.convolve(p[:, i], np.ones(k) / k, mode="same") for i in range(3)], 1)
    jitter = np.linalg.norm((p - sm)[k:-k], axis=1)
    drift = np.linalg.norm(p[-1] - p[0])
    path = np.linalg.norm(np.diff(p, axis=0), axis=1).sum()
    # orientation change relative to the first sample (degrees)
    q0 = q[0] / np.linalg.norm(q[0])
    ang = 2 * np.degrees(np.arccos(np.clip(np.abs(q @ q0 / np.linalg.norm(q, axis=1)), 0, 1)))
    print(f"{Path(d).name:22s} n={len(t):4d} rate={len(t) / (t[-1] - t[0]):5.1f}Hz tracking={np.mean(st == 1):.2f} "
          f"pos_std={np.linalg.norm(dp, axis=1).std() * 1000:6.2f}mm jitter_rms={np.sqrt((jitter ** 2).mean()) * 1000:5.2f}mm "
          f"drift={drift * 1000:6.1f}mm path={path * 1000:7.1f}mm rot_max={ang.max():5.2f}deg")
