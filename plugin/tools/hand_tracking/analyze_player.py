import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

S = Path(sys.argv[1])
tag = sys.argv[2]
OFFSET = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5  # unity time ~= process time - OFFSET

for name in ("ir0", "ir1"):
    csv = S / f"{tag}_player_{name}.csv"
    ph = S / f"{tag}_player_{name}.phases.txt"
    if not csv.exists():
        print(name, "no csv")
        continue
    df = pd.read_csv(csv)
    phases = []
    for line in open(ph):
        m = re.match(r"([\d.]+)\s+(.*)", line.strip())
        phases.append((float(m.group(1)) - OFFSET, m.group(2)))
    phases.append((phases[-1][0] + 5, "end"))
    print(f"\n===== run {name}: {len(df)} hand rows, unity time {df.unity_time.min():.1f}..{df.unity_time.max():.1f}s")
    print("overall: rows per hand", df.hand.value_counts().to_dict(), " stereo_calibrated frac %.2f" % df.stereo_calibrated.mean(),
          " latency ms median %.0f p90 %.0f" % (df.latency_ms.median(), df.latency_ms.quantile(.9)))
    for i in range(len(phases) - 1):
        a, text = phases[i]
        b = phases[i + 1][0]
        seg = df[(df.unity_time >= a) & (df.unity_time < b)]
        short = text[:48]
        print(f"\n[{a:5.1f}-{b:5.1f}s] {short}")
        if seg.empty:
            print("   no hands")
            continue
        print("   rows by hand:", seg.hand.value_counts().to_dict(), " conf %.2f" % seg.conf.mean(),
              " gestures:", seg.gesture.value_counts().head(2).to_dict())
        cam = seg[["cam_x", "cam_y", "cam_z"]].to_numpy()
        for hand, g in seg.groupby("hand"):
            palm = g[["palm_x", "palm_y", "palm_z"]].to_numpy()
            gc = g[["cam_x", "cam_y", "cam_z"]].to_numpy()
            dist = np.linalg.norm(palm - gc, axis=1)
            print(f"   {hand:5s} n={len(g):3d} palm distance to camera: median {np.median(dist) * 100:5.1f} cm  "
                  f"p10 {np.percentile(dist, 10) * 100:5.1f} p90 {np.percentile(dist, 90) * 100:5.1f}   "
                  f"hand world std (cm) xyz = {np.round(palm.std(0) * 100, 1)}   camera path range (cm) = "
                  f"{np.round((gc.max(0) - gc.min(0)) * 100, 1)}")
