import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

S = Path(sys.argv[1])
tag = sys.argv[2]
OFF = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5

df = pd.read_csv(S / f"{tag}_world.csv")
ph = []
for line in open(S / f"{tag}_world.phases.txt"):
    m = re.match(r"([\d.]+)\s+(.*)", line.strip())
    ph.append((float(m.group(1)) - OFF, m.group(2)))
print("rows:", len(df), " hands:", df.hand.value_counts().to_dict(), " unity time %.1f..%.1f" % (df.unity_time.min(), df.unity_time.max()))
for t, s in ph:
    print(f"  {t:6.1f}s {s[:70]}")

# VIO status from the player log
log = (S / f"{tag}_world.log").read_text(errors="ignore").splitlines()
status = [re.search(r"VIO: (\S+) poses (\d+)", l).groups() for l in log if "[HandTracking] VIO" in l]
print("VIO status every 2 s:", " ".join(s[0][:4] for s in status))

def window(a, b):
    return df[(df.unity_time >= a) & (df.unity_time < b)]

meas = [(t, s) for t, s in ph if s.startswith("Hold as still")]
truth = [0.30, 0.50]
print("\n=== ground truth windows (lens to screen measured with ruler, camera hand-held, +-3 cm)")
for (t, _), d in zip(meas, truth):
    w = window(t + 1.0, t + 14.0)
    w = w[w.hand == "Right"]
    if w.empty:
        print(f"[{t:.0f}s] truth {d * 100:.0f} cm: no right hand rows")
        continue
    sd = w.stereo_depth[w.stereo_depth > 0]
    scale = w.hand_scale
    mono = (w.stereo_depth / w.hand_scale)[w.stereo_depth > 0]
    palm = np.linalg.norm(w[["palm_x", "palm_y", "palm_z"]].to_numpy() - w[["cam_x", "cam_y", "cam_z"]].to_numpy(), axis=1)
    print(f"[{t:5.1f}s] truth {d * 100:.0f} cm  n={len(w)}  conf {w.conf.mean():.2f}")
    print(f"    stereo depth (palm z): median {sd.median() * 100:5.1f} cm  p10 {sd.quantile(.1) * 100:5.1f}  p90 {sd.quantile(.9) * 100:5.1f}   error {100 * (sd.median() - d) / d:+.0f}%")
    print(f"    hand scale (stereo/size-based): median {scale.median():.2f}  p10 {scale.quantile(.1):.2f} p90 {scale.quantile(.9):.2f}")
    print(f"    size-based depth = stereo/scale: median {mono.median() * 100:5.1f} cm   error {100 * (mono.median() - d) / d:+.0f}%")
    print(f"    placed palm distance to camera object (calibrated): median {np.median(palm) * 100:5.1f} cm")

# Static-hand world test
a = ph[1][0]
b = ph[2][0]
w = window(a + 2, b - 1)
w = w[w.hand == "Right"].reset_index(drop=True)
print(f"\n=== part one: hand resting on the table while the device moves ({a:.0f}-{b:.0f}s), n={len(w)}")
if len(w) > 20:
    palm = w[["palm_x", "palm_y", "palm_z"]].to_numpy()
    cam = w[["cam_x", "cam_y", "cam_z"]].to_numpy()
    q = w[["cam_qx", "cam_qy", "cam_qz", "cam_qw"]].to_numpy()
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    ang = 2 * np.degrees(np.arccos(np.clip(np.abs(q @ q[0]), 0, 1)))
    dp = palm - palm.mean(0)
    dc = cam - cam.mean(0)
    print(f"camera: translation std (cm) {np.round(cam.std(0) * 100, 1)}   rotation range {ang.max() - ang.min():.0f} deg")
    print(f"hand world: std (cm) {np.round(palm.std(0) * 100, 1)}   rms {np.sqrt((dp ** 2).sum(1).mean()) * 100:.1f} cm")
    print("corr(hand, camera) per axis:", [round(float(np.corrcoef(dp[:, i], dc[:, i])[0, 1]), 2) for i in range(3)])
    print("hand scale median %.2f  stereo calibrated frac %.2f" % (w.hand_scale.median(), w.stereo_calibrated.mean()))
