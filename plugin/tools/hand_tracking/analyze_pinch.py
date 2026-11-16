"""Analyze a guided pinch recording (pinch_session.ps1): which thumb-index feature separates pinch from the other poses.

usage: analyze_pinch.py DIR [enter exit]   (DIR has hands.jsonl and phases.json, written by pinch_session.ps1)
The last table shows, per pose, how often the rule with `enter` / `exit` on rxy (default 0.14 / 0.20, the values of
HandTracker) reports a pinch, after the 3-results confirmation. On the 2026-11-16 recording (one person) rxy separates
all pinch poses from open hand, fist, thumbs up and a small gap, the 3D ratio r3 does not (fingertip depth is unreliable).
Features, each relative to the palm length (wrist to middle finger base) so the hand size does not matter:
  r3   thumb tip to index tip in 3D (metric camera landmarks)      -- the first rule in HandTracker
  r2   same in the image plane (pixels of the color image)
  rxy  camera landmarks with the depth difference dropped (x, y only)
  rz25 camera landmarks with the depth difference scaled by 0.25
"""
import json
import sys
from pathlib import Path

import numpy as np

W, H = 640, 360
CONFIRM = 3
FEATURES = ["r3", "r2", "rxy", "rz25"]


def load(dirpath):
    d = Path(dirpath)
    meta = json.loads((d / "phases.json").read_text(encoding="utf-8-sig"))
    rows = [json.loads(l) for l in open(d / "hands.jsonl") if l.strip()]
    d0 = rows[0]["dev"]
    offset = meta["offset_first_frame_to_t0"]
    out = []
    for r in rows:
        t = (r["dev"] - d0) - offset  # seconds on the prompt timeline
        for h in r["hands"]:
            if not h["placed"] or h["lm_score"] < 0.6:
                continue
            c = np.array(h["cam"]).reshape(21, 3)
            im = np.array(h["landmarks"]).reshape(21, 3)[:, :2] * [W, H]
            palm3 = np.linalg.norm(c[0] - c[9])
            palm2 = np.linalg.norm(im[0] - im[9])
            if palm3 < 0.01 or palm2 < 5:
                continue
            d = c[4] - c[8]
            # palm normal: wrist, index base, pinky base. |nz| = how much the palm faces the camera
            n = np.cross(c[5] - c[0], c[17] - c[0])
            n /= max(np.linalg.norm(n), 1e-9)
            out.append(dict(t=t, r3=np.linalg.norm(d) / palm3, r2=np.linalg.norm(im[4] - im[8]) / palm2,
                            rxy=np.linalg.norm(d[:2]) / palm3, rz25=np.linalg.norm([d[0], d[1], 0.25 * d[2]]) / palm3,
                            facing=abs(n[2]), gesture=h["gesture"], depth=c[9][2]))
    return meta, out, rows[-1]["dev"] - d0 - offset


def stats(v):
    v = np.asarray(v)
    return f"p5 {np.percentile(v, 5):4.2f} med {np.median(v):4.2f} p95 {np.percentile(v, 95):4.2f}"


def simulate(values, enter, exit_, confirm=CONFIRM):
    """HandTracker logic: hysteresis on the ratio, then the stable state needs `confirm` results in a row."""
    pinching, stable, cand, count = False, False, False, 0
    flags = []
    for r in values:
        if not pinching and r < enter:
            pinching = True
        elif pinching and r > max(exit_, enter):
            pinching = False
        if pinching == stable:
            cand, count = pinching, 0
        else:
            if pinching == cand:
                count += 1
            else:
                cand, count = pinching, 1
            if count >= confirm:
                stable, count = pinching, 0
        flags.append(stable)
    return np.array(flags)


def auc(pos, neg):
    s = np.concatenate([-np.asarray(pos), -np.asarray(neg)])
    order = np.argsort(s)
    ranks = np.empty(len(s))
    ranks[order] = np.arange(1, len(s) + 1)
    return (ranks[:len(pos)].sum() - len(pos) * (len(pos) + 1) / 2) / (len(pos) * len(neg))


d = sys.argv[1]
RULE_ENTER = float(sys.argv[2]) if len(sys.argv) > 2 else 0.14
RULE_EXIT = float(sys.argv[3]) if len(sys.argv) > 3 else 0.20
meta, res, last_t = load(d)
print(f"{d}: {len(res)} hand results, {last_t:.0f} s of the prompt timeline")
print("median r3 / r2 per 2 s:")
for a in np.arange(0, last_t, 2.0):
    v = [x for x in res if a <= x["t"] < a + 2.0]
    if v:
        print(f"  {a:4.0f}s r3 {np.median([x['r3'] for x in v]):.2f} r2 {np.median([x['r2'] for x in v]):.2f} n={len(v)}")

windows = {}
print(f"\n{'pose':12s} {'cls':5s}  " + "   ".join(f"{f:^22s}" for f in FEATURES) + "   facing  depth  gestures")
for ph in meta["phases"]:
    if ph["end"] > last_t + 0.5:
        continue
    w = [x for x in res if ph["hold"] + 1.0 <= x["t"] < ph["end"]]
    if len(w) < 5:
        print(f"{ph['id']:12s} {ph['cls']:5s}  too few results ({len(w)})")
        continue
    windows[ph["id"]] = (ph, w)
    g = np.bincount([x["gesture"] for x in w], minlength=9)
    gs = " ".join(f"{i}:{100 * c // len(w)}%" for i, c in enumerate(g) if c * 20 >= len(w))
    print(f"{ph['id']:12s} {ph['cls']:5s}  " + "   ".join(stats([x[f] for x in w]) for f in FEATURES)
          + f"   {np.median([x['facing'] for x in w]):.2f}   {np.median([x['depth'] for x in w]):.2f}   {gs}")

pos = [x for k, (ph, w) in windows.items() if ph["cls"] == "pos" for x in w]
neg_hard = [x for k, (ph, w) in windows.items() if ph["id"] == "almost" for x in w]
neg_easy = [x for k, (ph, w) in windows.items() if ph["cls"] == "neg" and ph["id"] != "almost" for x in w]
ask = [x for k, (ph, w) in windows.items() if ph["cls"] == "ask" for x in w]
if not pos or not (neg_easy or neg_hard):
    sys.exit("not enough data for the sweep")
print(f"\nAUC (pinch vs fist/open/thumbs up, then vs the 'almost' pose):")
for f in FEATURES:
    print(f"  {f:5s} {auc([x[f] for x in pos], [x[f] for x in neg_easy]):.3f}   "
          + (f"{auc([x[f] for x in pos], [x[f] for x in neg_hard]):.3f}" if neg_hard else "-"))

for f in FEATURES:
    print(f"\n{f}: threshold sweep, share of results below the threshold")
    print("   thr   pinch   open/fist/thumb   almost   ok_sign")
    lo = 0.05
    for t in np.arange(lo, 0.96, 0.05):
        def below(xs):
            return f"{100 * np.mean([x[f] < t for x in xs]):5.1f}%" if xs else "   -  "
        print(f"  {t:4.2f}  {below(pos)}  {below(neg_easy)}      {below(neg_hard)}  {below(ask)}")

print("\nhysteresis simulation on the stable flag (3 results in a row): detected in pinch poses / false in others / drop-outs in pinch poses / cycle transitions")
best = []
for f in FEATURES:
    for enter in np.arange(0.1, 0.8, 0.05):
        for ratio in (1.25, 1.5, 2.0):
            exit_ = enter * ratio
            det, drops, fals = [], 0, []
            for k, (ph, w) in windows.items():
                flags = simulate([x[f] for x in w], enter, exit_)
                if ph["cls"] == "pos":
                    det.append(flags.mean())
                    drops += int(np.sum(flags[:-1] & ~flags[1:]))
                elif ph["cls"] == "neg":
                    fals.append(flags.mean())
            cyc = windows.get("pinch_cycle")
            trans = int(np.sum(np.diff(simulate([x[f] for x in cyc[1]], enter, exit_).astype(int)) != 0)) if cyc else -1
            best.append((f, enter, exit_, np.mean(det), max(fals) if fals else 0, drops, trans))
print("  feature enter exit  pinch-detect  worst-false  dropouts  cycle-transitions(ideal 6)")
for b in sorted(best, key=lambda b: (-(b[3] - 3 * b[4] - 0.01 * b[5]),))[:25]:
    print(f"  {b[0]:6s} {b[1]:5.2f} {b[2]:5.2f}   {100 * b[3]:5.1f}%       {100 * b[4]:5.1f}%     {b[5]:3d}       {b[6]}")

print(f"\nrule rxy enter {RULE_ENTER} exit {RULE_EXIT}: share of each pose reported as pinch")
for k, (ph, w) in windows.items():
    flags = simulate([x["rxy"] for x in w], RULE_ENTER, RULE_EXIT)
    print(f"  {k:12s} {ph['cls']:5s} {100 * flags.mean():5.1f}%   rxy median {np.median([x['rxy'] for x in w]):.2f}")
cyc = windows.get("pinch_cycle")
if cyc:
    ts = np.array([x["t"] for x in cyc[1]])
    fl = simulate([x["rxy"] for x in cyc[1]], RULE_ENTER, RULE_EXIT)
    print("  pinch_cycle state changes at t =", [round(float(ts[i + 1]), 1) for i in np.nonzero(np.diff(fl.astype(int)))[0]])
