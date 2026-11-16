"""Render a contact sheet: recorded frames with hand landmarks from a *.jsonl overlaid.

usage: render_overlay.py REC_DIR JSONL OUT_PNG [times_s,...]
JSONL rows: {"seq": frame sequence, "hands": [{"landmarks": [63 floats normalized x,y,z], ...}]}
"""
import json
import struct
import sys
from pathlib import Path

import cv2
import numpy as np

rec = Path(sys.argv[1])
jsonl = Path(sys.argv[2])
out_png = sys.argv[3]
times = [float(t) for t in sys.argv[4].split(",")] if len(sys.argv) > 4 else [3, 8, 11, 14, 18, 22, 26, 30, 34, 38, 42, 46]

HDR = struct.Struct("<qddiiii")
frames = []
with open(rec / "color.raw", "rb") as f:
    f.seek(0, 2)
    size = f.tell()
    f.seek(0)
    while f.tell() + HDR.size <= size:
        seq, dts, host, w, h, n, _ = HDR.unpack(f.read(HDR.size))
        frames.append((seq, host, w, h, n, f.tell()))
        f.seek(n, 1)
by_seq = {fr[0]: fr for fr in frames}
t0 = frames[0][1]

rows = [json.loads(l) for l in open(jsonl) if l.strip()]
by_hseq = {r["seq"]: r for r in rows}

CONN = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
        (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]
COL = [(0, 255, 0), (0, 165, 255), (255, 0, 255)]


def load(fr):
    seq, host, w, h, n, off = fr
    with open(rec / "color.raw", "rb") as f:
        f.seek(off)
        buf = np.frombuffer(f.read(n), np.uint8).reshape(3, h, w)  # planar RGB
    return np.ascontiguousarray(np.stack([buf[2], buf[1], buf[0]], -1))


tiles = []
for t in times:
    target = t0 + t
    # frame whose host time is closest, that also has a hand-output row
    cands = [fr for fr in frames if fr[0] in by_hseq]
    fr = min(cands, key=lambda fr: abs(fr[1] - target))
    img = load(fr)
    r = by_hseq[fr[0]]
    for i, hd in enumerate(r["hands"]):
        lm = np.array(hd["landmarks"]).reshape(21, 3)
        pts = [(int(x * img.shape[1]), int(y * img.shape[0])) for x, y, _ in lm]
        for a, b in CONN:
            cv2.line(img, pts[a], pts[b], COL[i % 3], 2)
        for p in pts:
            cv2.circle(img, p, 3, (0, 0, 255), -1)
        cv2.putText(img, f"{'R' if hd.get('handedness', .5) > .5 else 'L'} lm={hd.get('lm_score', 0):.2f}",
                    (pts[0][0] + 5, pts[0][1] + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL[i % 3], 1)
    cv2.putText(img, f"t={fr[1] - t0:.1f}s seq={fr[0]} hands={len(r['hands'])}", (6, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    tiles.append(cv2.resize(img, (426, 240)))

cols = 3
while len(tiles) % cols:
    tiles.append(np.zeros_like(tiles[0]))
sheet = np.vstack([np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)])
cv2.imwrite(out_png, sheet)
print(out_png, sheet.shape)
