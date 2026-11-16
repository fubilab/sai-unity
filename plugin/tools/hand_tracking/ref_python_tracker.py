"""Run the original geaxgx Python HandTracker (host mode) over frames recorded by the sai-unity harness.

usage: ref_python_tracker.py REC_DIR OUT_JSONL [--lm full|lite] [--solo]
The recorded frames are re-encoded to a lossless AVI first (cached as REC_DIR/color_ffv1.avi).
Output rows: {"seq": recorded frame sequence, "hands": [{"landmarks": 63 normalized floats, ...}]}
"""
import argparse
import json
import os
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# checkout of https://github.com/geaxgx/depthai_hand_tracker (also in depthai-unity/unity_bridge/depthai_hand_tracker)
HT_DIR = Path(os.environ.get("DEPTHAI_HAND_TRACKER_DIR", r"C:\Fran\repos\depthai-unity\unity_bridge\depthai_hand_tracker"))
sys.path.insert(0, str(HT_DIR))
from HandTracker import HandTracker  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("rec")
ap.add_argument("out")
ap.add_argument("--lm", default="full")
ap.add_argument("--solo", action="store_true")
ap.add_argument("--max_frames", type=int, default=0)
args = ap.parse_args()
rec = Path(args.rec)

HDR = struct.Struct("<qddiiii")
avi = rec / "color_ffv1.avi"
seqs_path = rec / "color_seqs.json"
if not avi.exists():
    seqs = []
    writer = None
    with open(rec / "color.raw", "rb") as f:
        f.seek(0, 2)
        size = f.tell()
        f.seek(0)
        while f.tell() + HDR.size <= size:
            seq, dts, host, w, h, n, _ = HDR.unpack(f.read(HDR.size))
            if f.tell() + n > size:
                break
            buf = np.frombuffer(f.read(n), np.uint8).reshape(3, h, w)
            bgr = np.ascontiguousarray(np.stack([buf[2], buf[1], buf[0]], -1))
            if writer is None:
                writer = cv2.VideoWriter(str(avi), cv2.VideoWriter_fourcc(*"FFV1"), 30, (w, h))
                if not writer.isOpened():
                    raise SystemExit("FFV1 writer unavailable")
            writer.write(bgr)
            seqs.append(int(seq))
    writer.release()
    seqs_path.write_text(json.dumps(seqs))
seqs = json.loads(seqs_path.read_text())

tracker = HandTracker(
    input_src=str(avi),
    use_lm=True,
    lm_model=args.lm,
    use_world_landmarks=True,
    solo=args.solo,
    use_gesture=True,
    stats=True,
)
w, h = tracker.img_w, tracker.img_h
print("frame size", w, h, "pad_h", tracker.pad_h)

rows = []
t_start = time.time()
i = 0
while True:
    frame, hands, bag = tracker.next_frame()
    if frame is None:
        break
    row = {"seq": seqs[i], "hands": []}
    for hd in hands:
        lm = np.array(hd.landmarks, dtype=float)  # (21, 2) pixels in the original (unpadded) frame
        z = np.array(hd.norm_landmarks, dtype=float)[:, 2]  # relative depth, crop-normalized
        norm = np.stack([lm[:, 0] / w, lm[:, 1] / h, z], 1).reshape(-1)
        row["hands"].append(dict(
            landmarks=norm.tolist(),
            lm_score=float(hd.lm_score),
            handedness=float(hd.handedness),
            label=hd.label,
            gesture=hd.gesture,
            world=np.array(hd.world_landmarks).reshape(-1).tolist() if getattr(hd, "world_landmarks", None) is not None else None,
        ))
    rows.append(row)
    i += 1
    if args.max_frames and i >= args.max_frames:
        break
elapsed = time.time() - t_start
tracker.exit()
with open(args.out, "w") as f:
    for r in rows:
        f.write(json.dumps(r) + "\n")
print(f"processed {i} frames in {elapsed:.1f}s ({i / elapsed:.1f} fps) -> {args.out}")
