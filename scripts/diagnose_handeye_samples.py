#!/usr/bin/env python3
"""离线检查手眼样本的坐标链和单样本敏感性。

该脚本不连接机器人，只读取 sample_*.json 和其中的角点。
眼在手外假定 robot_pose 是 ^B T_G，传给 OpenCV 前取逆，
solvePnP 输出 ^C T_T，最终结果解释为 ^B T_C。
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def euler_abc(a, b, c):
    a, b, c = np.radians([a, b, c])
    ca, sa = np.cos(a), np.sin(a)
    cb, sb = np.cos(b), np.sin(b)
    cc, sc = np.cos(c), np.sin(c)
    rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    rz = np.array([[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]])
    return rz @ ry @ rx


def make_t(r, t):
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = r, np.asarray(t).reshape(3)
    return out


def inv_t(x):
    out = np.eye(4)
    out[:3, :3] = x[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ x[:3, 3]
    return out


def load(directory):
    files = sorted(Path(directory).glob("sample_*.json"))
    if len(files) < 3:
        raise SystemExit(f"需要至少 3 个样本，实际 {len(files)}")
    payloads = [json.loads(f.read_text(encoding="utf-8")) for f in files]
    latest = next((p for p in payloads if p.get("camera_matrix") is not None), None)
    if latest is None:
        raise SystemExit("样本中没有 camera_matrix")
    k = np.asarray(latest["camera_matrix"], float)
    d = np.asarray(latest.get("dist_coeffs", [0] * 5), float).reshape(-1, 1)
    obj = np.zeros((9 * 6, 3), np.float32)
    obj[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
    obj *= 24.0
    rows = []
    for p in payloads:
        corners = np.asarray(p["corners"], np.float32).reshape(-1, 1, 2)
        ok, rv, tv = cv2.solvePnP(obj, corners, k, d)
        if not ok:
            raise SystemExit(f"solvePnP 失败: {p.get('index')}")
        r = cv2.Rodrigues(rv)[0]
        pose = p["robot_pose_xyzabc_mm_deg"]
        bg = make_t(euler_abc(*pose[3:]), pose[:3])
        rows.append((p, bg, make_t(r, tv)))
    return rows, k, d, obj


def solve(rows):
    aa = [inv_t(bg) for _, bg, _ in rows]  # ^G T_B
    bb = [ct for _, _, ct in rows]        # ^C T_T
    r, t = cv2.calibrateHandEye(
        [x[:3, :3] for x in aa], [x[:3, 3, None] for x in aa],
        [x[:3, :3] for x in bb], [x[:3, 3, None] for x in bb],
        method=cv2.CALIB_HAND_EYE_PARK)
    return make_t(r, t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("directory", type=Path)
    args = ap.parse_args()
    rows, _, _, _ = load(args.directory)
    result = solve(rows)
    inferred = [inv_t(bg) @ result @ ct for _, bg, ct in rows]
    mean_t = np.mean([x[:3, 3] for x in inferred], axis=0)
    print(f"目录: {args.directory}  样本数: {len(rows)}")
    print("结果 ^B T_C 平移(mm):", np.round(result[:3, 3], 3))
    print("每条样本反推 ^G T_T 平移及偏差(mm):")
    for (p, _, _), x in zip(rows, inferred):
        print(f"  {int(p['index']):3d}: {np.round(x[:3,3],3)}  |d|={np.linalg.norm(x[:3,3]-mean_t):.3f}")
    print("机器人相邻位姿平移变化(mm):", [round(float(np.linalg.norm(rows[i][1][:3,3]-rows[i-1][1][:3,3])), 2) for i in range(1,len(rows))])
    print("留一法 ^B T_C 平移(mm):")
    for i in range(len(rows)):
        sub = rows[:i] + rows[i+1:]
        x = solve(sub)
        print(f"  去掉样本 {rows[i][0]['index']}: {np.round(x[:3,3],3)}")


if __name__ == "__main__":
    main()
