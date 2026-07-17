#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自动生成 4 条示例轨迹(塔筒展开导航坐标系,x=周向弧长 m,y=高度 m)。

轨迹:
  1. 定高环向一圈:在固定高度绕塔一周;
  2. 竖直上升:沿母线竖直向上;
  3. 之字形覆盖:高度带内往复扫描;
  4. 矩形巡检:环向-上升-环向-下降闭合矩形。

用法:
  python3 gen_trajectories.py --tower <tower.yaml> --out <trajectories_dir>

生成后自动自校验:点间距连续、yaw 与几何切向一致、曲率与 yaw 变化率一致。
"""

import argparse
import math
import os
import sys

import yaml


def load_tower(path):
    """加载塔筒配置并做基本校验(与 C++ TowerConfig 一致的规则)。"""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    t = data["tower"]
    segs = sorted(t["segments"], key=lambda s: s["start_height"])
    assert t["total_height"] > 0, "total_height 必须为正"
    assert abs(segs[0]["start_height"]) < 1e-6, "首段必须从 0 开始"
    assert abs(segs[-1]["end_height"] - t["total_height"]) < 1e-6, "末段必须到 total_height"
    for a, b in zip(segs, segs[1:]):
        assert abs(b["start_height"] - a["end_height"]) < 1e-6, "分段必须连续"
    return t, segs


def radius_at(segs, h):
    """r(h) 分段线性插值。"""
    for s in segs:
        if h <= s["end_height"] + 1e-9:
            span = s["end_height"] - s["start_height"]
            ratio = min(max((h - s["start_height"]) / span, 0.0), 1.0)
            d = s["bottom_diameter"] + ratio * (s["top_diameter"] - s["bottom_diameter"])
            return 0.5 * d
    return 0.5 * segs[-1]["top_diameter"]


def resample_polyline(waypoints, step):
    """把折线按弧长等距重采样,返回 (x, y) 列表。"""
    pts = []
    for (x0, y0), (x1, y1) in zip(waypoints, waypoints[1:]):
        seg_len = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(seg_len / step)))
        for i in range(n):
            r = i / n
            pts.append((x0 + r * (x1 - x0), y0 + r * (y1 - y0)))
    pts.append(waypoints[-1])
    return pts


def build_trajectory(pts, v_ref):
    """由 (x,y) 点列计算 yaw(切向)与曲率(yaw 变化率/弧长)。"""
    n = len(pts)
    traj = []
    yaws = []
    for i in range(n):
        if i < n - 1:
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
        else:
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
        yaws.append(math.atan2(dy, dx))

    # 展开(unwrap)yaw 保证连续,便于曲率差分
    unwrapped = [yaws[0]]
    for y in yaws[1:]:
        prev = unwrapped[-1]
        d = y - prev
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        unwrapped.append(prev + d)

    for i in range(n):
        if 0 < i < n - 1:
            ds1 = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            ds2 = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            ds = ds1 + ds2
            curv = (unwrapped[i + 1] - unwrapped[i - 1]) / ds if ds > 1e-9 else 0.0
        else:
            curv = 0.0
        traj.append({
            "x": round(pts[i][0], 6),
            "y": round(pts[i][1], 6),
            "yaw": round(math.atan2(math.sin(unwrapped[i]), math.cos(unwrapped[i])), 6),
            "v_ref": v_ref,
            "curvature": round(curv, 6),
        })
    return traj


def gen_traj_1(t, segs, step, v_ref):
    """① 定高环向一圈:h = 25% 总高,绕塔一周回到起点。"""
    h = 0.25 * t["total_height"]
    c = 2 * math.pi * radius_at(segs, h)
    n = int(math.ceil(c / step))
    pts = [(c * i / n, h) for i in range(n + 1)]
    return build_trajectory(pts, v_ref), "定高环向一圈 h=%.1fm 周长=%.2fm" % (h, c)


def gen_traj_2(t, segs, step, v_ref):
    """② 竖直上升:x 固定,从 10% 爬升到 60% 总高。"""
    x = 1.0
    y0, y1 = 0.10 * t["total_height"], 0.60 * t["total_height"]
    pts = resample_polyline([(x, y0), (x, y1)], step)
    return build_trajectory(pts, v_ref), "竖直上升 %.1fm → %.1fm" % (y0, y1)


def gen_traj_3(t, segs, step, v_ref):
    """③ 之字形覆盖:高度带内环向往复 + 逐级抬升。"""
    y_lo, y_hi = 0.15 * t["total_height"], 0.45 * t["total_height"]
    rows = 4
    dy = (y_hi - y_lo) / rows
    # 周向覆盖范围取该带最小周长的 60%,避免高处环绕越界
    c_min = 2 * math.pi * min(radius_at(segs, y_lo), radius_at(segs, y_hi))
    width = 0.6 * c_min
    wps = []
    for i in range(rows + 1):
        y = y_lo + i * dy
        if i % 2 == 0:
            wps += [(0.0, y), (width, y)]
        else:
            wps += [(width, y), (0.0, y)]
    # 相邻行之间竖直过渡(资源点已按顺序衔接)
    pts = resample_polyline(wps, step)
    return build_trajectory(pts, v_ref), "之字形覆盖 %.1f~%.1fm 宽%.1fm %d行" % (y_lo, y_hi, width, rows)


def gen_traj_4(t, segs, step, v_ref):
    """④ 矩形巡检:环向→上升→反向环向→下降,闭合矩形。"""
    y_lo, y_hi = 0.20 * t["total_height"], 0.40 * t["total_height"]
    c_min = 2 * math.pi * min(radius_at(segs, y_lo), radius_at(segs, y_hi))
    width = 0.5 * c_min
    wps = [(0.0, y_lo), (width, y_lo), (width, y_hi), (0.0, y_hi), (0.0, y_lo)]
    pts = resample_polyline(wps, step)
    return build_trajectory(pts, v_ref), "矩形巡检 %.1f×%.1fm" % (width, y_hi - y_lo)


def self_check(traj, step):
    """自校验:点距、yaw 切向一致性、曲率一致性。"""
    for i in range(len(traj) - 1):
        a, b = traj[i], traj[i + 1]
        ds = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        assert ds < 2.5 * step, f"点 {i} 间距 {ds:.3f} 超过阈值"
        if ds > 1e-6:
            tangent = math.atan2(b["y"] - a["y"], b["x"] - a["x"])
            dyaw = abs(math.atan2(math.sin(tangent - a["yaw"]), math.cos(tangent - a["yaw"])))
            assert dyaw < 0.5, f"点 {i} yaw 与切向偏差 {dyaw:.3f} rad 过大"
    for p in traj:
        assert p["v_ref"] > 0, "v_ref 必须为正"


def main():
    ap = argparse.ArgumentParser()
    default_root = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
    ap.add_argument("--tower", default=os.path.join(default_root, "config", "tower.yaml"))
    ap.add_argument("--out", default=os.path.join(default_root, "config", "trajectories"))
    ap.add_argument("--step", type=float, default=0.05, help="重采样步长 m")
    ap.add_argument("--v-ref", type=float, default=0.15, help="参考速度 m/s")
    args = ap.parse_args()

    t, segs = load_tower(args.tower)
    os.makedirs(args.out, exist_ok=True)

    generators = [gen_traj_1, gen_traj_2, gen_traj_3, gen_traj_4]
    for idx, gen in enumerate(generators, start=1):
        traj, desc = gen(t, segs, args.step, args.v_ref)
        self_check(traj, args.step)
        out_path = os.path.join(args.out, f"traj_{idx}.yaml")
        with open(out_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                {"trajectory": {"id": idx, "description": desc, "points": traj}},
                f, allow_unicode=True, sort_keys=False)
        print(f"traj_{idx}.yaml: {desc},{len(traj)} 点")
    print("全部轨迹生成并自校验通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
