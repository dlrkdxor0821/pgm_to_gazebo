#!/usr/bin/env python3
"""PGM (점유격자) → self-contained Gazebo world.sdf 변환기.

map.pgm + map.yaml 한 쌍을 받아 같은 좌표계의 world.sdf 를 생성한다.
알고리즘: occupied 픽셀 → row-strip → vertical-merge → 직사각형 분해 → box.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

WALL_HEIGHT = 0.72
WALL_Z = WALL_HEIGHT / 2.0
WALL_AMBIENT = "0.96 0.93 0.85 1"
WALL_DIFFUSE = "0.98 0.95 0.88 1"
FLOOR_AMBIENT = "0.78 0.76 0.72 1"
FLOOR_DIFFUSE = "0.82 0.80 0.76 1"


def read_pgm(path: Path) -> np.ndarray:
    """P5 (binary) PGM 만 지원. ROS map_server 가 저장하는 형식."""
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"{path}: P5 PGM 아님 (magic={magic!r})")
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        int(f.readline().strip())  # maxval, 사용 안 함
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    return data


def runs_in_row(row: np.ndarray) -> set[tuple[int, int]]:
    runs: set[tuple[int, int]] = set()
    c = 0
    n = len(row)
    while c < n:
        if row[c]:
            c0 = c
            while c < n and row[c]:
                c += 1
            runs.add((c0, c))
        else:
            c += 1
    return runs


def decompose_to_rects(occ_mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    """occupied mask → (r0, c0, r1, c1) 직사각형 리스트 (반열린 구간).

    행별 horizontal run 을 vertical merge — 같은 (c0, c1) run 이 연속 행에
    걸쳐 있으면 하나의 직사각형으로 합침.
    """
    H = occ_mask.shape[0]
    rects: list[tuple[int, int, int, int]] = []
    open_rect: dict[tuple[int, int], int] = {}
    for r in range(H):
        cur = runs_in_row(occ_mask[r])
        closed = [k for k in open_rect if k not in cur]
        for k in closed:
            rects.append((open_rect[k], k[0], r, k[1]))
            del open_rect[k]
        for run in cur:
            if run not in open_rect:
                open_rect[run] = r
    for run, r0 in open_rect.items():
        rects.append((r0, run[0], H, run[1]))
    return rects


def pix_rect_to_world(
    r0: int, c0: int, r1: int, c1: int,
    H: int, res: float, ox: float, oy: float,
) -> tuple[float, float, float, float]:
    """PGM 픽셀 직사각형 → world (cx, cy, sx, sy). y 축은 image row 반전."""
    cx_r = (c0 + c1) / 2 * res
    cy_r = (H - (r0 + r1) / 2) * res
    sx = (c1 - c0) * res
    sy = (r1 - r0) * res
    return cx_r + ox, cy_r + oy, sx, sy


def wall_material() -> str:
    # 나중에 PBR 텍스처로 바꾸려면 이 블록만 교체하면 됨.
    return f"<material><ambient>{WALL_AMBIENT}</ambient><diffuse>{WALL_DIFFUSE}</diffuse></material>"


def floor_material() -> str:
    return f"<material><ambient>{FLOOR_AMBIENT}</ambient><diffuse>{FLOOR_DIFFUSE}</diffuse></material>"


def build_world_sdf(
    rects_world: list[tuple[float, float, float, float]], world_name: str
) -> str:
    """직사각형 리스트 → self-contained world.sdf 문자열."""
    lines: list[str] = [
        '<?xml version="1.0" ?>',
        '<sdf version="1.8">',
        f'  <world name="{world_name}">',
        '    <physics name="1ms" type="ignored">',
        "      <max_step_size>0.001</max_step_size>",
        "      <real_time_factor>1.0</real_time_factor>",
        "    </physics>",
        "",
        '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
        '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">',
        "      <render_engine>ogre2</render_engine>",
        "    </plugin>",
        '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
        '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
        '    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>',
        "",
        '    <light type="directional" name="sun">',
        "      <cast_shadows>true</cast_shadows>",
        "      <pose>10 10 12 0 0 0</pose>",
        "      <diffuse>0.9 0.9 0.9 1</diffuse>",
        "      <specular>0.6 0.6 0.6 1</specular>",
        "      <attenuation>",
        "        <range>1000</range>",
        "        <constant>0.9</constant>",
        "        <linear>0.01</linear>",
        "        <quadratic>0.001</quadratic>",
        "      </attenuation>",
        "      <direction>-0.5 -0.3 -0.9</direction>",
        "    </light>",
        "",
        '    <model name="ground_plane">',
        "      <static>true</static>",
        '      <link name="link">',
        '        <collision name="collision">',
        "          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>",
        "        </collision>",
        '        <visual name="visual">',
        "          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>",
        f"          {floor_material()}",
        "        </visual>",
        "      </link>",
        "    </model>",
        "",
        f'    <model name="{world_name}_env">',
        "      <static>true</static>",
        '      <link name="link">',
        "        <pose>0 0 0 0 0 0</pose>",
    ]

    for idx, (cx, cy, sx, sy) in enumerate(rects_world):
        name = f"wall_{idx:04d}"
        lines += [
            f'        <collision name="{name}">',
            f"          <pose>{cx:.4f} {cy:.4f} {WALL_Z:.4f} 0 0 0</pose>",
            f"          <geometry><box><size>{sx:.4f} {sy:.4f} {WALL_HEIGHT:.4f}</size></box></geometry>",
            "        </collision>",
            f'        <visual name="{name}_v">',
            f"          <pose>{cx:.4f} {cy:.4f} {WALL_Z:.4f} 0 0 0</pose>",
            f"          <geometry><box><size>{sx:.4f} {sy:.4f} {WALL_HEIGHT:.4f}</size></box></geometry>",
            f"          {wall_material()}",
            "        </visual>",
        ]

    lines += [
        "      </link>",
        "    </model>",
        "  </world>",
        "</sdf>",
        "",
    ]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="PGM → self-contained Gazebo world.sdf")
    p.add_argument("--map", required=True, type=Path, help="map.yaml 경로")
    p.add_argument("--out", required=True, type=Path, help="출력 world.sdf 경로")
    p.add_argument("--name", default=None, help="world name (기본: 출력 파일 stem)")
    args = p.parse_args(argv)

    map_yaml = args.map.resolve()
    if not map_yaml.is_file():
        print(f"[error] map yaml 없음: {map_yaml}", file=sys.stderr)
        return 1

    out_path = args.out.resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    world_name = args.name or out_path.stem

    y = yaml.safe_load(map_yaml.read_text())
    res = float(y["resolution"])
    ox, oy = float(y["origin"][0]), float(y["origin"][1])
    free_thresh = float(y.get("free_thresh", 0.196))

    pgm_path = (map_yaml.parent / y["image"]).resolve()
    if not pgm_path.is_file():
        print(f"[error] PGM 없음: {pgm_path}", file=sys.stderr)
        return 1

    img = read_pgm(pgm_path)
    H, W = img.shape
    occ_mask = img < int(free_thresh * 255)
    print(f"[input]  {pgm_path}")
    print(f"         {W}x{H} px @ {res} m/px  origin=({ox:.3f}, {oy:.3f})")
    print(f"         occupied: {int(occ_mask.sum())} px ({100 * occ_mask.mean():.2f}%)")

    rects = decompose_to_rects(occ_mask)
    rects_world: list[tuple[float, float, float, float]] = []
    for r0, c0, r1, c1 in rects:
        cx, cy, sx, sy = pix_rect_to_world(r0, c0, r1, c1, H, res, ox, oy)
        if sx < res or sy < res:
            continue
        rects_world.append((cx, cy, sx, sy))

    out_path.write_text(build_world_sdf(rects_world, world_name))
    print(f"[output] {out_path}")
    print(f"         walls: {len(rects_world)} boxes  (world name = {world_name})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
