# PGM → Gazebo 변환 도구 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** ROS 점유격자 지도(map.pgm + map.yaml)를 골라 Gazebo에서 바로 띄울 수 있는 self-contained world.sdf로 바꾸는 대화형 CLI 도구를 만든다.

**Architecture:** `scripts/pgm-to-gazebo.sh`가 대화형 UI(목록·번호선택·이름입력)를 담당하고, 실제 변환은 `scripts/pgm_to_world.py`(numpy + pyyaml)가 수행한다. 변환 로직은 pingdergarten `make_candidate_world.py`에서 이식. 입력은 `pgm/`, 출력은 `world/` 폴더이며 둘 다 내용물은 gitignore한다.

**Tech Stack:** Python 3.12, numpy 1.26, pyyaml, pytest 7.4, bash.

## Global Constraints

- P5(binary) PGM만 지원. magic이 `P5`가 아니면 ValueError.
- 벽 높이 0.72m, z중심 0.36m.
- 벽 색: ambient `0.96 0.93 0.85 1`, diffuse `0.98 0.95 0.88 1`.
- 바닥 색: ambient `0.78 0.76 0.72 1`, diffuse `0.82 0.80 0.76 1`.
- `free_thresh` 기본값 0.196. 점유 판정: `img < int(free_thresh * 255)`.
- 픽셀→world: y축은 image row 반전, origin 오프셋 적용. `res`보다 작은 변은 스킵.
- material 생성은 별도 헬퍼 함수로 분리(나중 PBR 텍스처 확장 대비).
- 커밋 메시지 끝에 `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>` 추가.

---

## File Structure

- `scripts/pgm_to_world.py` — 변환 엔진 (순수 함수 + CLI main).
- `tests/test_pgm_to_world.py` — 엔진 단위/통합 테스트.
- `scripts/pgm-to-gazebo.sh` — 대화형 셸 래퍼.
- `.gitignore` — pgm/world 내용물 제외.
- `pgm/.gitkeep`, `world/.gitkeep` — 빈 폴더 유지.
- `README.md` — 원리·사용법·텍스처 교체법.

---

### Task 1: 프로젝트 스캐폴딩 (.gitignore + 폴더 구조)

**Files:**
- Create: `.gitignore`, `pgm/.gitkeep`, `world/.gitkeep`

**Interfaces:**
- Consumes: 없음
- Produces: `pgm/`·`world/` 디렉토리가 git에 존재. 내용물은 무시되되 `.gitkeep`은 추적됨.

- [ ] **Step 1: 디렉토리와 .gitkeep 생성**

```bash
mkdir -p pgm world
touch pgm/.gitkeep world/.gitkeep
```

- [ ] **Step 2: .gitignore 작성**

Create `.gitignore`:

```gitignore
# pgm/ 와 world/ 내용물은 추적하지 않음 (.gitkeep 은 예외)
pgm/*
world/*
!pgm/.gitkeep
!world/.gitkeep

# python
__pycache__/
*.pyc
.pytest_cache/
```

- [ ] **Step 3: 무시 규칙 검증**

Run: `touch pgm/dummy.pgm world/dummy.sdf && git status --porcelain`
Expected: 출력에 `pgm/dummy.pgm`·`world/dummy.sdf` 없음. `pgm/.gitkeep`·`world/.gitkeep`·`.gitignore`만 `??`로 보임.
그 후: `rm pgm/dummy.pgm world/dummy.sdf`

- [ ] **Step 4: 커밋**

```bash
git add .gitignore pgm/.gitkeep world/.gitkeep
git commit -m "chore: scaffold pgm/world folders and gitignore

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: 변환 엔진 순수 함수 (pgm_to_world.py 코어 + 테스트)

**Files:**
- Create: `scripts/pgm_to_world.py`
- Test: `tests/test_pgm_to_world.py`

**Interfaces:**
- Consumes: 없음
- Produces:
  - `read_pgm(path: Path) -> np.ndarray` — (H, W) uint8 배열. P5 아니면 ValueError.
  - `runs_in_row(row: np.ndarray) -> set[tuple[int,int]]` — 행의 True run을 (c0, c1) 반열린구간 집합으로.
  - `decompose_to_rects(occ_mask: np.ndarray) -> list[tuple[int,int,int,int]]` — (r0, c0, r1, c1) 직사각형 리스트.
  - `pix_rect_to_world(r0, c0, r1, c1, H, res, ox, oy) -> tuple[float,float,float,float]` — (cx, cy, sx, sy).

- [ ] **Step 1: 순수 함수 테스트 작성**

Create `tests/test_pgm_to_world.py`:

```python
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import pgm_to_world as p2w


def test_runs_in_row_finds_contiguous_true_runs():
    row = np.array([0, 1, 1, 0, 1, 0], dtype=bool)
    assert p2w.runs_in_row(row) == {(1, 3), (4, 5)}


def test_runs_in_row_empty():
    assert p2w.runs_in_row(np.zeros(5, dtype=bool)) == set()


def test_decompose_single_rectangle():
    # 2x2 occupied block at rows 1-2, cols 1-2
    mask = np.zeros((4, 4), dtype=bool)
    mask[1:3, 1:3] = True
    rects = p2w.decompose_to_rects(mask)
    # (r0, c0, r1, c1) 반열린: rows[1,3), cols[1,3)
    assert rects == [(1, 1, 3, 3)]


def test_decompose_two_separate_columns_same_rows():
    mask = np.zeros((3, 5), dtype=bool)
    mask[0:3, 0:1] = True
    mask[0:3, 3:4] = True
    rects = sorted(p2w.decompose_to_rects(mask))
    assert rects == [(0, 0, 3, 1), (0, 3, 3, 4)]


def test_pix_rect_to_world_center_and_size():
    # H=10, res=0.05, origin=(-1.0, -2.0)
    # rect rows[0,2) cols[0,2)
    cx, cy, sx, sy = p2w.pix_rect_to_world(0, 0, 2, 2, H=10, res=0.05, ox=-1.0, oy=-2.0)
    assert sx == pytest.approx(0.1)
    assert sy == pytest.approx(0.1)
    # cx_r = (0+2)/2 * 0.05 = 0.05 ; + ox = -0.95
    assert cx == pytest.approx(-0.95)
    # cy_r = (10 - (0+2)/2) * 0.05 = (10-1)*0.05 = 0.45 ; + oy = -1.55
    assert cy == pytest.approx(-1.55)
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `python3 -m pytest tests/test_pgm_to_world.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'pgm_to_world'` (아직 파일 없음).

- [ ] **Step 3: 엔진 코어 구현**

Create `scripts/pgm_to_world.py`:

```python
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
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `python3 -m pytest tests/test_pgm_to_world.py -q`
Expected: PASS (5 passed).

- [ ] **Step 5: 커밋**

```bash
git add scripts/pgm_to_world.py tests/test_pgm_to_world.py
git commit -m "feat: add pgm occupancy decomposition core

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: world.sdf 생성 + CLI main (엔진 완성 + 통합 테스트)

**Files:**
- Modify: `scripts/pgm_to_world.py` (append: material 헬퍼, build_world_sdf, main)
- Test: `tests/test_pgm_to_world.py` (append: sdf/통합 테스트)

**Interfaces:**
- Consumes: Task 2의 `read_pgm`, `decompose_to_rects`, `pix_rect_to_world`.
- Produces:
  - `wall_material() -> str` / `floor_material() -> str` — `<material>...</material>` 블록 문자열.
  - `build_world_sdf(rects_world: list[tuple[float,float,float,float]], world_name: str) -> str` — 완전한 sdf 문자열.
  - `main(argv: list[str] | None = None) -> int` — CLI. `--map`(필수), `--out`(필수), `--name`(선택).

- [ ] **Step 1: sdf/통합 테스트 추가**

Append to `tests/test_pgm_to_world.py`:

```python
import xml.etree.ElementTree as ET


def _write_pgm(path, arr):
    h, w = arr.shape
    with open(path, "wb") as f:
        f.write(b"P5\n")
        f.write(f"{w} {h}\n".encode())
        f.write(b"255\n")
        f.write(arr.astype(np.uint8).tobytes())


def test_build_world_sdf_is_valid_xml_with_walls():
    rects = [(0.0, 0.0, 0.2, 1.0), (1.0, 1.0, 1.0, 0.2)]
    sdf = p2w.build_world_sdf(rects, "demo")
    root = ET.fromstring(sdf)  # raises if malformed
    assert root.tag == "sdf"
    # ground_plane + env 모델 포함, 벽 box 2개
    assert sdf.count("<box>") >= 2
    assert 'name="demo"' in sdf


def test_main_end_to_end(tmp_path):
    # 5x5, 가운데 한 칸만 occupied(검정=0)
    arr = np.full((5, 5), 254, dtype=np.uint8)
    arr[2, 2] = 0
    pgm = tmp_path / "m.pgm"
    _write_pgm(pgm, arr)
    yaml_path = tmp_path / "m.yaml"
    yaml_path.write_text(
        "image: m.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nfree_thresh: 0.196\n"
    )
    out = tmp_path / "out.sdf"
    rc = p2w.main(["--map", str(yaml_path), "--out", str(out)])
    assert rc == 0
    assert out.is_file()
    root = ET.fromstring(out.read_text())
    assert root.tag == "sdf"
    assert "<box>" in out.read_text()
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `python3 -m pytest tests/test_pgm_to_world.py -q`
Expected: FAIL — `AttributeError: module 'pgm_to_world' has no attribute 'build_world_sdf'`.

- [ ] **Step 3: material 헬퍼 + build_world_sdf + main 구현**

Append to `scripts/pgm_to_world.py`:

```python
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
```

- [ ] **Step 4: 전체 테스트 통과 확인**

Run: `python3 -m pytest tests/test_pgm_to_world.py -q`
Expected: PASS (7 passed).

- [ ] **Step 5: 커밋**

```bash
git add scripts/pgm_to_world.py tests/test_pgm_to_world.py
git commit -m "feat: build self-contained world.sdf and CLI entrypoint

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: 대화형 셸 래퍼 (pgm-to-gazebo.sh)

**Files:**
- Create: `scripts/pgm-to-gazebo.sh`

**Interfaces:**
- Consumes: `scripts/pgm_to_world.py`의 `--map/--out/--name` CLI.
- Produces: 실행 가능한 대화형 진입점. `pgm/`의 .pgm을 나열·선택해 `world/<이름>.sdf` 생성.

- [ ] **Step 1: 셸 스크립트 작성**

Create `scripts/pgm-to-gazebo.sh`:

```bash
#!/usr/bin/env bash
# pgm/ 의 점유격자 지도를 골라 world/ 에 self-contained world.sdf 를 생성한다.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PGM_DIR="$REPO_DIR/pgm"
WORLD_DIR="$REPO_DIR/world"

mkdir -p "$WORLD_DIR"

# pgm/ 의 .pgm 목록 수집
shopt -s nullglob
pgms=("$PGM_DIR"/*.pgm)
shopt -u nullglob

if [ ${#pgms[@]} -eq 0 ]; then
    echo "[안내] pgm/ 폴더에 .pgm 파일이 없습니다."
    echo "       map.pgm 과 map.yaml 을 한 쌍으로 pgm/ 에 넣어주세요."
    exit 1
fi

echo "=== pgm/ 의 점유격자 지도 목록 ==="
i=1
for f in "${pgms[@]}"; do
    base="$(basename "$f" .pgm)"
    if [ -f "$PGM_DIR/$base.yaml" ]; then
        printf "  [%d] %s.pgm   (+ %s.yaml ✓)\n" "$i" "$base" "$base"
    else
        printf "  [%d] %s.pgm   (⚠ %s.yaml 없음)\n" "$i" "$base" "$base"
    fi
    i=$((i + 1))
done

# 번호 선택
read -rp "변환할 번호를 선택하세요 [1-${#pgms[@]}]: " choice
if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt ${#pgms[@]} ]; then
    echo "[error] 잘못된 번호입니다: $choice"
    exit 1
fi

selected="${pgms[$((choice - 1))]}"
sel_base="$(basename "$selected" .pgm)"
sel_yaml="$PGM_DIR/$sel_base.yaml"

if [ ! -f "$sel_yaml" ]; then
    echo "[error] 짝 yaml 이 없습니다: $sel_base.yaml"
    echo "        map.pgm 과 같은 이름의 map.yaml 이 필요합니다."
    exit 1
fi

# 출력 이름 입력
read -rp "출력 world 이름 (엔터 시 '$sel_base'): " out_name
out_name="${out_name:-$sel_base}"
out_name="${out_name%.sdf}"  # .sdf 붙여 입력해도 제거
out_path="$WORLD_DIR/$out_name.sdf"

if [ -f "$out_path" ]; then
    read -rp "$out_name.sdf 가 이미 있습니다. 덮어쓸까요? [y/N]: " yn
    case "$yn" in
        [yY]) ;;
        *) echo "취소했습니다."; exit 0 ;;
    esac
fi

# 변환 실행
echo "--- 변환 중 ---"
python3 "$SCRIPT_DIR/pgm_to_world.py" --map "$sel_yaml" --out "$out_path"
echo "--- 완료: $out_path ---"
echo "Gazebo 실행 예: gz sim \"$out_path\""
```

- [ ] **Step 2: 실행 권한 부여**

```bash
chmod +x scripts/pgm-to-gazebo.sh
```

- [ ] **Step 3: 빈 폴더 안내 동작 확인**

Run: `bash scripts/pgm-to-gazebo.sh`
Expected: `pgm/`에 .pgm이 없으므로 "[안내] pgm/ 폴더에 .pgm 파일이 없습니다." 출력 후 종료(exit 1).

- [ ] **Step 4: 합성 지도로 end-to-end 수동 확인**

```bash
python3 - <<'PY'
import numpy as np
from pathlib import Path
arr = np.full((6, 6), 254, dtype=np.uint8)
arr[0, :] = 0; arr[-1, :] = 0; arr[:, 0] = 0; arr[:, -1] = 0  # 테두리 벽
p = Path("pgm/sample.pgm")
with open(p, "wb") as f:
    f.write(b"P5\n6 6\n255\n"); f.write(arr.tobytes())
Path("pgm/sample.yaml").write_text("image: sample.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nfree_thresh: 0.196\n")
print("wrote pgm/sample.{pgm,yaml}")
PY
printf '1\nsample\n' | bash scripts/pgm-to-gazebo.sh
```
Expected: 목록에 `[1] sample.pgm (+ sample.yaml ✓)` 표시, 변환 성공, `world/sample.sdf` 생성. 그 후 `python3 -c "import xml.etree.ElementTree as ET; ET.parse('world/sample.sdf'); print('valid xml')"` → `valid xml`.
정리: `rm -f pgm/sample.pgm pgm/sample.yaml world/sample.sdf`

- [ ] **Step 5: 커밋**

```bash
git add scripts/pgm-to-gazebo.sh
git commit -m "feat: add interactive pgm-to-gazebo shell wrapper

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: README (변환 원리 + 사용법 + 텍스처 교체법)

**Files:**
- Modify: `README.md` (전체 교체)

**Interfaces:**
- Consumes: Task 1-4의 폴더 구조·스크립트·CLI.
- Produces: 없음 (문서).

- [ ] **Step 1: README 작성**

Replace `README.md` with:

````markdown
# pgm_to_gazebo

ROS 점유격자 지도(`map.pgm` + `map.yaml`)를 Gazebo에서 바로 띄울 수 있는
self-contained `world.sdf`로 바꿔주는 대화형 도구.

## 빠른 시작

```bash
# 1) pgm/ 에 map.pgm 과 map.yaml 을 한 쌍으로 넣는다
cp /path/to/office.pgm  pgm/
cp /path/to/office.yaml pgm/

# 2) 대화형 변환 실행
./scripts/pgm-to-gazebo.sh
#   → pgm/ 목록에서 번호 선택 → 출력 이름 입력 → world/<이름>.sdf 생성

# 3) Gazebo 로 확인
gz sim world/office.sdf
```

## 폴더 구조

```
pgm/      입력: map.pgm + map.yaml 쌍   (내용물은 git 추적 안 함)
world/    출력: 생성된 world.sdf        (내용물은 git 추적 안 함)
scripts/  pgm-to-gazebo.sh (대화형) + pgm_to_world.py (변환 엔진)
```

## 변환 원리

### 1. 점유격자 지도란?

ROS의 `map_server`가 저장하는 지도는 두 파일로 이뤄진다.

- **`map.pgm`** — 흑백 이미지. 각 픽셀이 공간 한 칸의 상태를 나타낸다.
  검정(0)에 가까울수록 **점유(벽/장애물)**, 흰색(254)에 가까울수록 **빈 공간**,
  회색(205)은 **미탐색**.
- **`map.yaml`** — 이미지에 실제 크기·위치를 부여하는 메타데이터.
  - `resolution`: 픽셀 한 칸이 몇 미터인지 (예: 0.05 → 5cm).
  - `origin`: 이미지 좌하단이 world 좌표 어디에 놓이는지 `[x, y, theta]`.
  - `free_thresh`: 점유 판정 임계값.

### 2. 점유 픽셀 → 직사각형 묶기

픽셀 하나하나를 박스로 만들면 수만 개가 되어 무겁다. 그래서 붙어 있는
점유 픽셀을 큰 직사각형으로 묶는다.

1. **행별 run**: 각 가로줄에서 연속된 점유 픽셀 구간(run)을 찾는다.
2. **세로 병합(vertical merge)**: 위아래로 같은 열 구간이 이어지면 하나의
   직사각형으로 합친다. 곧은 벽 하나가 박스 하나가 된다.

### 3. 픽셀 ↔ 미터 좌표 변환

직사각형의 픽셀 좌표를 world(미터) 좌표로 옮긴다.

- 크기: `픽셀 개수 × resolution`.
- 중심 x: `중심 픽셀열 × resolution + origin_x`.
- 중심 y: 이미지의 행은 위에서 아래로 증가하지만 Gazebo의 y축은
  아래에서 위로 증가하므로 **행을 뒤집어** `(H − 중심행) × resolution + origin_y`.

### 4. SDF box 생성

각 직사각형을 높이 0.72m의 box(collision + visual)로 만들고, 여기에
물리엔진 설정·태양 조명·바닥(ground_plane)을 더해 **하나의 완결된 world.sdf**로
출력한다. 별도 리소스 경로 설정 없이 `gz sim world/<이름>.sdf`로 바로 열린다.

## 벽/바닥에 PNG 텍스처 입히기

기본 출력은 단색(크림색 벽 / 회색 바닥)이다. 나중에 사진/패턴을 입히려면
`scripts/pgm_to_world.py`의 `wall_material()` / `floor_material()` 함수가
반환하는 `<material>` 블록을 PBR 텍스처 형태로 바꾸면 된다.

```python
def floor_material() -> str:
    return """<material>
      <diffuse>1 1 1 1</diffuse>
      <pbr><metal>
        <albedo_map>textures/floor.png</albedo_map>
        <roughness>1.0</roughness>
      </metal></pbr>
    </material>"""
```

PNG는 `world.sdf` 옆 `world/textures/`에 두고 상대경로로 참조한다.
이 한 함수만 고치면 생성되는 모든 world에 동일하게 적용된다.

## 요구사항

- Python 3, `numpy`, `pyyaml`
- 입력 PGM은 **P5(binary)** 형식 (ROS map_server 기본 저장 형식)
````

- [ ] **Step 2: 커밋**

```bash
git add README.md
git commit -m "docs: explain pgm-to-gazebo conversion principle and usage

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Self-Review

**Spec coverage:**
- 폴더 구조(pgm/world/scripts) → Task 1 ✓
- pgm/world 내용물 gitignore + 폴더 유지 → Task 1 ✓
- 변환 엔진(직사각형 분해, 좌표 변환, world.sdf) → Task 2, 3 ✓
- 대화형 셸(목록·번호·이름·덮어쓰기·world/ 저장) → Task 4 ✓
- self-contained world.sdf 출력 → Task 3 (build_world_sdf) ✓
- 텍스처 구조만 열어두기(material 헬퍼 분리) → Task 3 + README Task 5 ✓
- README 원리 설명 → Task 5 ✓
- 에러 처리(.pgm 없음/yaml 없음/잘못된 번호/덮어쓰기/P5 아님) → Task 3, 4 ✓
- 테스트(순수 함수 + end-to-end) → Task 2, 3 ✓

**Placeholder scan:** 모든 코드 스텝에 실제 코드 포함. TBD/TODO 없음.

**Type consistency:** `read_pgm`, `runs_in_row`, `decompose_to_rects`, `pix_rect_to_world`(Task 2)와 `wall_material`/`floor_material`/`build_world_sdf`/`main`(Task 3) 시그니처가 테스트·호출부와 일치. 셸의 `--map/--out/--name`이 `main`의 argparse와 일치.
