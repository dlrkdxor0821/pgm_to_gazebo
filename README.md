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
