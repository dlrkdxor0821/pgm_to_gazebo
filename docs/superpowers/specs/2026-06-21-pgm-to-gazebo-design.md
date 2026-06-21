# PGM → Gazebo 변환 도구 설계

날짜: 2026-06-21

## 목적

ROS map_server가 저장한 점유격자 지도(`map.pgm` + `map.yaml`)를 골라서, Gazebo에서
바로 띄울 수 있는 self-contained `world.sdf`로 변환하는 대화형 CLI 도구.

pingdergarten 레포의 `scripts/make_candidate_world.py` 변환 로직을 이식하고, 그 위에
대화형 셸 래퍼와 폴더 컨벤션을 얹는다.

## 폴더 구조

```
pgm_to_gazebo/
├── README.md              # 변환 원리 + 사용법 + 텍스처 교체법
├── .gitignore             # pgm/*, world/* 내용물 제외 (.gitkeep 예외)
├── pgm/                   # 입력: map.pgm + map.yaml 쌍 (내용물 gitignore)
│   └── .gitkeep
├── world/                 # 출력: 생성된 *.sdf (내용물 gitignore)
│   └── .gitkeep
└── scripts/
    ├── pgm-to-gazebo.sh   # 대화형 진입점
    └── pgm_to_world.py    # 변환 엔진 (numpy + pyyaml)
```

`.gitkeep`을 두는 이유: `pgm/`·`world/`의 내용물은 gitignore하지만 폴더 자체는 git에
남겨서 clone 시 구조가 보이도록 한다. `.gitignore`는 `.gitkeep`을 예외 처리한다.

## 컴포넌트

### scripts/pgm-to-gazebo.sh — 대화형 진입점

역할: 사용자 대화 UI만 담당. 실제 변환은 `pgm_to_world.py`에 위임.

흐름:
1. `pgm/` 폴더에서 `*.pgm` 스캔 → 번호 매겨 출력. 각 항목 옆에 짝 yaml 존재 여부 표시.
   ```
   [1] office.pgm   (+ office.yaml ✓)
   [2] lab.pgm      (⚠ lab.yaml 없음)
   ```
2. 번호 입력받음 → 유효성 검사(범위, 짝 yaml 존재). yaml 없으면 에러 메시지 후 종료.
3. 출력 이름 입력받음(확장자 없이, 예: `office`). 빈 입력이면 pgm 파일명 그대로 사용.
   `world/<이름>.sdf`가 이미 있으면 덮어쓸지 y/N 확인.
4. `python3 scripts/pgm_to_world.py --map pgm/<선택>.yaml --out world/<이름>.sdf` 호출.
5. 종료 코드 확인 후 성공/실패 메시지.

엣지 케이스:
- `pgm/`에 `.pgm`이 하나도 없으면 안내 후 종료.
- 번호가 숫자가 아니거나 범위 밖이면 재입력 요구(또는 종료).
- `python3` / numpy / pyyaml 미설치 시 변환 단계에서 실패 → 메시지 전달.

### scripts/pgm_to_world.py — 변환 엔진

pingdergarten `make_candidate_world.py`의 핵심 로직을 이식. CLI 인자:
- `--map <path>`: map.yaml 경로 (필수). yaml의 `image:` 필드로 pgm 경로 해석.
- `--out <path>`: 출력 sdf 파일 경로 (필수). 부모 디렉토리 자동 생성.
- `--name <str>`: world name (선택, 기본은 출력 파일 stem).

파이프라인:
1. `read_pgm()` — P5(binary) PGM만 지원. magic이 `P5`가 아니면 에러.
2. yaml에서 `resolution`, `origin[0:2]`, `free_thresh`(기본 0.196) 읽음.
3. `occ_mask = img < int(free_thresh * 255)` — 점유 픽셀 마스킹.
4. `decompose_to_rects()` — 각 행의 horizontal run을 vertical-merge해 직사각형 리스트로.
5. `pix_rect_to_world()` — 픽셀 직사각형 → world (cx, cy, sx, sy). y축은 image row 반전,
   origin 오프셋 적용. `res`보다 작은 변은 스킵.
6. `build_world_sdf()` — self-contained world.sdf 문자열 생성:
   physics + 5개 gz-sim 플러그인 + directional light(sun) + ground_plane + 벽 박스들.
   벽 높이 0.72m, z중심 0.36m.

material 생성은 별도 헬퍼(`wall_material()` / `floor_material()`)로 분리해서, 나중에
`<pbr><metal><albedo_map>` 추가가 한 곳 수정으로 끝나도록 한다.

상수:
- `WALL_HEIGHT = 0.72`, `WALL_Z = 0.36`
- 벽 색: ambient `0.96 0.93 0.85 1`, diffuse `0.98 0.95 0.88 1` (크림색)
- 바닥 색: ambient `0.78 0.76 0.72 1`, diffuse `0.82 0.80 0.76 1` (회색)

### 텍스처 — 구조만 열어두기 (이번 범위 밖, 구조만 준비)

- 지금은 단색만 출력.
- material 블록을 헬퍼 함수로 분리 → 나중에 albedo_map 추가 용이.
- 텍스처 경로 컨벤션: `world/<이름>.sdf` 옆 `world/textures/`에 PNG를 두고 상대경로 참조.
- README에 "PNG 입히는 법" 섹션으로 문서화(material 블록 교체 예시).

### README.md

PGM → SDF 변환 원리를 단계별로 설명:
1. 점유격자 지도란? (PGM 픽셀 = free/occupied/unknown, yaml의 resolution·origin 메타데이터)
2. 점유 픽셀 → 직사각형 분해 알고리즘 (row-run + vertical-merge)
3. 픽셀 ↔ 미터 좌표 변환 (해상도 곱, y축 반전, origin 오프셋)
4. SDF box 생성 (self-contained world 구조)
5. 사용법 (`./scripts/pgm-to-gazebo.sh` 대화 예시)
6. 텍스처 교체법 (material 블록 → PBR albedo_map)

## 데이터 흐름

```
pgm/office.pgm ─┐
pgm/office.yaml ┴→ pgm-to-gazebo.sh (번호 선택, 이름 입력)
                     └→ pgm_to_world.py --map pgm/office.yaml --out world/office.sdf
                          read_pgm → occ_mask → decompose_to_rects
                          → pix_rect_to_world → build_world_sdf
                          → world/office.sdf
```

## 에러 처리

| 상황 | 처리 |
|------|------|
| `pgm/`에 .pgm 없음 | 셸이 안내 후 종료 |
| 짝 yaml 없음 | 셸이 해당 항목에 ⚠ 표시, 선택 시 에러 종료 |
| 잘못된 번호 입력 | 재입력 요구 |
| 출력 파일 이미 존재 | y/N 덮어쓰기 확인 |
| P5 아닌 PGM | python이 ValueError, 셸에 실패 전달 |
| numpy/pyyaml 미설치 | python ImportError, 셸에 실패 전달 |

## 테스트

- `pgm_to_world.py`의 순수 함수(`runs_in_row`, `decompose_to_rects`, `pix_rect_to_world`)에
  대한 단위 테스트: 알려진 작은 mask → 기대 직사각형/좌표.
- 작은 합성 PGM + yaml로 end-to-end: 생성된 sdf가 valid XML이고 벽 박스 개수가 맞는지.
- 셸 스크립트는 수동 검증(대화형) + .pgm 없는 빈 폴더 시 안내 메시지 확인.

## 범위 밖 (YAGNI)

- 벽/가구 자동 분류, 방별 바닥 타일, 라벨 PNG 생성, 데코 배치 (pingdergarten 전용 기능)
- 텍스처 자동 적용 (구조만 열어둠)
- P2(ASCII) PGM 지원
