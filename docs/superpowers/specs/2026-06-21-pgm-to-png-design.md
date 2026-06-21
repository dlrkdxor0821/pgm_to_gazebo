# PGM → PNG 자동 동반 출력 설계

## 목적

`map.pgm`(점유격자)을 같은 픽셀의 `map.png`로도 내보낸다. 별도 스크립트가
아니라 기존 변환 엔진 `pgm_to_world.py`에 합쳐, world.sdf 변환을 돌리면
PNG도 **자동으로 같이** 생성되게 한다.

## 동작

```
pgm/office/map.pgm   →   world/office.sdf   +   png/office/map.png
```

- `pgm-to-gazebo.sh`로 폴더를 골라 변환하면 world.sdf와 png가 한 번에 생성된다.
- PNG는 PGM 픽셀을 1:1 그대로 옮긴 8-bit grayscale (크기·회색값 동일). 점유격자
  이미지를 일반 뷰어/도구에서 바로 열기 위함이며, 별도 가공은 하지 않는다.

## 변경 사항

1. **`scripts/pgm_to_world.py`**
   - `write_png(gray: np.ndarray, path: Path) -> None` 추가.
     - 순수 stdlib(`zlib` + `struct`)로 8-bit grayscale PNG를 인코딩한다.
       (filter 0, color type 0) — 새 의존성 없음(numpy/pyyaml 그대로 유지).
   - `main()`에 `--png <경로>` 선택 인자 추가. 주어지면 이미 읽어둔 `img`를
     해당 경로에 PNG로 저장하고 `[output]` 한 줄을 출력한다. 없으면 기존과 동일.

2. **`scripts/pgm-to-gazebo.sh`**
   - 변환 시 `png/<source_folder>/<pgm_stem>.png` 경로를 계산해 `--png`로 항상 넘긴다
     (출력 폴더는 pgm 소스 폴더 이름을 거울처럼 따른다).
   - 완료 메시지에 생성된 png 경로도 함께 안내한다.

3. **`png/` 폴더 신설**
   - `png/.gitkeep` 추가.
   - `.gitignore`에 `png/*`, `!png/.gitkeep` 추가 (pgm·world와 동일하게 내용물 미추적).

4. **`tests/test_pgm_to_world.py`**
   - `write_png` 출력이 유효한 PNG 시그니처/크기를 갖는지, PGM→PNG 회색값이
     보존되는지 검증.
   - `--png` 인자 사용 시 main이 png 파일을 함께 생성하는지 end-to-end 검증.

5. **`README.md`**
   - png 자동 동반 출력과 `png/` 폴더를 폴더 구조/빠른 시작에 한 줄씩 반영.

## 비목표

- 임의 경로 일회성 CLI, 일괄 변환, 색상/리사이즈 가공은 범위 밖(YAGNI).
- Pillow 등 새 이미지 라이브러리 도입하지 않는다.
