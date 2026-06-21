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
