#!/usr/bin/env bash
# pgm/ 의 지도 폴더를 골라 world/ 에 self-contained world.sdf 를 생성한다.
# pgm/ 바로 아래 흩어진 파일은 무시하고, "폴더" 단위로만 인식한다.
# 각 폴더는 map.yaml + (yaml 의 image: 가 가리키는) map.pgm 한 쌍을 담는다.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PGM_DIR="$REPO_DIR/pgm"
WORLD_DIR="$REPO_DIR/world"
PNG_DIR="$REPO_DIR/png"

mkdir -p "$WORLD_DIR" "$PNG_DIR"

# yaml 의 image: 필드 값을 추출 (앞뒤 따옴표 제거). 없으면 빈 문자열.
extract_image_field() {
    grep -E '^[[:space:]]*image:' "$1" 2>/dev/null | head -1 \
        | sed -E 's/^[[:space:]]*image:[[:space:]]*//; s/[[:space:]]*$//; s/^["'"'"']//; s/["'"'"']$//'
}

# 폴더 하나를 검사. echo "<yaml_path>|<pgm_path>" 후 return 0 (정상),
# 또는 echo "<사유>" 후 return 1 (문제).
inspect_folder() {
    local dir="$1"
    shopt -s nullglob
    local yamls=("$dir"/*.yaml "$dir"/*.yml)
    shopt -u nullglob
    if [ ${#yamls[@]} -eq 0 ]; then
        echo "yaml 없음"
        return 1
    fi
    local yaml="${yamls[0]}"
    local img
    img="$(extract_image_field "$yaml")"
    if [ -z "$img" ]; then
        echo "yaml 에 image 필드 없음 ($(basename "$yaml"))"
        return 1
    fi
    local pgm="$dir/$img"
    if [ ! -f "$pgm" ]; then
        echo "yaml 의 image '$img' 에 해당하는 pgm 없음"
        return 1
    fi
    echo "$yaml|$pgm"
    return 0
}

# pgm/ 의 하위 폴더 수집
shopt -s nullglob
folders=("$PGM_DIR"/*/)
shopt -u nullglob

if [ ${#folders[@]} -eq 0 ]; then
    echo "[안내] pgm/ 폴더 안에 지도 폴더가 없습니다."
    echo "       pgm/<이름>/ 폴더를 만들고 map.pgm 과 map.yaml 을 넣어주세요."
    exit 1
fi

echo "=== pgm/ 의 지도 폴더 목록 ==="
i=1
for d in "${folders[@]}"; do
    name="$(basename "$d")"
    if result="$(inspect_folder "$d")"; then
        pgm_name="$(basename "${result#*|}")"
        printf "  [%d] %s/   (✓ %s)\n" "$i" "$name" "$pgm_name"
    else
        printf "  [%d] %s/   (⚠ %s)\n" "$i" "$name" "$result"
    fi
    i=$((i + 1))
done

# 번호 선택
read -rp "변환할 폴더 번호를 선택하세요 [1-${#folders[@]}]: " choice
if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt ${#folders[@]} ]; then
    echo "[error] 잘못된 번호입니다: $choice"
    exit 1
fi

sel_dir="${folders[$((choice - 1))]}"
sel_name="$(basename "$sel_dir")"

if ! result="$(inspect_folder "$sel_dir")"; then
    echo "[error] '$sel_name' 폴더를 변환할 수 없습니다: $result"
    echo "        폴더 안에 map.yaml 과 yaml 이 가리키는 pgm 이 함께 있어야 합니다."
    exit 1
fi
sel_yaml="${result%%|*}"
sel_pgm="${result#*|}"

# png 는 pgm 소스 폴더 구조를 거울처럼 따른다: png/<폴더>/<pgm이름>.png
pgm_base="$(basename "$sel_pgm")"
png_out="$PNG_DIR/$sel_name/${pgm_base%.*}.png"

# 출력 이름 입력
read -rp "출력 world 이름 (엔터 시 '$sel_name'): " out_name
out_name="${out_name:-$sel_name}"
out_name="${out_name%.sdf}"  # .sdf 붙여 입력해도 제거
out_path="$WORLD_DIR/$out_name.sdf"

if [ -f "$out_path" ]; then
    read -rp "$out_name.sdf 가 이미 있습니다. 덮어쓸까요? [y/N]: " yn
    case "$yn" in
        [yY]) ;;
        *) echo "취소했습니다."; exit 0 ;;
    esac
fi

# 변환 실행 (world.sdf 와 png 를 한 번에 같이 생성)
echo "--- 변환 중 ---"
python3 "$SCRIPT_DIR/pgm_to_world.py" --map "$sel_yaml" --out "$out_path" --png "$png_out"
echo "--- 완료 ---"
echo "  world: $out_path"
echo "  png  : $png_out"
echo "Gazebo 실행 예: gz sim \"$out_path\""
