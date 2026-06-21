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


import struct
import zlib


def _read_png_gray(path):
    """테스트용 8-bit grayscale PNG 디코더 (filter 0 만 가정)."""
    data = Path(path).read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos = 8
    width = height = bitdepth = colortype = None
    idat = b""
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos : pos + 4])
        tag = data[pos + 4 : pos + 8]
        body = data[pos + 8 : pos + 8 + length]
        pos += 12 + length
        if tag == b"IHDR":
            width, height, bitdepth, colortype = struct.unpack(">IIBB", body[:10])
        elif tag == b"IDAT":
            idat += body
        elif tag == b"IEND":
            break
    raw = zlib.decompress(idat)
    rows = []
    for r in range(height):
        start = r * (width + 1)
        assert raw[start] == 0  # filter type 0 (None)
        rows.append(list(raw[start + 1 : start + 1 + width]))
    return width, height, bitdepth, colortype, np.array(rows, dtype=np.uint8)


def test_write_png_preserves_grayscale_pixels(tmp_path):
    arr = np.array([[0, 50, 254], [205, 128, 0]], dtype=np.uint8)
    out = tmp_path / "x.png"
    p2w.write_png(arr, out)
    w, h, bitdepth, colortype, got = _read_png_gray(out)
    assert (w, h) == (3, 2)
    assert bitdepth == 8 and colortype == 0  # 8-bit grayscale
    assert np.array_equal(got, arr)


def test_write_png_creates_parent_dirs(tmp_path):
    arr = np.zeros((2, 2), dtype=np.uint8)
    out = tmp_path / "nested" / "deep" / "x.png"
    p2w.write_png(arr, out)
    assert out.is_file()


def test_main_writes_png_alongside_world_when_requested(tmp_path):
    arr = np.full((5, 5), 254, dtype=np.uint8)
    arr[2, 2] = 0
    pgm = tmp_path / "m.pgm"
    _write_pgm(pgm, arr)
    yaml_path = tmp_path / "m.yaml"
    yaml_path.write_text(
        "image: m.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nfree_thresh: 0.196\n"
    )
    out = tmp_path / "out.sdf"
    png = tmp_path / "png" / "m.png"
    rc = p2w.main(["--map", str(yaml_path), "--out", str(out), "--png", str(png)])
    assert rc == 0
    assert out.is_file()
    assert png.is_file()
    w, h, _, _, got = _read_png_gray(png)
    assert (w, h) == (5, 5)
    assert np.array_equal(got, arr)  # pgm 픽셀이 png 에 1:1 보존
