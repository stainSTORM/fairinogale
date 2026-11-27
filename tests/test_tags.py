"""The printable sheets: every tag on them must be found again, upright and at its printed size."""

import cv2
import numpy as np

import tags
import vision

SCALE = 8  # pixels per mm


def drawn(steps):
    """The sheet's rectangles as an image."""
    image = np.full((int(tags.A4[1] * SCALE), int(tags.A4[0] * SCALE)), 255, np.uint8)
    for step in steps:
        if step[0] == "rect":
            _, x, y, w, h, gray = step
            cv2.rectangle(image, (round(x * SCALE), round(y * SCALE)),
                          (round((x + w) * SCALE) - 1, round((y + h) * SCALE) - 1), int(gray * 255), -1)
    return image


def test_every_tag_is_found_at_its_size():
    for wanted, size in ((tags.STATIONS, 30.0), (tags.CARRIERS, 15.0)):
        found = vision.detect(drawn(tags.sheet(wanted, size, "test")))
        assert sorted(found) == sorted(wanted)
        for corners in found.values():  # clockwise from the top left, as printed
            assert abs(np.linalg.norm(corners[1] - corners[0]) / SCALE - size) < 0.3
            assert corners[1][0] > corners[0][0] and corners[3][1] > corners[0][1]


def test_pdf_and_svg_are_written(tmp_path):
    tags.write("sheet", {5: "five"}, 20.0, "Test", tmp_path)
    data = (tmp_path / "sheet.pdf").read_bytes()
    assert data.startswith(b"%PDF-1.4") and data.rstrip().endswith(b"%%EOF")
    start = int(data[data.rindex(b"startxref") + 9:].split()[0])
    assert data[start:start + 4] == b"xref"
    assert "<svg" in (tmp_path / "sheet.svg").read_text()
