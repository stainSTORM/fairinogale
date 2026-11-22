"""Printable AprilTags (family 36h11) for the stations and the carriers.

    uv run tags.py                                    the two sheets in tags/ (PDF and SVG, A4)
    uv run tags.py --ids 40 41 --size 20 --name extra   a sheet of your own: tags/extra.pdf

Print at 100 % ("actual size", not "fit to page") and measure the 100 mm bar on the sheet. The
size of a tag is the edge of its black square; config.yaml needs it (vision.tag_size_mm for the
stations, vision.carrier_tag_size_mm for the carriers). Keep the white margin around each tag:
the detector needs it. docs/vision.md says where the tags go.
"""

import argparse
from pathlib import Path

from vision import DICTIONARY

A4 = (210.0, 297.0)
MARGIN, GAP, LABEL = 12.0, 6.0, 7.0  # mm: sheet margin, space between tags, room for the label

# Which tag goes where. Stations get 30 mm tags, carriers 15 mm ones.
STATIONS = {0: "pickup station", 1: "pickup station", 2: "pickup station (spare)", 3: "pickup station (spare)",
            10: "Opentrons", 11: "Opentrons", 12: "Opentrons (spare)", 13: "Opentrons (spare)",
            20: "microscope", 21: "microscope", 22: "microscope (spare)", 23: "microscope (spare)",
            30: "spare station", 31: "spare station", 32: "spare station", 33: "spare station"}
CARRIERS = {i: f"carrier {i - 99}" for i in range(100, 148)}


def sheet(tags: dict[int, str], size: float, title: str) -> list[tuple]:
    """One A4 page as drawing steps in mm from the top left: ("rect", x, y, w, h, gray),
    ("text", x, y, points, string), ("line", x1, y1, x2, y2)."""
    cell = size / 8          # a 36h11 tag is 8 x 8 cells, the outer ring black
    outer = size + 2 * cell  # with one cell of white around it
    columns = int((A4[0] - 2 * MARGIN + GAP) // (outer + GAP))
    steps = [("text", MARGIN, MARGIN, 10, f"{title}: AprilTag 36h11, black square {size:g} mm. "
                                          f"Print at 100 %, then check the bar below.")]
    for k, (i, label) in enumerate(tags.items()):
        x = MARGIN + (k % columns) * (outer + GAP)
        y = MARGIN + 6 + (k // columns) * (outer + LABEL + GAP)
        if y + outer + LABEL > A4[1] - MARGIN - 12:
            raise ValueError(f"{len(tags)} tags of {size:g} mm do not fit on one A4 sheet")
        steps.append(("rect", x, y, outer, outer, 0.75))                       # cut line
        steps.append(("rect", x + 0.15, y + 0.15, outer - 0.3, outer - 0.3, 1.0))
        steps.append(("rect", x + cell, y + cell, size, size, 0.0))
        bits = DICTIONARY.generateImageMarker(i, 8)  # 8 x 8 pixels, 255 = white
        for row in range(8):
            for column in range(8):
                if bits[row, column]:
                    steps.append(("rect", x + cell * (1 + column), y + cell * (1 + row), cell, cell, 1.0))
        steps.append(("text", x, y + outer + 3.2, 7, f"id {i}  {label}"))
    bar = A4[1] - MARGIN - 4
    steps.append(("line", MARGIN, bar, MARGIN + 100, bar))
    for tick in range(0, 101, 10):
        steps.append(("line", MARGIN + tick, bar - (2.5 if tick % 50 == 0 else 1.2), MARGIN + tick, bar))
    steps.append(("text", MARGIN + 104, bar, 8, "100 mm"))
    return steps


def svg(steps: list[tuple]) -> str:
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{A4[0]}mm" height="{A4[1]}mm" '
           f'viewBox="0 0 {A4[0]} {A4[1]}" shape-rendering="crispEdges">']
    for step in steps:
        if step[0] == "rect":
            _, x, y, w, h, gray = step
            shade = round(gray * 255)
            out.append(f'<rect x="{x:.3f}" y="{y:.3f}" width="{w:.3f}" height="{h:.3f}" '
                       f'fill="rgb({shade},{shade},{shade})"/>')
        elif step[0] == "line":
            _, x1, y1, x2, y2 = step
            out.append(f'<line x1="{x1:.3f}" y1="{y1:.3f}" x2="{x2:.3f}" y2="{y2:.3f}" stroke="black" '
                       f'stroke-width="0.2"/>')
        else:
            _, x, y, points, string = step
            out.append(f'<text x="{x:.3f}" y="{y:.3f}" font-family="Helvetica, Arial, sans-serif" '
                       f'font-size="{points * 25.4 / 72:.3f}">{string}</text>')
    return "\n".join(out + ["</svg>"])


def pdf(steps: list[tuple]) -> bytes:
    """A one-page PDF, written by hand: rectangles, lines and Helvetica text need no library."""
    pt = 72 / 25.4
    draw = []
    for step in steps:
        if step[0] == "rect":
            _, x, y, w, h, gray = step
            draw.append(f"{gray:.2f} g {x * pt:.3f} {(A4[1] - y - h) * pt:.3f} {w * pt:.3f} {h * pt:.3f} re f")
        elif step[0] == "line":
            _, x1, y1, x2, y2 = step
            draw.append(f"0 G 0.5 w {x1 * pt:.3f} {(A4[1] - y1) * pt:.3f} m "
                        f"{x2 * pt:.3f} {(A4[1] - y2) * pt:.3f} l S")
        else:
            _, x, y, points, string = step
            safe = string.replace("\\", r"\\").replace("(", r"\(").replace(")", r"\)")
            draw.append(f"0 g BT /F1 {points} Tf {x * pt:.3f} {(A4[1] - y) * pt:.3f} Td ({safe}) Tj ET")
    stream = "\n".join(draw).encode("latin-1")
    objects = [b"<< /Type /Catalog /Pages 2 0 R >>",
               b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
               f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {A4[0] * pt:.2f} {A4[1] * pt:.2f}] "
               f"/Contents 5 0 R /Resources << /Font << /F1 4 0 R >> >> >>".encode(),
               b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
               b"<< /Length %d >>\nstream\n" % len(stream) + stream + b"\nendstream"]
    out, offsets = bytearray(b"%PDF-1.4\n"), []
    for number, body in enumerate(objects, 1):
        offsets.append(len(out))
        out += b"%d 0 obj\n" % number + body + b"\nendobj\n"
    start = len(out)
    out += b"xref\n0 %d\n0000000000 65535 f \n" % (len(objects) + 1)
    out += b"".join(b"%010d 00000 n \n" % offset for offset in offsets)
    out += b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (len(objects) + 1, start)
    return bytes(out)


def write(name: str, tags: dict[int, str], size: float, title: str, folder: Path) -> None:
    steps = sheet(tags, size, title)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / f"{name}.pdf").write_bytes(pdf(steps))
    (folder / f"{name}.svg").write_text(svg(steps), encoding="utf-8")
    print(f"{folder / name}.pdf and .svg: ids {min(tags)} to {max(tags)}, {size:g} mm")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ids", type=int, nargs="+", help="tag ids for a sheet of your own")
    parser.add_argument("--size", type=float, default=30.0, help="edge of the black square in mm")
    parser.add_argument("--name", default="custom", help="file name of your own sheet, without ending")
    parser.add_argument("--out", type=Path, default=Path(__file__).parent / "tags")
    args = parser.parse_args()
    if args.ids:
        write(args.name, {i: "" for i in args.ids}, args.size, "Tags", args.out)
    else:
        write("stations-30mm", STATIONS, 30.0, "Station tags", args.out)
        write("carriers-15mm", CARRIERS, 15.0, "Carrier tags", args.out)
