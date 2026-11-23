#!/usr/bin/env python
"""Make print-ready PDF sheets for go2/tag_tracker.py. Never touches the robot.

    python go2/make_tag_sheets.py                 # go2/tags/tag_sheets_a4.pdf and tag_sheets_letter.pdf

Four pages:
  1. the robot tag (tag36h11 ID 0, black square 120 mm), taped flat on the Go2's back between
     the handle and the head
  2. the floor tag (tag36h11 ID 1, black square 140 mm), flat on the floor beside the lane
  3. the crate tag (tag36h11 ID 2, black square 140 mm), flat on the middle of the top of
     push_box_turn's crate
  4. a checkerboard (7 x 9 squares of 25 mm, so 6 x 8 inner corners) for calibrating the camera

Print at 100% ("actual size"), never "fit to page". Each page has a 100 mm bar, so
measure it to check. The tags come from AprilRobotics/apriltag-imgs (go2/tags/, BSD
licence). They're drawn rotated 180 degrees from those images, so that OpenCV's idea
of the tag's top edge is the top of the page, where the arrow points. A tag without its
image there (ID 2 has none) is drawn from OpenCV's own tag36h11 marker, which is that
image already turned: they match cell for cell for IDs 0 and 1.
"""
import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import FancyArrow, Rectangle
from PIL import Image

TAG_DIR = Path(__file__).resolve().parent / "tags"
PAGES = {"a4": (210.0, 297.0), "letter": (215.9, 279.4)}  # mm
ROBOT_TAG = {"id": 0, "black_mm": 120.0}
FLOOR_TAG = {"id": 1, "black_mm": 140.0}
CRATE_TAG = {"id": 2, "black_mm": 140.0}
BOARD = {"columns": 7, "rows": 9, "square_mm": 25.0}  # squares; inner corners are one fewer each way
CUT_MARGIN_CELLS = 0.5  # white kept outside the tag's own one-cell white border
GREY = "0.6"
SCALE_BAR_Y = 14.0  # mm above the bottom of the page
NOTE_SPACING = 5.0  # mm


def tag_cells(tag_id):
    """10 x 10 booleans (True = black), row 0 at the top of the page, rotated to match OpenCV."""
    path = TAG_DIR / f"tag36_11_{tag_id:05d}.png"
    if not path.exists():
        import cv2
        cells = np.zeros((10, 10), dtype=bool)
        marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                                               tag_id, 8, borderBits=1)
        cells[1:9, 1:9] = marker < 128
        return cells
    image = np.array(Image.open(path).convert("L"))
    if image.shape != (10, 10):
        raise ValueError(f"tag {tag_id}: expected a 10 x 10 image, got {image.shape}")
    return np.rot90(image < 128, 2)


def new_page(size):
    width, height = size
    figure = plt.figure(figsize=(width / 25.4, height / 25.4))
    axes = figure.add_axes((0, 0, 1, 1))
    axes.set_xlim(0, width)
    axes.set_ylim(0, height)
    axes.set_axis_off()
    return figure, axes


def scale_bar(axes, x, y):
    axes.add_patch(Rectangle((x, y), 100.0, 1.2, color="black", lw=0))
    for mm in range(0, 101, 10):
        axes.plot([x + mm, x + mm], [y, y + (5 if mm % 50 == 0 else 3)], color="black", lw=0.6)
    axes.text(x + 50, y - 2.5, "this bar must measure 100 mm. If it doesn't, reprint at 100% (actual size)",
              ha="center", va="top", fontsize=7.5)


def cut_square(axes, x, y, side):
    axes.add_patch(Rectangle((x, y), side, side, fill=False, ec=GREY, lw=0.5, ls=(0, (4, 3))))


def draw_tag(axes, cells, centre_x, top, black_mm):
    """Draw the tag with its black square black_mm wide; return the cut square's bottom y."""
    cell = black_mm / 8.0
    left = centre_x - 5 * cell
    for row, col in zip(*np.nonzero(cells)):
        axes.add_patch(Rectangle((left + col * cell, top - (row + 1) * cell), cell, cell,
                                 color="black", lw=0, antialiased=False))
    margin = CUT_MARGIN_CELLS * cell
    side = 10 * cell + 2 * margin
    cut_square(axes, left - margin, top - 10 * cell - margin, side)
    return top - 10 * cell - margin


def tag_page(size, tag, title, arrow_label, notes):
    figure, axes = new_page(size)
    width, height = size
    centre = width / 2
    axes.text(centre, height - 10, title, ha="center", va="top", fontsize=13, weight="bold")
    arrow_bottom = height - 30
    axes.add_patch(FancyArrow(centre, arrow_bottom, 0, 14, width=2.0, head_width=8, head_length=6,
                              length_includes_head=True, color="black"))
    axes.text(centre + 7, arrow_bottom + 4, arrow_label, ha="left", va="center", fontsize=10)
    cell = tag["black_mm"] / 8.0
    tag_top = arrow_bottom - 4 - CUT_MARGIN_CELLS * cell
    bottom = draw_tag(axes, tag_cells(tag["id"]), centre, tag_top, tag["black_mm"])
    lines = [f"tag36h11, ID {tag['id']}. The black square must measure {tag['black_mm']:g} mm edge to edge.",
             "Cut on the dashed line, keeping the white border: the detector needs it. Glue it to stiff card."] + notes
    lowest = bottom - 6 - NOTE_SPACING * len(lines)
    if lowest < SCALE_BAR_Y + 6:
        raise ValueError(f"{title}: the notes run into the scale bar on a {width:g} x {height:g} mm page")
    for i, line in enumerate(lines):
        axes.text(centre, bottom - 6 - NOTE_SPACING * i, line, ha="center", va="top", fontsize=8.5)
    scale_bar(axes, centre - 50, SCALE_BAR_Y)
    return figure


def board_page(size):
    figure, axes = new_page(size)
    width, height = size
    columns, rows, square = BOARD["columns"], BOARD["rows"], BOARD["square_mm"]
    left = (width - columns * square) / 2
    top = height - 22
    axes.text(width / 2, height - 8, f"Camera calibration board: {columns} x {rows} squares of {square:g} mm "
              f"({columns - 1} x {rows - 1} inner corners)", ha="center", va="top", fontsize=10, weight="bold")
    for row in range(rows):
        for col in range(columns):
            if (row + col) % 2 == 0:
                axes.add_patch(Rectangle((left + col * square, top - (row + 1) * square), square, square,
                                         color="black", lw=0, antialiased=False))
    axes.text(width / 2, top - rows * square - 3,
              "Glue it flat to stiff card. Measure a square and pass it to tag_tracker.py calibrate --square-mm.",
              ha="center", va="top", fontsize=8)
    scale_bar(axes, width / 2 - 50, SCALE_BAR_Y)
    return figure


def make(path, size):
    pages = [
        tag_page(size, ROBOT_TAG, "Go2 robot tag", "robot's head this way",
                 ["Tape it flat on the back between the handle and the head, centred, arrow towards the head."]),
        tag_page(size, FLOOR_TAG, "Floor tag", "walking direction",
                 ["Lay it flat beside the lane, on the camera's side, arrow along the lane.",
                  "Forward, left and yaw are measured from it, so don't move it during a session."]),
        tag_page(size, CRATE_TAG, "Crate tag", "any way round",
                 ["Lay it flat on the middle of the crate's top, face up. Only its position is used,",
                  "so the arrow can point anywhere."]),
        board_page(size),
    ]
    with PdfPages(path, metadata={"Title": "Go2 AprilTag sheets", "Creator": "go2/make_tag_sheets.py"}) as pdf:
        for figure in pages:
            pdf.savefig(figure)
            plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out-dir", type=Path, default=TAG_DIR, help="where to write the PDFs (default: go2/tags)")
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for name, size in PAGES.items():
        path = args.out_dir / f"tag_sheets_{name}.pdf"
        make(path, size)
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
