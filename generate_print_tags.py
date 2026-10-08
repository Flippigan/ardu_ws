#!/usr/bin/env python3
"""Generate print-ready PDFs for AprilTag landing targets.

Outputs:
  print_tags/tag36_11_00000_600mm.pdf  - Primary tag, 600mm x 600mm
  print_tags/tag36_11_00001_150mm.pdf  - Secondary tag, 150mm x 150mm

Both PDFs include dimension labels and crop marks for the print shop.
"""

import os
from pathlib import Path
from PIL import Image
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader
import io


TAGS = [
    {
        "id": "tag36_11_00000",
        "src": "src/ardupilot_gazebo/models/Apriltag36_11_00000/materials/textures/tag36_11_00000.png",
        "size_mm": 600,
        "label": "Primary Tag — tag36h11 ID 0",
    },
    {
        "id": "tag36_11_00001",
        "src": "src/ardupilot_gazebo/models/Apriltag36_11_00001/materials/textures/tag36_11_00001.png",
        "size_mm": 150,
        "label": "Secondary Tag — tag36h11 ID 1",
    },
]

WORKSPACE = Path(__file__).parent
OUTPUT_DIR = WORKSPACE / "print_tags"
MARGIN = 15 * mm  # margin around tag for crop marks and labels
CROP_MARK_LEN = 8 * mm
CROP_MARK_OFFSET = 3 * mm  # gap between tag edge and crop mark


def draw_crop_marks(c, x, y, w, h):
    """Draw crop marks at the four corners of the tag area."""
    c.setStrokeColorRGB(0, 0, 0)
    c.setLineWidth(0.5)
    offset = CROP_MARK_OFFSET
    length = CROP_MARK_LEN

    corners = [
        (x, y),                  # bottom-left
        (x + w, y),              # bottom-right
        (x + w, y + h),          # top-right
        (x, y + h),              # top-left
    ]
    # Each corner gets two lines (horizontal + vertical) pointing outward
    directions = [
        [(-1, 0), (0, -1)],     # bottom-left
        [(1, 0), (0, -1)],      # bottom-right
        [(1, 0), (0, 1)],       # top-right
        [(-1, 0), (0, 1)],      # top-left
    ]
    for (cx, cy), dirs in zip(corners, directions):
        for dx, dy in dirs:
            x0 = cx + dx * offset
            y0 = cy + dy * offset
            x1 = x0 + dx * length
            y1 = y0 + dy * length
            c.line(x0, y0, x1, y1)


def generate_tag_pdf(tag_info):
    """Generate a single print-ready PDF for one tag."""
    src_path = WORKSPACE / tag_info["src"]
    size_mm_val = tag_info["size_mm"]
    size_pt = size_mm_val * mm
    label = tag_info["label"]

    # Page size: tag + margins on all sides + extra bottom for label
    label_space = 12 * mm
    page_w = size_pt + 2 * MARGIN
    page_h = size_pt + 2 * MARGIN + label_space

    out_path = OUTPUT_DIR / f"{tag_info['id']}_{size_mm_val}mm.pdf"
    c = canvas.Canvas(str(out_path), pagesize=(page_w, page_h))

    # Tag position (centered horizontally, offset from bottom to leave label room)
    tag_x = MARGIN
    tag_y = MARGIN + label_space

    # Load image — convert RGBA to RGB on white background for PDF
    img = Image.open(src_path)
    if img.mode == "RGBA":
        bg = Image.new("RGB", img.size, (255, 255, 255))
        bg.paste(img, mask=img.split()[3])
        img = bg
    elif img.mode != "RGB":
        img = img.convert("RGB")

    # Write image to buffer
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    buf.seek(0)

    # Draw the tag image at exact physical size
    c.drawImage(ImageReader(buf), tag_x, tag_y, width=size_pt, height=size_pt)

    # Draw crop marks
    draw_crop_marks(c, tag_x, tag_y, size_pt, size_pt)

    # Dimension labels along edges
    c.setFont("Helvetica", 8)
    c.setFillColorRGB(0.3, 0.3, 0.3)

    # Top dimension
    dim_text = f"{size_mm_val} mm"
    c.drawCentredString(tag_x + size_pt / 2, tag_y + size_pt + CROP_MARK_OFFSET + CROP_MARK_LEN + 2 * mm, dim_text)

    # Right dimension (rotated)
    c.saveState()
    c.translate(tag_x + size_pt + CROP_MARK_OFFSET + CROP_MARK_LEN + 3 * mm, tag_y + size_pt / 2)
    c.rotate(90)
    c.drawCentredString(0, 0, dim_text)
    c.restoreState()

    # Label at bottom
    c.setFont("Helvetica-Bold", 10)
    c.setFillColorRGB(0, 0, 0)
    c.drawCentredString(page_w / 2, MARGIN / 2 + label_space / 2, label)

    c.setFont("Helvetica", 7)
    c.setFillColorRGB(0.4, 0.4, 0.4)
    c.drawCentredString(page_w / 2, MARGIN / 2, f"Print at exactly {size_mm_val} x {size_mm_val} mm — MATTE finish — no scaling — tag36h11 family")

    c.save()
    print(f"  Created: {out_path}  ({size_mm_val}mm x {size_mm_val}mm)")


def main():
    OUTPUT_DIR.mkdir(exist_ok=True)
    print("Generating print-ready AprilTag PDFs...")
    for tag in TAGS:
        generate_tag_pdf(tag)
    print(f"\nFiles in: {OUTPUT_DIR}/")
    print("Instructions:")
    print("  - Primary (600mm): send to large-format / poster printer")
    print("  - Secondary (150mm): prints on standard A4/Letter")
    print("  - Tell the print shop: NO SCALING, matte finish, exact dimensions")


if __name__ == "__main__":
    main()
