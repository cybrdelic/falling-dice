#!/usr/bin/env python3
import sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

root = Path(sys.argv[1])
out = root / "presentation"
out.mkdir(parents=True, exist_ok=True)
shots = [
    ("hero_front.png", "FRONT THREE-QUARTER"),
    ("hero_rear.png", "REAR THREE-QUARTER"),
    ("side.png", "SIDE PACKAGE"),
    ("front_low.png", "LOW FRONT"),
]
images = [Image.open(root / "renders" / f).convert("RGB") for f, _ in shots]
cell_w, cell_h = 1400, 875
board = Image.new("RGB", (cell_w * 2, (cell_h + 110) * 2 + 120), (12, 13, 16))
draw = ImageDraw.Draw(board)
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 34)
    small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 21)
except OSError:
    font = ImageFont.load_default()
    small = font
for i, (image, (_, label)) in enumerate(zip(images, shots)):
    image.thumbnail((cell_w, cell_h), Image.Resampling.LANCZOS)
    x = (i % 2) * cell_w + (cell_w - image.width) // 2
    y = 80 + (i // 2) * (cell_h + 110) + (cell_h - image.height) // 2
    board.paste(image, (x, y))
    draw.text((x + 34, y + image.height + 26), label, font=font, fill=(235, 237, 242))
    draw.text((x + 34, y + image.height + 70), "IDENTICAL EXACT B-REP ASSEMBLY", font=small, fill=(138, 144, 156))
draw.text((42, 24), "ASTRA GT  /  EXACT VEHICLE PROGRAM", font=font, fill=(245, 246, 249))
board.save(out / "ASTRA_GT_four_view_board.png", compress_level=6)

diags = [Image.open(root / "diagnostics" / f"{v}.png").convert("RGB") for v in ["side", "front", "top"]]
diag_board = Image.new("RGB", (1200, 2100), (244, 245, 247))
draw = ImageDraw.Draw(diag_board)
for i, (im, label) in enumerate(zip(diags, ["SIDE", "FRONT", "TOP"])):
    im.thumbnail((1120, 610), Image.Resampling.LANCZOS)
    x = (1200 - im.width) // 2
    y = 70 + i * 680
    diag_board.paste(im, (x, y))
    draw.text((42, y + im.height + 18), label, font=font, fill=(22, 25, 31))
diag_board.save(out / "ASTRA_GT_package_diagnostics.png", compress_level=6)
