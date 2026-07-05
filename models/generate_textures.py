#!/usr/bin/env python3
"""Generate procedural textures for the maze world.

The maze surfaces used to be flat solid colours, which gives a monocular
camera almost nothing to track. This script renders self-contained,
high-corner-density textures (grids, bricks, Mondrian-style panels, crate /
pillar / barrel skins) so the visual-odometry front-end has trackable
Shi-Tomasi corners. No external image assets are used or downloaded.

Outputs (all committed alongside this script, like generate_markers.py):
  vo_textures/materials/textures/floor_tiles.png    - tiled floor (main VO feature source)
  vo_textures/materials/textures/wall_brick.png     - offset brick wall
  vo_textures/materials/textures/wall_panels.png     - plank / panel wall
  vo_textures/materials/textures/wall_blocks.png     - ashlar stone-block wall
  vo_textures/materials/textures/painting_1..6.png  - distinct high-contrast wall art
  crate/materials/textures/crate.png                - wooden crate skin
  pillar/materials/textures/pillar.png              - concrete pillar skin
  barrel/materials/textures/barrel.png              - metal barrel skin

Rerun after editing to regenerate; results are deterministic (fixed seed).
"""

import os
import random

try:
    from PIL import Image, ImageDraw
    import numpy as np
except ImportError:
    print("Pillow and numpy are required: pip install pillow numpy")
    raise SystemExit(0)

SEED = 20260705
random.seed(SEED)
np.random.seed(SEED)

HERE = os.path.dirname(os.path.abspath(__file__))


def _save(img, *parts):
    path = os.path.join(HERE, *parts)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    img.save(path)
    print(f"  {os.path.relpath(path, HERE)}")


def _speckle(img, amount=0.05, strength=28):
    """Add fine per-pixel noise for micro-texture (helps sub-pixel KLT)."""
    arr = np.asarray(img).astype(np.int16)
    noise = np.random.randint(-strength, strength + 1, arr.shape[:2])
    mask = np.random.random(arr.shape[:2]) < amount
    for c in range(3):
        arr[:, :, c] = np.clip(arr[:, :, c] + noise * mask, 0, 255)
    return Image.fromarray(arr.astype(np.uint8))


def _jitter(base, spread):
    return tuple(int(np.clip(c + random.randint(-spread, spread), 0, 255)) for c in base)


def make_floor(size=1024, tiles=32):
    """Tiled floor: a fine grid of jittered tiles with dark grout. Grid
    intersections are the workhorse corner features for the low camera."""
    img = Image.new("RGB", (size, size), (60, 58, 54))  # grout colour
    draw = ImageDraw.Draw(img)
    step = size / tiles
    grout = max(1, int(step * 0.10))
    base = (150, 145, 135)
    for i in range(tiles):
        for j in range(tiles):
            x0 = int(i * step) + grout
            y0 = int(j * step) + grout
            x1 = int((i + 1) * step) - grout
            y1 = int((j + 1) * step) - grout
            draw.rectangle([x0, y0, x1, y1], fill=_jitter(base, 22))
    # A few darker scuff marks / stains for global variety.
    for _ in range(40):
        cx, cy = random.randint(0, size), random.randint(0, size)
        r = random.randint(6, 26)
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=_jitter((110, 105, 98), 15))
    return _speckle(img, amount=0.10, strength=18)


def make_brick(w=1024, h=512, cols=20, rows=14):
    img = Image.new("RGB", (w, h), (48, 40, 36))  # mortar
    draw = ImageDraw.Draw(img)
    bw, bh = w / cols, h / rows
    mortar = max(1, int(bh * 0.12))
    for r in range(rows):
        offset = (bw / 2) if (r % 2) else 0
        for c in range(-1, cols + 1):
            x0 = int(c * bw + offset) + mortar
            y0 = int(r * bh) + mortar
            x1 = int((c + 1) * bw + offset) - mortar
            y1 = int((r + 1) * bh) - mortar
            draw.rectangle([x0, y0, x1, y1], fill=_jitter((150, 74, 58), 26))
    return _speckle(img, amount=0.08, strength=16)


def make_panels(w=1024, h=512, planks=14):
    img = Image.new("RGB", (w, h), (40, 46, 52))
    draw = ImageDraw.Draw(img)
    pw = w / planks
    seam = max(1, int(pw * 0.06))
    for p in range(planks):
        x0 = int(p * pw) + seam
        x1 = int((p + 1) * pw) - seam
        draw.rectangle([x0, 0, x1, h], fill=_jitter((110, 122, 130), 20))
        # horizontal grain / rivets to add corners along the plank
        for _ in range(6):
            y = random.randint(4, h - 4)
            draw.line([x0, y, x1, y], fill=_jitter((70, 80, 88), 18), width=1)
        for _ in range(3):
            y = random.randint(10, h - 10)
            draw.ellipse([x0 + 4, y - 3, x0 + 10, y + 3], fill=(40, 44, 48))
    return _speckle(img, amount=0.06, strength=14)


def make_blocks(w=1024, h=512, cols=8, rows=6):
    img = Image.new("RGB", (w, h), (55, 55, 58))  # deep joints
    draw = ImageDraw.Draw(img)
    bw, bh = w / cols, h / rows
    joint = max(2, int(bh * 0.10))
    for r in range(rows):
        offset = (bw / 2) if (r % 2) else 0
        for c in range(-1, cols + 1):
            x0 = int(c * bw + offset) + joint
            y0 = int(r * bh) + joint
            x1 = int((c + 1) * bw + offset) - joint
            y1 = int((r + 1) * bh) - joint
            draw.rectangle([x0, y0, x1, y1], fill=_jitter((165, 160, 150), 22))
    return _speckle(img, amount=0.07, strength=15)


def _frame(draw, size, colour=(20, 20, 20), width=None):
    width = width or max(6, size // 22)
    draw.rectangle([0, 0, size - 1, size - 1], outline=colour, width=width)


def make_painting(kind, size=512):
    """Distinct high-contrast abstract 'paintings' - corner-rich landmarks."""
    palette = [
        (222, 60, 55), (60, 110, 210), (245, 200, 40),
        (40, 170, 120), (235, 130, 40), (120, 70, 190),
        (30, 30, 30), (240, 240, 235),
    ]
    img = Image.new("RGB", (size, size), (238, 236, 228))
    draw = ImageDraw.Draw(img)
    m = size // 8  # inner margin

    if kind == 1:  # Mondrian blocks - dense right-angle corners
        xs = sorted(random.sample(range(m, size - m), 3))
        ys = sorted(random.sample(range(m, size - m), 3))
        xs = [m] + xs + [size - m]
        ys = [m] + ys + [size - m]
        for i in range(len(xs) - 1):
            for j in range(len(ys) - 1):
                draw.rectangle([xs[i], ys[j], xs[i + 1], ys[j + 1]],
                               fill=random.choice(palette), outline=(20, 20, 20), width=6)
    elif kind == 2:  # diagonal stripes + triangles
        for i, x in enumerate(range(-size, size, size // 8)):
            draw.polygon([(x, m), (x + size // 8, m), (x + size // 2, size - m),
                          (x + size // 2 - size // 8, size - m)],
                         fill=palette[i % len(palette)])
    elif kind == 3:  # concentric squares
        for k, r in enumerate(range(m, size // 2, size // 16)):
            draw.rectangle([r, r, size - r, size - r],
                           outline=palette[k % len(palette)], width=8)
    elif kind == 4:  # grid of dots / checkers
        n = 6
        step = (size - 2 * m) / n
        for i in range(n):
            for j in range(n):
                cx = int(m + (i + 0.5) * step)
                cy = int(m + (j + 0.5) * step)
                r = int(step * 0.38)
                if (i + j) % 2 == 0:
                    draw.rectangle([cx - r, cy - r, cx + r, cy + r],
                                   fill=random.choice(palette))
                else:
                    draw.ellipse([cx - r, cy - r, cx + r, cy + r],
                                 fill=random.choice(palette))
    elif kind == 5:  # radial starburst
        cx = cy = size // 2
        for a in range(0, 360, 18):
            import math
            x = cx + int(math.cos(math.radians(a)) * (size // 2 - m))
            y = cy + int(math.sin(math.radians(a)) * (size // 2 - m))
            draw.polygon([(cx, cy), (x, y),
                          (cx + int(math.cos(math.radians(a + 9)) * (size // 3)),
                           cy + int(math.sin(math.radians(a + 9)) * (size // 3)))],
                         fill=palette[(a // 18) % len(palette)])
    else:  # overlapping rectangles
        for _ in range(9):
            x0 = random.randint(m, size - m - 40)
            y0 = random.randint(m, size - m - 40)
            x1 = x0 + random.randint(40, size // 2)
            y1 = y0 + random.randint(40, size // 2)
            draw.rectangle([x0, y0, x1, y1],
                           fill=random.choice(palette), outline=(20, 20, 20), width=5)

    _frame(draw, size)
    return _speckle(img, amount=0.03, strength=10)


def make_crate(size=512, planks=6):
    img = Image.new("RGB", (size, size), (74, 48, 26))
    draw = ImageDraw.Draw(img)
    pw = size / planks
    for p in range(planks):
        x0 = int(p * pw) + 4
        x1 = int((p + 1) * pw) - 4
        draw.rectangle([x0, 6, x1, size - 6], fill=_jitter((150, 96, 48), 20))
    # diagonal braces + corner bolts (strong corners)
    draw.line([8, 8, size - 8, size - 8], fill=(90, 58, 30), width=12)
    draw.line([size - 8, 8, 8, size - 8], fill=(90, 58, 30), width=12)
    draw.rectangle([6, 6, size - 6, size - 6], outline=(60, 38, 20), width=14)
    for bx in (24, size - 24):
        for by in (24, size - 24):
            draw.ellipse([bx - 9, by - 9, bx + 9, by + 9], fill=(40, 26, 14))
    return _speckle(img, amount=0.05, strength=14)


def make_pillar(size=512, rows=8):
    img = Image.new("RGB", (size, size), (70, 70, 74))
    draw = ImageDraw.Draw(img)
    bh = size / rows
    for r in range(rows):
        offset = (size / 6) if (r % 2) else 0
        for c in range(-1, 4):
            x0 = int(c * (size / 3) + offset) + 4
            y0 = int(r * bh) + 4
            x1 = int((c + 1) * (size / 3) + offset) - 4
            y1 = int((r + 1) * bh) - 4
            draw.rectangle([x0, y0, x1, y1], fill=_jitter((155, 152, 148), 18))
    return _speckle(img, amount=0.08, strength=16)


def make_barrel(size=512, bands=7):
    img = Image.new("RGB", (size, size), (36, 74, 82))
    draw = ImageDraw.Draw(img)
    bh = size / bands
    for b in range(bands):
        y0 = int(b * bh) + 3
        y1 = int((b + 1) * bh) - 3
        draw.rectangle([0, y0, size, y1], fill=_jitter((70, 140, 150), 18))
        # rivets along each band edge -> corners
        for x in range(20, size, size // 8):
            draw.ellipse([x - 6, y0 - 4, x + 6, y0 + 8], fill=(28, 58, 64))
    return _speckle(img, amount=0.05, strength=14)


def main():
    print("Generating maze VO textures ...")
    _save(make_floor(), "vo_textures", "materials", "textures", "floor_tiles.png")
    _save(make_brick(), "vo_textures", "materials", "textures", "wall_brick.png")
    _save(make_panels(), "vo_textures", "materials", "textures", "wall_panels.png")
    _save(make_blocks(), "vo_textures", "materials", "textures", "wall_blocks.png")
    for k in range(1, 7):
        _save(make_painting(k), "vo_textures", "materials", "textures", f"painting_{k}.png")
    _save(make_crate(), "crate", "materials", "textures", "crate.png")
    _save(make_pillar(), "pillar", "materials", "textures", "pillar.png")
    _save(make_barrel(), "barrel", "materials", "textures", "barrel.png")
    print("Done.")


if __name__ == "__main__":
    main()
