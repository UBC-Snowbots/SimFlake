#!/usr/bin/env python3
"""Makes the textures of the URC site world (rover_sim/Assets/Worlds/UrcSite/Source/Layers and /Props). No downloads:
it only needs Source/site.json (written by process_data.py). Everything here is drawn by code, so we own it.

  Layers/<soil>_Detail.jpg, _Normal.jpg   close-up ground texture per soil class, 1024 px, seamless. Detail is brightness
                                          around mid grey: the colour comes from the aerial photo (ground_color.jpg).
  Props/Rock_Albedo.jpg, Rock_Normal.jpg  bedded sandstone for the generated boulders (grey: tinted per boulder)
  Props/ShrubAtlas.png                    4 x 2 cut-out cards for the generated shrubs (sprigs, stems, grass, twigs)
  Props/palette.csv                       rock colours measured in the aerial photo (from site.json), for the builder

What is real and what is not: the aerial photo has 0.6 m pixels, so it gives the true COLOUR of the ground and of the
rock, but nothing about how a square metre looks close up. The patterns here are modelled on what these surfaces look
like at MDRS: popcorn-crusted bentonite, crusted silt with a few pebbles, rippled wash sand, lag gravel, angular talus,
bedded sandstone. Shrub colours are those of the species that grow there (shadscale, mat saltbush, Mormon tea, galleta
grass): a 0.6 m pixel is half shrub, half soil, so the photo can't give them.

  python3 -s tools/urc_site/make_assets.py [shrubs]      then rebuild the world (UrcSiteBuilder)
"""
import json, math, os
import cv2
import numpy as np
from scipy import ndimage as ndi
from scipy.spatial import cKDTree

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(REPO, "rover_sim/Assets/Worlds/UrcSite/Source")
PRE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "previews")
N = 1024

# ------------------------------------------------------------------ seamless building blocks


def fnoise(seed, beta=2.0, lo=1, hi=None, n=N):
    """Periodic fractal noise (power spectrum ~ 1/f^beta between spatial frequencies lo..hi cycles per tile), std 1."""
    rng = np.random.default_rng(seed)
    f = np.fft.fftfreq(n) * n
    r = np.hypot(*np.meshgrid(f, f))
    amp = np.where(r >= lo, np.maximum(r, 1) ** (-beta / 2), 0)
    if hi: amp = amp * (r <= hi)
    x = np.fft.ifft2(amp * np.exp(2j * np.pi * rng.random((n, n)))).real
    return ((x - x.mean()) / x.std()).astype(np.float32)


def voronoi(seed, count, n=N):
    """Periodic Voronoi cells: distance to the nearest and second nearest point (px), cell id, offset from the cell centre."""
    rng = np.random.default_rng(seed)
    p = rng.random((count, 2)) * n
    tiled = np.concatenate([p + (dx * n, dy * n) for dx in (-1, 0, 1) for dy in (-1, 0, 1)])
    yy, xx = np.mgrid[0:n, 0:n]
    d, i = cKDTree(tiled).query(np.stack([xx.ravel(), yy.ravel()], 1), k=2)
    f1, f2 = d[:, 0].reshape(n, n).astype(np.float32), d[:, 1].reshape(n, n).astype(np.float32)
    near = tiled[i[:, 0]]
    off = (np.stack([xx.ravel(), yy.ravel()], 1) - near).reshape(n, n, 2).astype(np.float32)
    return f1, f2, (i[:, 0] % count).reshape(n, n), off


def blur(a, sigma):
    return ndi.gaussian_filter(a, sigma, mode="wrap")


def normal_map(h, strength):
    """Height (px units) -> Unity tangent-space normal map (BGR image, +v is up in the texture)."""
    dx = (np.roll(h, -1, 1) - np.roll(h, 1, 1)) * 0.5 * strength
    dy = (np.roll(h, -1, 0) - np.roll(h, 1, 0)) * 0.5 * strength      # along image rows (down)
    inv = 1 / np.sqrt(dx * dx + dy * dy + 1)
    return np.stack([inv * 0.5 + 0.5, dy * inv * 0.5 + 0.5, -dx * inv * 0.5 + 0.5], -1)      # B = z, G = y, R = x


def detail(h, tone=None, cavity=0.5, relief=0.25, contrast=1.0, seed=0, hue=0.012):
    """Brightness texture around mid grey from a height field: high = a little lighter, hollows and cracks darker,
    'tone' = per-stone lightness. A trace of warm / cool variation so it isn't a flat grey multiply."""
    cav = np.clip((h - blur(h, 6)) / (np.std(h - blur(h, 6)) + 1e-6), -3, 3)
    v = relief * (h - h.mean()) / (h.std() + 1e-6) + cavity * np.minimum(cav, 0.5)
    if tone is not None: v = v + tone
    v = 0.5 + 0.11 * contrast * (v - v.mean()) / (v.std() + 1e-6)
    v = np.clip(v, 0.18, 0.88)
    v = v + (0.5 - v.mean())
    warm = hue * blur(fnoise(seed + 77, 2.5, 2, 40), 2)
    return np.clip(np.stack([v - warm, v, v + warm], -1), 0, 1)       # BGR


def save(name, img, quality=92):
    cv2.imwrite(os.path.join(SRC, name), np.clip(img * 255 + 0.5, 0, 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, quality])


def per_cell(ids, count, seed, lo=0.0, hi=1.0):
    return (np.random.default_rng(seed).random(count) * (hi - lo) + lo).astype(np.float32)[ids]

# ------------------------------------------------------------------ the six soils (height in px; tile size in comments)


def hardpack():
    """Crusted silt, 2 m tile (2 mm / px): smooth crust, hairline polygon cracks, a sprinkle of 1-2 cm pebbles."""
    h = 1.2 * fnoise(1, 2.4, 2, 60) + 0.5 * fnoise(2, 1.2, 60, 400)
    f1, f2, _, _ = voronoi(3, 70)
    wob = 10 * fnoise(8, 2.6, 3, 40)                                              # cracks wander: no ruler-straight lines
    h -= 0.9 * np.exp(-((f2 - f1 + 0.25 * wob) / 2.2) ** 2) * np.clip(0.5 + 0.5 * fnoise(9, 2.5, 1, 6), 0, 1)   # faint desiccation cracks, in patches
    f1, f2, ids, _ = voronoi(4, 2600)
    stone = per_cell(ids, 2600, 5) < 0.09
    r = per_cell(ids, 2600, 6, 3.5, 9)
    peb = np.where(stone, np.sqrt(np.clip(1 - (f1 / r) ** 2, 0, 1)) * r * 0.55, 0)
    tone = np.where(stone & (f1 < r), per_cell(ids, 2600, 7, -1.6, 1.0), 0)
    return h + peb, tone, 1.1


def wash_sand():
    """Loose wash sand, 3 m tile: wind / water ripples ~9 cm apart, grain, the odd granule."""
    warp = 14 * fnoise(11, 2.6, 1, 12)
    yy, xx = np.mgrid[0:N, 0:N].astype(np.float32)
    k = 2 * math.pi * 34 / N                                                     # 34 ripples per tile
    ph = k * (yy + 0.18 * xx * 0 + warp)
    rip = np.sin(ph) + 0.25 * np.sin(2 * ph + 0.6)                               # sharper crests
    amp = 0.75 + 0.25 * np.clip(fnoise(12, 2.5, 1, 8), -2, 2) * 0.5
    h = 2.2 * rip * amp + 0.35 * fnoise(13, 0.6, 120, 500) + 0.8 * fnoise(14, 2.2, 2, 30)
    return h, None, 0.8


def bentonite():
    """Dry bentonite 'popcorn' crust, 2 m tile: 2-4 cm puffed clods with deep gaps, over wider shrinkage polygons."""
    f1, f2, ids, _ = voronoi(21, 5200)
    r = 0.62 * (f1 + f2)                                                         # ~ local cell size
    clod = np.sqrt(np.clip(1 - (f1 / np.maximum(r, 1)) ** 2, 0, 1)) * per_cell(ids, 5200, 22, 4.0, 8.0)
    gap = np.exp(-((f2 - f1) / 1.8) ** 2)
    g1, g2, _, _ = voronoi(23, 45)
    h = clod - 3.0 * gap - 5.0 * np.exp(-((g2 - g1) / 3.0) ** 2) + 2.5 * fnoise(24, 2.4, 2, 30) + 0.5 * fnoise(25, 1.0, 100, 500)
    return h, per_cell(ids, 5200, 26, -0.5, 0.5), 1.25


def gravel():
    """Lag gravel / desert pavement, 2 m tile: 3-8 cm stones of mixed rock, close packed, grit between."""
    f1, f2, ids, _ = voronoi(31, 1100)
    is_stone = per_cell(ids, 1100, 32) < 0.72
    r = 0.5 * (f1 + f2) * per_cell(ids, 1100, 33, 0.7, 0.95)
    st = np.where(is_stone, np.clip(1 - (f1 / np.maximum(r, 1)) ** 2.6, 0, 1) ** 0.6 * per_cell(ids, 1100, 34, 5, 11), 0)
    g1, g2, gid, _ = voronoi(35, 9000)
    grit = np.sqrt(np.clip(1 - (g1 / 4.5) ** 2, 0, 1)) * 2.2
    h = np.maximum(st, grit) + 1.5 * fnoise(36, 2.2, 2, 40)
    tone = np.where(is_stone & (f1 < r), per_cell(ids, 1100, 37, -2.2, 1.6), per_cell(gid, 9000, 38, -0.5, 0.5))
    return h, tone, 1.3


def talus():
    """Talus, 3 m tile: angular sandstone pieces 10-30 cm, tilted plates with sharp edges, rubble and soil between."""
    f1, f2, ids, off = voronoi(41, 300)
    is_block = per_cell(ids, 300, 42) < 0.6
    edge = np.clip((f2 - f1) / 7.0, 0, 1)                                        # flat top, sharp sides
    tilt = (off[..., 0] * per_cell(ids, 300, 43, -0.22, 0.22) + off[..., 1] * per_cell(ids, 300, 44, -0.22, 0.22))
    block = np.where(is_block, edge * (per_cell(ids, 300, 45, 9, 22) + tilt), 0)
    g1, g2, gid, goff = voronoi(46, 3800)
    small = np.clip((g2 - g1) / 3.0, 0, 1) * (per_cell(gid, 3800, 47, 1.5, 6) + goff[..., 0] * per_cell(gid, 3800, 48, -0.3, 0.3))
    small = np.where(per_cell(gid, 3800, 49) < 0.55, small, 0)
    h = np.maximum(block, small) + 2.5 * fnoise(50, 2.3, 2, 30) + 0.4 * fnoise(51, 1.0, 100, 500)
    tone = np.where(is_block & (edge > 0.2), per_cell(ids, 300, 52, -1.3, 1.3), per_cell(gid, 3800, 53, -0.6, 0.6))
    return h, tone, 1.35


def sandstone(seed=61, sets=5):
    """Bedded sandstone, 6 m tile (terrain) / 2 m (boulders): thin laminae in cross-bedded sets, joints, pits."""
    yy, xx = np.mgrid[0:N, 0:N].astype(np.float32)
    s1, s2, sid, _ = voronoi(seed, sets)                                         # cross-bedding sets: each dips its own way
    dip = per_cell(sid, sets, seed + 1, -2.0, 2.0)
    dip = blur(np.round(dip), 10)                                                # whole numbers keep the lines seamless
    warp = 9 * fnoise(seed + 2, 2.8, 1, 10)
    ph = 2 * math.pi * (46 * yy + dip * 6 * xx) / N + 0.35 * warp
    lam = np.sin(ph) * 0.6 + np.sin(2.3 * ph + 1.0) * 0.3 + np.sign(np.sin(0.21 * ph)) * 0.5        # thin laminae + thicker beds
    hard = blur(fnoise(seed + 3, 2.0, 1, 6), 3)                                  # harder beds stand out
    h = 2.6 * lam * (0.7 + 0.3 * hard) + 3.0 * fnoise(seed + 4, 2.4, 1, 40) + 0.6 * fnoise(seed + 5, 1.0, 100, 500)
    j1, j2, _, _ = voronoi(seed + 6, 9)
    h -= 7.0 * np.exp(-((j2 - j1) / 2.5) ** 2)                                   # joints
    p1, _, pid, _ = voronoi(seed + 7, 700)
    pit = np.where(per_cell(pid, 700, seed + 8) < 0.25, np.clip(1 - (p1 / per_cell(pid, 700, seed + 9, 3, 9)) ** 2, 0, 1), 0)
    h -= 5.0 * pit                                                               # weathering pits (small tafoni)
    tone = 0.9 * np.sin(0.21 * ph + 0.5) * 0.5 + 0.7 * hard
    return h, tone, 1.2


LAYERS = {"Hardpack": (hardpack, 1.0), "WashSand": (wash_sand, 0.75), "BentoniteClay": (bentonite, 1.15), "Gravel": (gravel, 1.2),
          "RockyTalus": (talus, 1.25), "Sandstone": (sandstone, 1.1)}

# ------------------------------------------------------------------ shrub cards

CELL, SS = 256, 4       # cell size in the atlas, supersampling while drawing


class Card:
    """One cut-out card, drawn 4x too big then shrunk. The plant grows from the bottom centre."""
    def __init__(self, seed):
        self.rng = np.random.default_rng(seed)
        self.s = CELL * SS
        self.rgb = np.zeros((self.s, self.s, 3), np.float32)
        self.a = np.zeros((self.s, self.s), np.float32)

    def paint(self, draw, lo, hi, color):
        """Draw a shape (draw(mask, offset)) inside the box lo..hi only: whole-card masks made this take minutes."""
        x0, y0 = max(0, int(min(lo[0], hi[0])) - 8), max(0, int(min(lo[1], hi[1])) - 8)
        x1, y1 = min(self.s, int(max(lo[0], hi[0])) + 9), min(self.s, int(max(lo[1], hi[1])) + 9)
        if x1 <= x0 or y1 <= y0: return
        m = np.zeros((y1 - y0, x1 - x0), np.uint8)
        draw(m, (x0, y0))
        m = m.astype(np.float32)[..., None] / 255
        c = np.array(color[::-1], np.float32) * (1 + self.rng.uniform(-0.09, 0.09))
        self.rgb[y0:y1, x0:x1] = self.rgb[y0:y1, x0:x1] * (1 - m) + c * m
        self.a[y0:y1, x0:x1] = np.maximum(self.a[y0:y1, x0:x1], m[..., 0])

    def line(self, p, q, w, color):
        w = max(1, int(w))
        self.paint(lambda m, o: cv2.line(m, (int(p[0]) - o[0], int(p[1]) - o[1]), (int(q[0]) - o[0], int(q[1]) - o[1]), 255, w, cv2.LINE_AA),
                   (min(p[0], q[0]) - w, min(p[1], q[1]) - w), (max(p[0], q[0]) + w, max(p[1], q[1]) + w), color)

    def leaf(self, p, length, width, ang, color):
        c = (int(p[0] + 0.5 * length * math.cos(ang)), int(p[1] + 0.5 * length * math.sin(ang)))
        self.paint(lambda m, o: cv2.ellipse(m, (c[0] - o[0], c[1] - o[1]), (max(1, int(length / 2)), max(1, int(width / 2))), math.degrees(ang), 0, 360, 255, -1, cv2.LINE_AA),
                   (c[0] - length, c[1] - length), (c[0] + length, c[1] + length), color)

    def twig(self, p, ang, length, w, depth, wood, leaf=None, leaf_len=0, leaf_w=0, every=0, fork=0.5, spread=0.6):
        """A twig that bends, forks and (optionally) carries leaves on both sides."""
        steps = max(2, int(length / (10 * SS)))
        for k in range(steps):
            ang += self.rng.normal(0, 0.10)
            q = (p[0] + math.cos(ang) * length / steps, p[1] + math.sin(ang) * length / steps)
            self.line(p, q, w * (1 - 0.5 * k / steps), wood)
            if leaf is not None and every and k % every == 0 and (depth < 2 or k > 0):
                for side in (-1, 1):
                    la = ang + side * self.rng.uniform(0.5, 1.2)
                    self.leaf(q, leaf_len * self.rng.uniform(0.7, 1.2), leaf_w * self.rng.uniform(0.8, 1.2), la, leaf)
            if depth > 0 and k > 0 and self.rng.random() < fork:
                self.twig(q, ang + self.rng.choice([-1, 1]) * self.rng.uniform(0.35, spread + 0.35), length * (1 - k / steps) * self.rng.uniform(0.6, 0.9) + 6 * SS,
                          w * 0.7, depth - 1, wood, leaf, leaf_len, leaf_w, every, fork, spread)
            p = q
        if leaf is not None: self.leaf(p, leaf_len, leaf_w, ang, leaf)

    def done(self, mound=True):
        rgb = cv2.resize(self.rgb, (CELL, CELL), interpolation=cv2.INTER_AREA)
        a = cv2.resize(self.a, (CELL, CELL), interpolation=cv2.INTER_AREA)
        if mound:
            # cut the card to a ragged mound: a plant's outline is a lumpy dome, not the square it was drawn in
            yy, xx = np.mgrid[0:CELL, 0:CELL].astype(np.float32)
            u, v = (xx / CELL - 0.5) / 0.48, (1 - yy / CELL) / 0.95
            th = np.arctan2(v, u)
            lim = 1 + sum(self.rng.uniform(0.03, 0.08) * np.sin(k * th + self.rng.uniform(0, 6.28)) for k in (3, 5, 8, 13)) - 0.06
            a = a * np.clip((lim - np.hypot(u, v)) / 0.05, 0, 1)
            rgb = rgb * np.clip((lim - np.hypot(u, v)) / 0.05, 0, 1)[..., None]
        col = rgb / np.maximum(a, 1e-4)[..., None]
        # colour under the transparent pixels = the card's own colour, so mip maps don't get dark fringes
        mean = (col * a[..., None]).sum(axis=(0, 1)) / max(a.sum(), 1e-3)
        w = np.clip(a * 6, 0, 1)
        wb = cv2.GaussianBlur(w, (0, 0), 3)
        near = cv2.GaussianBlur(col * w[..., None], (0, 0), 3) / np.maximum(wb, 1e-3)[..., None]
        fill = np.where(wb[..., None] > 0.02, near, mean)
        col = col * w[..., None] + fill * (1 - w[..., None])
        return np.dstack([col, np.clip(a * 1.15, 0, 1)])


def shrub_atlas():
    S = CELL * SS
    base = (S / 2, S * 0.99)
    up = -math.pi / 2
    wood, dead = (0.40, 0.33, 0.27), (0.56, 0.50, 0.40)
    cards = []
    # 0 shadscale sprig: stiff spiny twigs, small round grey-green leaves
    c = Card(1)
    for a in (-0.55, -0.2, 0.15, 0.5): c.twig(base, up + a, S * 0.78, 3.2 * SS, 3, wood, (0.50, 0.55, 0.44), 9 * SS, 6.5 * SS, 1, 0.55)
    cards.append(c.done())
    # 1 mat saltbush: dense pale grey scale-like leaves
    c = Card(2)
    for a in np.linspace(-0.9, 0.9, 7): c.twig(base, up + a, S * 0.62, 2.5 * SS, 3, (0.48, 0.44, 0.38), (0.62, 0.64, 0.56), 7 * SS, 5.5 * SS, 1, 0.7, 0.8)
    cards.append(c.done())
    # 2 Mormon tea (ephedra): leafless jointed green stems, broom-like
    c = Card(3)
    for a in np.linspace(-0.5, 0.5, 9): c.twig(base, up + a + c.rng.normal(0, 0.05), S * 0.9, 2.6 * SS, 2, (0.43, 0.47, 0.31), None, fork=0.45, spread=0.15)
    cards.append(c.done())
    # 3 bunch grass (galleta / ricegrass): straw blades
    c = Card(4)
    for _ in range(46):
        a = up + c.rng.normal(0, 0.33)
        p = (base[0] + c.rng.normal(0, 6 * SS), base[1])
        L = S * c.rng.uniform(0.45, 0.95)
        for k in range(6):
            q = (p[0] + math.cos(a) * L / 6, p[1] + math.sin(a) * L / 6)
            c.line(p, q, (1.9 - 0.22 * k) * SS, (0.70, 0.62, 0.42) if c.rng.random() < 0.8 else (0.58, 0.56, 0.38))
            a += c.rng.normal(0.0, 0.05) + 0.05 * np.sign(a - up); p = q
    cards.append(c.done())
    # 4 bare woody twigs (dormant / dead shrub)
    c = Card(5)
    for a in (-0.6, -0.25, 0.1, 0.45): c.twig(base, up + a, S * 0.8, 5.0 * SS, 4, (0.50, 0.44, 0.35), None, fork=0.6, spread=0.7)
    cards.append(c.done())
    # 5 bark: opaque strip for the main branches
    c = Card(6)
    c.rgb[:] = wood[::-1]; c.a[:] = 1
    for _ in range(90):
        x = c.rng.uniform(0, S); c.line((x, 0), (x + c.rng.normal(0, 20), S), c.rng.uniform(1, 4) * SS, (0.33, 0.27, 0.22) if c.rng.random() < 0.5 else (0.50, 0.44, 0.37))
    cards.append(c.done(mound=False))
    # 6 half-dead sprig: mostly straw twigs, a few leaves left
    c = Card(7)
    for a in (-0.5, -0.1, 0.35): c.twig(base, up + a, S * 0.75, 3.0 * SS, 3, dead, (0.58, 0.57, 0.44), 8 * SS, 5.5 * SS, 3, 0.6)
    cards.append(c.done())
    # 7 shadscale again, darker and denser (variety)
    c = Card(8)
    for a in (-0.7, -0.35, 0.0, 0.3, 0.65): c.twig(base, up + a, S * 0.7, 3.0 * SS, 3, wood, (0.44, 0.50, 0.40), 9 * SS, 7 * SS, 1, 0.65)
    cards.append(c.done())
    atlas = np.vstack([np.hstack(cards[:4]), np.hstack(cards[4:])])
    cv2.imwrite(os.path.join(SRC, "Props/ShrubAtlas.png"), np.clip(atlas * 255 + 0.5, 0, 255).astype(np.uint8))
    bg = np.full(atlas.shape[:2] + (3,), (0.40, 0.53, 0.62), np.float32)
    cv2.imwrite(os.path.join(PRE, "assets_shrub_cards.jpg"), ((atlas[..., :3] * atlas[..., 3:] + bg * (1 - atlas[..., 3:])) * 255).astype(np.uint8))


def main():
    os.makedirs(SRC + "/Layers", exist_ok=True); os.makedirs(SRC + "/Props", exist_ok=True); os.makedirs(PRE, exist_ok=True)
    site = json.load(open(SRC + "/site.json"))
    import sys
    if 'shrubs' in sys.argv[1:]: return shrub_atlas()        # just the shrub cards (the ground textures take a minute)
    sheet = []
    for name, (fn, contrast) in LAYERS.items():
        h, tone, strength = fn()
        d = detail(h, tone, contrast=contrast, seed=len(sheet))
        save(f"Layers/{name}_Detail.jpg", d)
        save(f"Layers/{name}_Normal.jpg", normal_map(h, 0.55 * strength), 95)
        # preview: the detail as it shows on the ground = detail * 2 * that soil's real colour, lit from the side
        col = np.array(site["ground_colors"][name][::-1], np.float32)
        nm = normal_map(h, 0.55 * strength) * 2 - 1
        light = np.clip(nm[..., 0] * 0.75 + nm[..., 2] * -0.45 + nm[..., 1] * 0.45, 0, 1) / 0.75
        pv = np.clip(d * 2 * col * (0.35 + 0.65 * light[..., None]), 0, 1)[:512, :512]
        cv2.putText(pv, name, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (1, 1, 1), 2)
        sheet.append(pv)
        print(name, "detail mean %.3f std %.3f" % (d.mean(), d.std()))
    # boulders: the same bedded sandstone at boulder scale, light grey so the material colour (from the photo) tints it
    h, tone, strength = sandstone(seed=91, sets=3)
    d = detail(h, tone, contrast=1.5, seed=9, hue=0.015)
    rock = np.clip(0.76 + (d - 0.5) * 1.1, 0, 1)
    save("Props/Rock_Albedo.jpg", rock)
    save("Props/Rock_Normal.jpg", normal_map(h, 0.9), 95)
    with open(SRC + "/Props/palette.csv", "w") as f:
        f.write("# rock colours measured in the aerial photo on sandstone ledges and talus (k-means, sRGB 0..1) and their share\nkind,r,g,b,share\n")
        for c, s in zip(site["rock_colors"], site["rock_color_share"]): f.write("rock,%.3f,%.3f,%.3f,%.2f\n" % (c[0], c[1], c[2], s))
    pv = np.clip(rock * np.array(site["rock_colors"][0][::-1], np.float32) / 0.76, 0, 1)[:512, :512]
    cv2.putText(pv, "Boulder rock", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (1, 1, 1), 2)
    sheet.append(pv)
    sheet.append(np.zeros_like(pv))
    cv2.imwrite(os.path.join(PRE, "assets_ground_textures.jpg"), (np.vstack([np.hstack(sheet[:4]), np.hstack(sheet[4:])]) * 255).astype(np.uint8))
    shrub_atlas()


if __name__ == "__main__":
    main()
