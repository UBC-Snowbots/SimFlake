#!/usr/bin/env python3
"""
Makes the desert-tinted terrain albedo textures for the Badlands traversal world.

The source textures (TerrainSampleAssets) are greyish/green. This regrades them toward Utah / Alberta badlands
colours, downsizes to 1024 px and saves JPGs to rover_sim/Assets/Worlds/BadlandsTextures. The normal maps are used
straight from TerrainSampleAssets, so they aren't copied. Only needed if you change the palette; the output is
committed. Requires Pillow + numpy:  python3 tools/make_badlands_textures.py
"""
import os
import numpy as np
from PIL import Image

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rover_sim", "Assets")
SRC = os.path.join(ROOT, "ThirdParty", "TerrainSampleAssets", "Textures", "Terrain")
DST = os.path.join(ROOT, "Worlds", "BadlandsTextures")

# output name: (source texture, target mean RGB, contrast)
LAYERS = {
    "DesertSoil": ("Pebbles_B", (160, 121, 90), 1.25),  # crusty red soil with grit
    "Pebbles":    ("Pebbles_A", (124, 96, 76), 1.2),
    "Sandstone":  ("Rock", (160, 100, 72), 1.1),       # steep slopes
    "WashSand":   ("Sand", (196, 170, 134), 1.6),      # wash bed, dunes
    "DryGrass":   ("Grass_Dry", (138, 118, 78), 1.0),
    "GreyClay":   ("Rock", (160, 150, 136), 0.8),      # badlands bentonite (cracked)
    "RockySoil":  ("Soil_Rocks", (134, 104, 82), 1.1),
    "Gravel":     ("Pebbles_C", (128, 124, 116), 1.0),  # station pad, track
}


def main():
    os.makedirs(DST, exist_ok=True)
    for name, (src, target, contrast) in LAYERS.items():
        im = Image.open(os.path.join(SRC, src + "_BaseColor.tif")).convert("RGB").resize((1024, 1024), Image.LANCZOS)
        a = np.asarray(im).astype(np.float32)
        # desaturate a little first so the hue comes from the target colour, then regrade per channel
        lum = a.mean(axis=2, keepdims=True)
        a = 0.45 * a + 0.55 * lum
        mean = a.reshape(-1, 3).mean(0)
        a = (a - mean) * contrast + mean
        a = a * (np.array(target, np.float32) / mean)
        out = Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))
        path = os.path.join(DST, name + "_Albedo.jpg")
        out.save(path, quality=90)
        print(f"{path}: mean {np.asarray(out).reshape(-1, 3).mean(0).astype(int)}")


if __name__ == "__main__":
    main()
