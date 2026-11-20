#!/usr/bin/env python3
"""Turns the raw downloads (fetch_data.py) into the files UrcSiteBuilder reads, in
rover_sim/Assets/Worlds/UrcSite/Source/:

  heights_1m.bytes    2049 x 2049 uint16, row 0 = south, 1 m lidar DEM (elevation = base + v / 65535 * range)
  soil_classes.png    2048 x 2048, one soil class per 1 m cell (see CLASSES), row 0 = north (image order)
  ground_color.jpg    4096 x 4096, the aerial photo with sun shading, shadows, shrubs and boulders taken out: ground colour
  shrubs.csv          x, z, radius of every shrub found in the aerial photo
  boulders.csv        x, z, size of boulders found in the aerial photo
  far_heights.bytes   1025 x 1025 uint16, 16 m DEM of the 16.4 km around the site (backdrop)
  far_color.jpg       Sentinel-2 colour for the backdrop, graded to match ground_color
  site.json           georeference, encoding, and colours measured in the photo (ground per soil, rock, shrub)
  (the textures and atlases in Source/Layers and Source/Props are made by make_assets.py from site.json)
  (tools/urc_site/previews/*.jpg: pictures to check the result by eye, not used by Unity)

The soil map is an interpretation, not ground truth: nobody dug holes. Rules (in order):
  sandstone   slope > 38 deg at 1 m (ledges and cliffs: the Dakota / Morrison sandstone caps)
  talus       slope 24-38 deg, or within a few metres below a ledge
  wash sand   drainage lines (flow accumulation on the DEM) with gentle gradient
  bentonite   red / purple / bleached mudstone colours without shrubs (Brushy Basin Member badlands)
  gravel      gentle convex crests (deflation lag), steeper gully beds
  hardpack    everything else: the tan shrubby flats (silty sand with a crust)

  python3 -s tools/urc_site/process_data.py <raw dir>
"""
import heapq, json, math, os, sys
import cv2
import numpy as np
from scipy import ndimage as ndi

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fetch_data as F

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(REPO, "rover_sim/Assets/Worlds/UrcSite/Source")
PRE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "previews")
CLASSES = ["Hardpack", "WashSand", "BentoniteClay", "Gravel", "RockyTalus", "Sandstone"]
HARD, SAND, BENT, GRAV, TALUS, ROCK = range(6)
BASE, RANGE = 1340.0, 100.0            # core elevation encoding (m)
FAR_BASE, FAR_RANGE = 1250.0, 400.0
H = F.CORE // 2


def slope_deg(d, cell=1.0):
    gy, gx = np.gradient(d, cell)
    return np.degrees(np.arctan(np.hypot(gx, gy))), gx, gy


def flow_accumulation(d):
    """D8 flow accumulation (cells) after a priority-flood fill of pits."""
    n, m = d.shape
    filled = d.astype(np.float64).copy()
    done = np.zeros(d.shape, bool)
    pq = []
    for i in range(n):
        for j in (0, m - 1):
            pq.append((filled[i, j], i, j)); done[i, j] = True
    for j in range(m):
        for i in (0, n - 1):
            if not done[i, j]: pq.append((filled[i, j], i, j)); done[i, j] = True
    heapq.heapify(pq)
    nb = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    order = []
    while pq:
        e, i, j = heapq.heappop(pq)
        order.append((i, j))
        for di, dj in nb:
            a, b = i + di, j + dj
            if 0 <= a < n and 0 <= b < m and not done[a, b]:
                done[a, b] = True
                if filled[a, b] <= e: filled[a, b] = e + 1e-4
                heapq.heappush(pq, (filled[a, b], a, b))
    acc = np.ones(d.shape, np.float64)
    for i, j in reversed(order):            # highest first
        best, tgt = 0.0, None
        for di, dj in nb:
            a, b = i + di, j + dj
            if 0 <= a < n and 0 <= b < m:
                drop = (filled[i, j] - filled[a, b]) / math.hypot(di, dj)
                if drop > best: best, tgt = drop, (a, b)
        if tgt: acc[tgt] += acc[i, j]
    return acc


def sun_from_image(lum, d):
    """The photo's sun direction: the azimuth / elevation whose hillshade of the DEM best matches the brightness."""
    s, gx, gy = slope_deg(d)
    hp = lum - cv2.GaussianBlur(lum, (0, 0), 25)
    m = s > 8
    best = (-1, 0, 0)
    for el in (35, 45, 55):
        for az in range(0, 360, 10):
            a, e = math.radians(az), math.radians(el)
            l = np.array([math.sin(a) * math.cos(e), math.cos(a) * math.cos(e), math.sin(e)])   # east, north, up
            sh = (-gx * l[0] - gy * l[1] + l[2]) / np.sqrt(gx * gx + gy * gy + 1)
            c = np.corrcoef(hp[m], sh[m])[0, 1]
            if c > best[0]: best = (c, az, el)
    return best


def cast_shadow(d, az, el):
    """True where the DEM itself blocks the sun (ray march towards the sun)."""
    a = math.radians(az)
    dx, dz, rise = math.sin(a), math.cos(a), math.tan(math.radians(el))
    shadow = np.zeros(d.shape, bool)
    yy, xx = np.mgrid[0:d.shape[0], 0:d.shape[1]].astype(np.float32)
    for k in range(1, 40):
        t = k * 1.5
        samp = cv2.remap(d, xx + dx * t, yy + dz * t, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        shadow |= samp > d + rise * t + 0.3
    return shadow


def grade(c):
    """Photo colour -> ground albedo (BGR, 0..1): aerial photos are pale and hazy-blue."""
    lin = c.astype(np.float32) ** 2.2
    lin *= np.array([0.84, 1.0, 1.10], np.float32)                 # warm it back up
    grey = lin.mean(axis=-1, keepdims=True)
    lin = np.clip(grey + (lin - grey) * 1.7, 0, 1) * 0.5           # dry desert soil reflects ~25-35 % of the light
    return np.clip(lin, 0, 1) ** (1 / 2.2)


def main():
    raw = sys.argv[1]
    os.makedirs(OUT + "/Layers", exist_ok=True); os.makedirs(PRE, exist_ok=True)
    x0, y0 = F.origin()

    # ---------------------------------------------------------------- elevation
    d = cv2.imread(f"{raw}/dem_core_1m.tif", cv2.IMREAD_UNCHANGED)[::-1].astype(np.float32).copy()    # row 0 = south
    assert d.shape == (F.CORE + 1, F.CORE + 1) and d.min() > BASE and d.max() < BASE + RANGE, (d.shape, d.min(), d.max())
    np.round((d - BASE) / RANGE * 65535).astype("<u2").tofile(OUT + "/heights_1m.bytes")
    far = cv2.imread(f"{raw}/dem_far_16m.tif", cv2.IMREAD_UNCHANGED)[::-1].astype(np.float32)
    far = cv2.resize(far, (1025, 1025), interpolation=cv2.INTER_LINEAR)
    np.round(np.clip((far - FAR_BASE) / FAR_RANGE, 0, 1) * 65535).astype("<u2").tofile(OUT + "/far_heights.bytes")

    # cell-centre grids (2048 x 2048, 1 m), row 0 = south
    dc = 0.25 * (d[:-1, :-1] + d[1:, :-1] + d[:-1, 1:] + d[1:, 1:])
    slope, gx, gy = slope_deg(dc)
    # slope that matters for a cliff: the steepest of the four corners' differences
    smax = ndi.maximum_filter(slope, 3)

    # ---------------------------------------------------------------- imagery (flip to row 0 = south)
    quads = [cv2.imread(f"{raw}/naip_core_{i}.png") for i in range(4)]
    rgb = np.vstack([np.hstack(quads[:2]), np.hstack(quads[2:])])[::-1].astype(np.float32) / 255      # BGR, 0.5 m
    nq = [cv2.imread(f"{raw}/naip_core_nir_{i}.png", cv2.IMREAD_UNCHANGED) for i in range(4)]
    nq = [q[..., 0] if q.ndim == 3 else q for q in nq]
    nir = np.vstack([np.hstack(nq[:2]), np.hstack(nq[2:])])[::-1].astype(np.float32) / 255
    lum = rgb.mean(axis=2)
    lum1 = cv2.resize(lum, (F.CORE, F.CORE), interpolation=cv2.INTER_AREA)

    corr, az, el = sun_from_image(lum1, dc)
    print(f"sun in the photo: azimuth {az} deg, elevation {el} deg (correlation {corr:.2f})")
    a, e = math.radians(az), math.radians(el)
    l = (math.sin(a) * math.cos(e), math.cos(a) * math.cos(e), math.sin(e))
    lambert = np.clip((-gx * l[0] - gy * l[1] + l[2]) / np.sqrt(gx * gx + gy * gy + 1), 0, 1) / l[2]   # 1 on flat ground
    shadow1 = cast_shadow(dc, az, el) | (lambert < 0.25)
    up = lambda a_, interp=cv2.INTER_LINEAR: cv2.resize(a_.astype(np.float32), (2 * F.CORE, 2 * F.CORE), interpolation=interp)
    # de-light: photo = albedo * (ambient + direct * lambert). Unity lights the terrain again.
    amb = 0.5
    shade = up(amb + (1 - amb) * lambert)
    delit = np.clip(rgb / np.clip(shade, 0.7, 1.25)[..., None], 0, 1)
    shadow = up(ndi.binary_dilation(shadow1, iterations=1)) > 0.5
    # shadows the 1 m DEM can't predict (overhangs, boulders): much darker than the surroundings and blue (skylight only)
    lbg = cv2.medianBlur(np.clip(lum * 255, 0, 255).astype(np.uint8), 51).astype(np.float32) / 255
    shadow |= ndi.binary_dilation((lum < 0.8 * lbg) & (rgb[..., 0] > 0.97 * rgb[..., 2]), iterations=2)

    # ---------------------------------------------------------------- shrubs and boulders (dark blobs, 0.5 m pixels)
    l8 = np.clip(delit.mean(axis=2) * 255, 0, 255).astype(np.uint8)
    bg = cv2.medianBlur(l8, 21).astype(np.float32)
    dark = (bg - l8.astype(np.float32)) > 13
    red = rgb[..., 2]
    ndvi = (nir - red) / (nir + red + 1e-3)
    ndvi_bg = cv2.blur(ndvi, (41, 41))
    veg = ndvi - ndvi_bg                                    # desert shrubs: only slightly "greener" than the soil
    lab, n = ndi.label(dark & ~shadow)
    idx = np.arange(1, n + 1)
    area = ndi.sum(np.ones_like(lab), lab, idx)
    cy, cx = np.array(ndi.center_of_mass(dark, lab, idx)).T
    vmean = ndi.mean(veg, lab, idx)
    depth = ndi.maximum(bg - l8.astype(np.float32), lab, idx)
    s_at = up(slope)[cy.astype(int), cx.astype(int)]
    near_cliff = up(ndi.binary_dilation(smax > 30, iterations=12))[cy.astype(int), cx.astype(int)] > 0.5
    ok = (area >= 2) & (area <= 90)
    is_boulder = ok & (vmean < -0.004) & near_cliff & (area >= 4) & (depth > 22)
    is_shrub = ok & ~is_boulder & (s_at < 26) & (area <= 40)
    wx, wz = cx * 0.5 - H + 0.25, cy * 0.5 - H + 0.25
    rad = np.sqrt(area * 0.25 / math.pi)
    with open(OUT + "/shrubs.csv", "w") as f:
        f.write("x,z,radius_m\n")
        for i in np.nonzero(is_shrub)[0]: f.write(f"{wx[i]:.2f},{wz[i]:.2f},{rad[i]:.2f}\n")
    with open(OUT + "/boulders.csv", "w") as f:
        f.write("x,z,size_m\n")
        for i in np.nonzero(is_boulder)[0]: f.write(f"{wx[i]:.2f},{wz[i]:.2f},{min(2 * rad[i], 3.5):.2f}\n")
    print(f"shrubs {is_shrub.sum()}, boulders {is_boulder.sum()}")
    blob = np.isin(lab, idx[is_shrub | is_boulder])
    shrub_density = cv2.blur(cv2.resize(np.isin(lab, idx[is_shrub]).astype(np.float32), (F.CORE, F.CORE), interpolation=cv2.INTER_AREA), (31, 31))

    # ---------------------------------------------------------------- ground colour
    hole = (ndi.binary_dilation(blob, iterations=2) | shadow).astype(np.uint8)
    img8 = np.clip(delit * 255, 0, 255).astype(np.uint8)
    # fill the holes from the ground around them (masked blur, small radius first, then wider for big shadows)
    valid = (hole == 0).astype(np.float32)
    src = img8.astype(np.float32) / 255
    ground = src.copy()
    todo = hole > 0
    for sigma in (2.5, 8, 30):
        w = cv2.GaussianBlur(valid, (0, 0), sigma)
        fill = cv2.GaussianBlur(src * valid[..., None], (0, 0), sigma) / np.maximum(w, 1e-4)[..., None]
        now = todo & (w > 0.08)
        ground[now] = fill[now]
        todo &= ~now
    ground = cv2.GaussianBlur(ground, (0, 0), 0.8)
    # the photo is pale and hazy: bring it to a plausible dry-soil albedo and give the colour back
    ground_raw = ground.copy()                                  # ungraded: the soil rules below were tuned on this
    ground = grade(ground)

    # ---------------------------------------------------------------- backdrop colour, matched to the core
    s2 = cv2.imread(f"{raw}/s2_far.png")[..., :3].astype(np.float32) / 255
    s2 = cv2.resize(s2, (2048, 2048), interpolation=cv2.INTER_CUBIC)[::-1]              # row 0 = south
    c0 = int((F.FAR / 2 - H) / F.FAR * 2048); c1 = int((F.FAR / 2 + H) / F.FAR * 2048)
    core_s2 = s2[c0:c1, c0:c1]
    # colour relative to the site's average, applied to the site's average ground colour (the Sentinel "visual" image
    # is overexposed here, so only its relative differences are usable)
    rel = s2 / core_s2.reshape(-1, 3).mean(axis=0)
    lum_rel = rel.mean(axis=2, keepdims=True)
    far_col = np.clip(ground.reshape(-1, 3).mean(axis=0) * lum_rel ** 1.1 * (rel / lum_rel) ** 0.55, 0, 1).astype(np.float32)   # hue differences at half strength
    cv2.imwrite(OUT + "/far_color.jpg", (far_col[::-1] * 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 90])
    # fade the site's edge into the backdrop so the border doesn't show as a line
    edge = np.minimum.reduce(np.meshgrid(np.minimum(np.arange(4096), 4095 - np.arange(4096)), np.minimum(np.arange(4096), 4095 - np.arange(4096)))) * 0.5
    wgt = np.clip(edge / 150.0, 0, 1)[..., None] ** 0.7
    far_up = cv2.resize(far_col[c0:c1, c0:c1], (4096, 4096), interpolation=cv2.INTER_CUBIC)
    ground = (ground * wgt + far_up * (1 - wgt)).astype(np.float32)

    cv2.imwrite(OUT + "/ground_color.jpg", (ground[::-1] * 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 90])

    # ---------------------------------------------------------------- soil classes (1 m)
    g1 = cv2.resize(ground_raw, (F.CORE, F.CORE), interpolation=cv2.INTER_AREA)
    labc = cv2.cvtColor(cv2.GaussianBlur(g1, (0, 0), 1.5), cv2.COLOR_BGR2Lab)       # float input: L 0..100, a/b signed
    L, A, B = labc[..., 0], labc[..., 1], labc[..., 2]
    Lr, Ar, Br = (np.median(v) for v in (L, A, B))
    print(f"typical ground Lab: {Lr:.1f} {Ar:.1f} {Br:.1f}; a* 5/50/95 %: {np.percentile(A, [5, 50, 95]).round(1)}, L 5/50/95: {np.percentile(L, [5, 50, 95]).round(1)}, "
          f"b*: {np.percentile(B, [5, 50, 95]).round(1)}")
    reddish = (A - Ar > 2.2) | ((B - Br < -4.0) & (A - Ar > 0.5))       # red / purple mudstone
    bleached = (L - Lr > 6.5) & (B - Br < -1.5)                         # white ash / bleached clay
    bare = shrub_density < 0.012
    # shrub-free ground at MDRS is the clay: sandy ground carries shrubs. Bare + coloured, or bare + sloping (clay hills).
    bent = ndi.binary_opening((reddish | bleached | (slope > 5)) & bare, iterations=2)
    bent = ndi.binary_closing(bent, iterations=3)

    d2 = cv2.resize(d, (H + 1, H + 1), interpolation=cv2.INTER_AREA)
    acc = cv2.resize(flow_accumulation(cv2.GaussianBlur(d2, (0, 0), 0.7)).astype(np.float32) * 4, (F.CORE, F.CORE), interpolation=cv2.INTER_NEAREST)   # m^2
    wash = np.zeros(dc.shape, bool)
    for area_m2, r in ((2500, 1), (10000, 2), (40000, 3), (150000, 5)):
        wash |= ndi.binary_dilation(acc > area_m2, iterations=r)
    wash_sand = wash & (slope < 7)
    gully = wash & (slope >= 7) & (slope < 24)

    tpi = dc - cv2.blur(dc, (41, 41))
    rock = smax > 38
    talus = ((slope > 24) | ndi.binary_dilation(rock, iterations=4)) & ~rock
    crest = (tpi > 0.8) & (slope < 12)
    cls = np.full(dc.shape, HARD, np.uint8)
    cls[crest | gully] = GRAV
    cls[bent] = BENT
    cls[wash_sand] = SAND
    cls[talus] = TALUS
    cls[rock] = ROCK
    cls = ndi.median_filter(cls, 3)
    cv2.imwrite(OUT + "/soil_classes.png", cls[::-1])
    frac = {CLASSES[k]: round(float((cls == k).mean()), 3) for k in range(6)}
    print("soil classes (fraction of the map):", frac)

    # ---------------------------------------------------------------- real colours for the generated assets
    gg = cv2.resize(grade(ground_raw), (F.CORE, F.CORE), interpolation=cv2.INTER_AREA)
    rgb01 = lambda bgr: [round(float(v), 3) for v in bgr[::-1]]
    class_colors = {CLASSES[k]: rgb01(gg[cls == k].mean(axis=0)) for k in range(6)}
    rocky = gg[(cls == ROCK) | (cls == TALUS)]
    rocky = rocky[np.random.default_rng(1).choice(len(rocky), 60000, replace=False)].astype(np.float32)
    _, lbl, centers = cv2.kmeans(rocky, 4, None, (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4), 3, cv2.KMEANS_PP_CENTERS)
    order = np.argsort(-np.bincount(lbl.ravel(), minlength=4))
    rock_colors = [rgb01(centers[i]) for i in order]
    rock_share = [round(float((lbl == i).mean()), 2) for i in order]
    shrub_px = np.isin(lab, idx[is_shrub & (area >= 6)])
    shrub_color = rgb01(np.median(grade(delit[shrub_px]), axis=0))
    print("ground colour by soil:", class_colors, "\nrock colours:", rock_colors, rock_share, "\nshrub colour:", shrub_color)

    # ---------------------------------------------------------------- metadata + previews
    json.dump({
        "source": "USGS 3DEP 1 m lidar (UT_WestEast_B22 / UT Southern QL1), USDA NAIP 2021-09-15 0.6 m, Sentinel-2 2025-07-05 (backdrop)",
        "rules": "URC 2027 1.e.xiii: square mile between (38.411N, -110.786E) and (38.425N, -110.768E)",
        "origin_lat": F.LAT0, "origin_lon": F.LON0, "origin_utm12n_e": x0, "origin_utm12n_n": y0,
        "core_size_m": F.CORE, "core_samples": F.CORE + 1, "elev_base_m": BASE, "elev_range_m": RANGE,
        "far_size_m": F.FAR, "far_samples": 1025, "far_base_m": FAR_BASE, "far_range_m": FAR_RANGE,
        "classes": CLASSES, "photo_sun_azimuth_deg": az, "photo_sun_elevation_deg": el,
        # colours measured in the aerial photo (sRGB 0..1, after the same grading as ground_color.jpg); make_assets.py uses them
        "ground_colors": class_colors, "rock_colors": rock_colors, "rock_color_share": rock_share, "shrub_color": shrub_color,
    }, open(OUT + "/site.json", "w"), indent=1)
    pal = np.array([(150, 180, 200), (190, 225, 240), (120, 110, 190), (120, 130, 130), (70, 90, 110), (40, 40, 60)], np.uint8)
    cv2.imwrite(PRE + "/preview_soil.jpg", cv2.resize(pal[cls][::-1], (1400, 1400), interpolation=cv2.INTER_NEAREST))
    pv = (ground[::-1] * 255).astype(np.uint8)
    cv2.imwrite(PRE + "/preview_ground.jpg", cv2.resize(pv, (1400, 1400), interpolation=cv2.INTER_AREA))
    # detail crop around the task area with the detections drawn in
    c = (rgb * 255).astype(np.uint8).copy()
    for i in np.nonzero(is_shrub)[0]: cv2.circle(c, (int(cx[i]), int(cy[i])), 3, (0, 220, 0), 1)
    for i in np.nonzero(is_boulder)[0]: cv2.circle(c, (int(cx[i]), int(cy[i])), 4, (0, 0, 255), 1)
    crop = c[(250 + H) * 2:(550 + H) * 2, (-450 + H) * 2:(-100 + H) * 2][::-1]
    cv2.imwrite(PRE + "/preview_detections.jpg", crop)


if __name__ == "__main__":
    main()
