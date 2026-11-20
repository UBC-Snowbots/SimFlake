#!/usr/bin/env python3
"""Downloads the real-world source data for the URC autonomy world (Scenes/TaskWorlds/UrcAutonomySite).

The URC 2027 rules (1.e.xiii) put the route-finding targets in the Utah state-owned square mile between
(38.411N, -110.786E) and (38.425N, -110.768E), just north of MDRS. Everything here is public US data:
  - USGS 3DEP elevation (1 m lidar where flown), ImageServer exportImage
  - USDA NAIP aerial imagery (2021-09-15, 0.6 m) and Sentinel-2 (ESA Copernicus, 2025-07-05, 10 m) for the
    backdrop, both through Microsoft Planetary Computer's crop API (the USGS NAIP server timed out)
Grid: UTM zone 12N (EPSG:26912), so 1 px = whole metres and Unity x = east, z = north.

  python3 tools/urc_site/fetch_data.py <out dir>       (raw downloads are not committed; process_data.py makes
                                                         the small files the Unity builder reads)
"""
import math, os, sys, urllib.parse, urllib.request

LAT0, LON0 = 38.418, -110.777            # centre of the rules' square mile
CORE = 2048                              # metres, drivable area (the square mile + ~240 m margin)
FAR = 16384                              # metres, backdrop (Factory Butte, Skyline Rim, the Henry Mtns foothills)

def utm12(lat, lon):
    a, f, k0, lon0 = 6378137.0, 1 / 298.257222101, 0.9996, math.radians(-111)
    e2 = f * (2 - f); ep2 = e2 / (1 - e2)
    p, l = math.radians(lat), math.radians(lon)
    N = a / math.sqrt(1 - e2 * math.sin(p) ** 2); T = math.tan(p) ** 2; C = ep2 * math.cos(p) ** 2
    A = (l - lon0) * math.cos(p)
    M = a * ((1 - e2 / 4 - 3 * e2 ** 2 / 64 - 5 * e2 ** 3 / 256) * p - (3 * e2 / 8 + 3 * e2 ** 2 / 32 + 45 * e2 ** 3 / 1024) * math.sin(2 * p)
             + (15 * e2 ** 2 / 256 + 45 * e2 ** 3 / 1024) * math.sin(4 * p) - (35 * e2 ** 3 / 3072) * math.sin(6 * p))
    x = 500000 + k0 * N * (A + (1 - T + C) * A ** 3 / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ep2) * A ** 5 / 120)
    y = k0 * (M + N * math.tan(p) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A ** 4 / 24 + (61 - 58 * T + T * T + 600 * C - 330 * ep2) * A ** 6 / 720))
    return x, y

def origin():
    """UTM of the world origin (Unity 0,0), rounded to a whole metre so DEM pixels line up."""
    x, y = utm12(LAT0, LON0)
    return round(x), round(y)

DEM = "https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage"
PC = "https://planetarycomputer.microsoft.com/api/data/v1/item/bbox/{box}/{px}x{px}.png"
NAIP_ITEM = "ut_m_3811034_se_12_060_20210915"
S2_ITEM = "S2C_MSIL2A_20250705T180941_R084_T12SWH_20250705T231716"

def get(full, out):
    for attempt in range(4):
        try:
            data = urllib.request.urlopen(urllib.request.Request(full, headers={"User-Agent": "simflake-urc-site"}), timeout=180).read()
            if len(data) > 2000 and not data.lstrip().startswith(b"{"): break
            print("  server said:", data[:200])
        except Exception as e:
            print("  retry:", e)
    else:
        sys.exit("failed: " + out)
    open(out, "wb").write(data); print(out, len(data) // 1024, "kB")

def crop(collection, item, bbox, px, out, **q):
    q = dict(collection=collection, item=item, coord_crs="epsg:26912", dst_crs="epsg:26912", **q)
    get(PC.format(box=",".join(str(v) for v in bbox), px=px) + "?" + urllib.parse.urlencode(q), out)

def export(url, bbox, px, out, **extra):
    q = dict(bbox=",".join(str(v) for v in bbox), bboxSR=26912, imageSR=26912, size=f"{px},{px}", f="image", **extra)
    get(url + "?" + urllib.parse.urlencode(q), out)

def main():
    out = sys.argv[1]; os.makedirs(out, exist_ok=True)
    x0, y0 = origin(); print("origin UTM 12N:", x0, y0)
    def box(size, cx=0, cy=0, half=None):
        h = (half or size / 2); return (x0 + cx - h, y0 + cy - h, x0 + cx + h, y0 + cy + h)
    dem = dict(format="tiff", pixelType="F32", interpolation="RSP_BilinearInterpolation", noData=-9999)
    # core DEM at 1 m: +1 px so heightmap tiles (2^n + 1 samples) share their edge samples
    h = CORE / 2
    export(DEM, (x0 - h - 0.5, y0 - h - 0.5, x0 + h + 0.5, y0 + h + 0.5), CORE + 1, f"{out}/dem_core_1m.tif", **dem)
    export(DEM, box(FAR), 1024, f"{out}/dem_far_16m.tif", **dem)
    # core imagery at 0.5 m in four quadrants, colour and near infrared (plants are bright in NIR)
    for i, (cx, cy) in enumerate([(-h / 2, h / 2), (h / 2, h / 2), (-h / 2, -h / 2), (h / 2, -h / 2)]):
        crop("naip", NAIP_ITEM, box(0, cx, cy, h / 2), 2048, f"{out}/naip_core_{i}.png", assets="image", asset_bidx="image|1,2,3")
        crop("naip", NAIP_ITEM, box(0, cx, cy, h / 2), 2048, f"{out}/naip_core_nir_{i}.png", assets="image", asset_bidx="image|4")
    crop("sentinel-2-l2a", S2_ITEM, box(FAR), 1638, f"{out}/s2_far.png", assets="visual", asset_bidx="visual|1,2,3")

if __name__ == "__main__":
    main()
