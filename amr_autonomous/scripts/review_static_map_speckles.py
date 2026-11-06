#!/usr/bin/env python3
"""Generate a *candidate* cleaned ROS static map by removing isolated PGM speckles.

Never changes the original image/YAML, /map, /scan_slam, or any live costmap.
Only removes very small eight-connected occupied clusters fully surrounded by
known free space and well inside image boundaries. All larger shapes, walls,
and unknown space remain. Operator MUST visually inspect before using a map.
PGM (P5/P2, maxval=255) only; PNG maps must be converted explicitly first.
"""
import argparse
import json
import math
import re
from collections import deque
from pathlib import Path


def read_pgm(path):
    with path.open("rb") as f:
        # PGM whitespace/comment scanner. Keep the raster bytes unmodified.
        def token():
            while True:
                ch = f.read(1)
                if not ch:
                    raise ValueError("Truncated PGM header")
                if ch == b"#":
                    f.readline()
                    continue
                if ch not in b" \t\r\n":
                    break
            buf = bytearray(ch)
            while True:
                ch = f.read(1)
                if not ch or ch in b" \t\r\n":
                    break
                buf.extend(ch)
            return bytes(buf)
        magic = token()
        width = int(token())
        height = int(token())
        maxval = int(token())
        if magic not in (b"P5", b"P2") or maxval != 255:
            raise ValueError("Only 8-bit P5/P2 PGM maps are supported")
        if width <= 0 or height <= 0 or width*height > 100_000_000:
            raise ValueError("Invalid or excessive image size")
        if magic == b"P5":
            raw = f.read()
            if len(raw) != width*height:
                raise ValueError("Unexpected PGM raster length")
            data = bytearray(raw)
        else:
            values = [int(token()) for _ in range(width*height)]
            if any(v < 0 or v > 255 for v in values):
                raise ValueError("PGM pixel outside 0..255")
            data = bytearray(values)
        return width, height, data


def get_yaml_scalar(text, name):
    match = re.search(r"^\s*" + re.escape(name) + r"\s*:\s*(.*?)\s*(?:#.*)?$", text, re.M)
    if not match:
        raise ValueError("YAML missing: " + name)
    return match.group(1).strip("'\"")


def map_info(map_yaml):
    import ast
    text = map_yaml.read_text(encoding="utf-8")
    image = get_yaml_scalar(text, "image")
    path = Path(image)
    if not path.is_absolute():
        path = map_yaml.parent / path
    if path.suffix.lower() != ".pgm":
        raise ValueError("Map image must be PGM (not PNG): " + str(path))
    resolution = float(get_yaml_scalar(text, "resolution"))
    negate = int(get_yaml_scalar(text, "negate"))
    occupied = float(get_yaml_scalar(text, "occupied_thresh"))
    free = float(get_yaml_scalar(text, "free_thresh"))
    origin = ast.literal_eval(get_yaml_scalar(text, "origin"))
    if not (resolution > 0 and negate in (0, 1) and
            0 < free < occupied < 1 and len(origin) >= 3):
        raise ValueError("Unexpected map metadata; refusing to modify")
    if abs(float(origin[2])) > 1e-6:
        raise ValueError("Rotated map origin requires separate review")
    return text, path, resolution, negate, occupied, free, origin


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", required=True, type=Path)
    ap.add_argument("--out-dir", type=Path, default=None)
    ap.add_argument("--max-cluster-pixels", type=int, default=4)
    ap.add_argument("--border-margin-m", type=float, default=0.4)
    args = ap.parse_args()
    if not (1 <= args.max_cluster_pixels <= 20):
        ap.error("max-cluster-pixels must be 1..20 (no broad obstacle deletion)")
    if not (0.1 <= args.border_margin_m <= 2.0):
        ap.error("border-margin-m must be 0.1..2.0")

    yaml_path = args.map.expanduser().resolve()
    source_yaml, image_path, res, negate, occupied, free, origin = map_info(yaml_path)
    w, h, pixels = read_pgm(image_path)

    # OccupancyGrid probabilities follow the standard Nav2 map YAML semantics.
    occ = bytearray(w*h)
    unknown = bytearray(w*h)
    for i, value in enumerate(pixels):
        probability = (value if negate else 255-value)/255.0
        if probability >= occupied:
            occ[i] = 1
        elif probability > free:
            unknown[i] = 1

    margin = math.ceil(args.border_margin_m/res)
    visited = bytearray(w*h)
    removable = []
    found = 0
    for first in range(w*h):
        if not occ[first] or visited[first]:
            continue
        found += 1
        queue = deque([first])
        visited[first] = 1
        group = []
        touches_unknown = False
        too_big = False
        near_border = False
        while queue:
            index = queue.popleft()
            group.append(index)
            x, y = index % w, index//w
            if x < margin or y < margin or x >= w-margin or y >= h-margin:
                near_border = True
            if len(group) > args.max_cluster_pixels:
                too_big = True
            for yy in range(max(0,y-1), min(h,y+2)):
                for xx in range(max(0,x-1), min(w,x+2)):
                    nb = yy*w + xx
                    if unknown[nb]:
                        touches_unknown = True
                    if occ[nb] and not visited[nb]:
                        visited[nb] = 1
                        queue.append(nb)
        if not too_big and not touches_unknown and not near_border:
            removable.append(group)

    out = args.out_dir or yaml_path.parent / (yaml_path.stem+"_review")
    out = out.expanduser().resolve()
    if out == yaml_path.parent:
        raise ValueError("Output directory must differ from source map directory")
    out.mkdir(parents=True, exist_ok=True)
    clean = bytearray(pixels)
    mask = bytearray([255])*len(pixels)
    for group in removable:
        for index in group:
            # 255 = free for negate=0; 0 = free for negate=1
            clean[index] = 0 if negate else 255
            mask[index] = 0
    map_image = out/"map_speckle_candidate.pgm"
    mask_image = out/"removed_pixels_review.pgm"
    for output, raster in ((map_image, clean), (mask_image, mask)):
        with output.open("wb") as f:
            f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
            f.write(raster)
    changed_yaml = re.sub(
        r"^(\s*image\s*:\s*).*?$",
        "image: " + map_image.name, source_yaml,
        count=1, flags=re.M)
    new_yaml = out/"map_speckle_candidate.yaml"
    new_yaml.write_text(changed_yaml, encoding="utf-8")

    centers = []
    for group in removable[:2000]:
        col = sum(i%w for i in group)/len(group)
        row = sum(i//w for i in group)/len(group)
        centers.append({
            "pixels":len(group),
            "map_x_m":round(float(origin[0])+col*res, 3),
            "map_y_m":round(float(origin[1])+(h-1-row)*res, 3)
        })
    report = {
        "original_map_yaml": str(yaml_path),
        "original_image": str(image_path),
        "candidate_map_yaml": str(new_yaml),
        "map_resolution_m":res,
        "map_width":w, "map_height":h,
        "total_occupied_clusters":found,
        "candidate_removed_clusters":len(removable),
        "candidate_removed_pixels":sum(len(g) for g in removable),
        "criteria":{
            "max_cluster_pixels":args.max_cluster_pixels,
            "border_margin_m":args.border_margin_m,
            "requires_known_free_neighbors":True,
            "never_modifies_original":True,
            "never_modifies_live_costmap":True
        },
        "candidate_centers_first_2000":centers,
        "warning":"Candidate only. A small component could be a real obstacle. Review physically and visually before deployment."
    }
    (out/"review.json").write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding="utf-8")
    print(json.dumps({key:value for key,value in report.items() if key!="candidate_centers_first_2000"},
                     indent=2, ensure_ascii=False))
    print("[REVIEW] The original map is unchanged. Candidate is NOT loaded by Nav2.")


if __name__=="__main__":
    main()
