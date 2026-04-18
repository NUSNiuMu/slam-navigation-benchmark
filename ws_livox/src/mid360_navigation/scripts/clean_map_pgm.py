#!/usr/bin/env python3
import argparse
from collections import deque
from pathlib import Path
from typing import Tuple

import numpy as np
from PIL import Image


def load_yaml_lines(yaml_path: Path):
    return yaml_path.read_text(encoding="utf-8").splitlines()


def save_yaml(yaml_lines, out_yaml: Path, image_name: str):
    updated = []
    replaced = False
    for line in yaml_lines:
        if line.startswith("image:"):
            updated.append(f"image: {image_name}")
            replaced = True
        else:
            updated.append(line)
    if not replaced:
        updated.insert(0, f"image: {image_name}")
    out_yaml.write_text("\n".join(updated) + "\n", encoding="utf-8")


def remove_small_occupied_components(img_arr: np.ndarray, min_component_size: int) -> Tuple[np.ndarray, int, int]:
    occupied = img_arr == 0
    h, w = occupied.shape
    visited = np.zeros_like(occupied, dtype=bool)
    cleaned = img_arr.copy()
    removed_components = 0
    removed_cells = 0

    for y in range(h):
        for x in range(w):
            if not occupied[y, x] or visited[y, x]:
                continue

            q = deque([(y, x)])
            visited[y, x] = True
            component = []

            while q:
                cy, cx = q.popleft()
                component.append((cy, cx))
                for ny, nx in ((cy - 1, cx), (cy + 1, cx), (cy, cx - 1), (cy, cx + 1)):
                    if 0 <= ny < h and 0 <= nx < w and occupied[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        q.append((ny, nx))

            if len(component) < min_component_size:
                removed_components += 1
                removed_cells += len(component)
                for cy, cx in component:
                    cleaned[cy, cx] = 255

    return cleaned, removed_components, removed_cells


def main():
    parser = argparse.ArgumentParser(description="Clean a ROS occupancy PGM by removing small occupied components.")
    parser.add_argument("input_pgm", help="Input .pgm file")
    parser.add_argument("input_yaml", help="Input .yaml file")
    parser.add_argument("output_prefix", help="Output path prefix without extension")
    parser.add_argument("--min-component-size", type=int, default=100, help="Remove occupied components smaller than this many cells")
    args = parser.parse_args()

    input_pgm = Path(args.input_pgm)
    input_yaml = Path(args.input_yaml)
    output_prefix = Path(args.output_prefix)
    output_prefix.parent.mkdir(parents=True, exist_ok=True)

    img = Image.open(input_pgm).convert("L")
    arr = np.array(img)

    cleaned, removed_components, removed_cells = remove_small_occupied_components(arr, args.min_component_size)

    out_pgm = output_prefix.with_suffix(".pgm")
    out_yaml = output_prefix.with_suffix(".yaml")
    Image.fromarray(cleaned).save(out_pgm)
    save_yaml(load_yaml_lines(input_yaml), out_yaml, out_pgm.name)

    print(f"Saved cleaned map to {out_pgm} and {out_yaml}")
    print(f"Removed {removed_components} occupied components and {removed_cells} occupied cells")


if __name__ == "__main__":
    main()
