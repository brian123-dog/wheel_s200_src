"""Render panel-layout template annotations onto their reference images.

Read-only preview tool: it draws what the matcher actually uses, so the
annotations can be eyeballed against the picture they were traced on.

Usage:
    python3 tools/preview_templates.py            # all templates
    python3 tools/preview_templates.py --out DIR  # custom output directory
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = PACKAGE_ROOT / "config"

# BGR colours, matching draw_panel_layout_result where it overlaps.
COLOR_ANCHOR = (255, 160, 0)     # blue   - global feature matching region
COLOR_SEARCH = (0, 215, 255)     # amber  - target search area, excluded from matching
COLOR_PLATE = (0, 255, 0)        # green  - square plate corners
COLOR_EXCLUDE = (0, 0, 255)      # red    - anchor exclusion polygons
COLOR_BODY = (255, 0, 255)       # magenta- round knob centre + radius

TEMPLATE_GROUPS = {
    "left_knob_square_plate": (
        CONFIG_ROOT / "panel_layout",
        [
            "template.json",
            "template_largest.json",
            "template_smallest.json",
            "template_left.json",
            "template_right.json",
            "template_distant_panel.json",
        ],
    ),
    "right_knob_round_body": (
        CONFIG_ROOT / "panel_layout_right_knob",
        [
            "template_observation.json",
            "template_upper_row_right.json",
            "template_upper_row_right_right.json",
            "template_upper_row_right_smallest.json",
            "template_user_front_1.json",
            "template_user_front_2.json",
            "template_user_front_3.json",
            "template_user_front_4.json",
        ],
    ),
}


def resolve_reference(template_path: Path, name: str) -> Path | None:
    """Reference images are stored next to the template, or under config/."""
    for candidate in (template_path.parent / name, CONFIG_ROOT / name):
        if candidate.is_file():
            return candidate
    return None


def draw_annotations(image: np.ndarray, config: dict) -> np.ndarray:
    out = image.copy()
    height, width = out.shape[:2]

    def quad(name: str) -> np.ndarray | None:
        points = config.get(name)
        if not points:
            return None
        arr = np.asarray(points, dtype=np.float32)
        if arr.shape != (4, 2):
            return None
        return arr

    # Filled tints first so the outlines stay readable on top of them.
    for name, color in (("anchor_polygon", COLOR_ANCHOR), ("search_polygon", COLOR_SEARCH)):
        points = quad(name)
        if points is None:
            continue
        overlay = out.copy()
        cv2.fillPoly(overlay, [np.rint(points).astype(np.int32)], color)
        cv2.addWeighted(overlay, 0.18, out, 0.82, 0, out)

    for name, color in (("anchor_polygon", COLOR_ANCHOR), ("search_polygon", COLOR_SEARCH)):
        points = quad(name)
        if points is None:
            continue
        cv2.polylines(out, [np.rint(points).astype(np.int32)], True, color, 2, cv2.LINE_AA)

    for points in config.get("anchor_exclusion_polygons", []) or []:
        arr = np.asarray(points, dtype=np.float32)
        if arr.shape != (4, 2):
            continue
        cv2.polylines(out, [np.rint(arr).astype(np.int32)], True, COLOR_EXCLUDE, 2, cv2.LINE_AA)

    geometry = str(config.get("local_geometry", "square_plate"))
    if geometry == "round_knob":
        center = config.get("axis_center_reference_px")
        radius = config.get("body_radius_reference_px")
        if center and radius:
            cx, cy = int(round(float(center[0]))), int(round(float(center[1])))
            r = int(round(float(radius)))
            cv2.circle(out, (cx, cy), r, COLOR_BODY, 2, cv2.LINE_AA)
            cv2.drawMarker(out, (cx, cy), COLOR_BODY, cv2.MARKER_CROSS, 17, 2)
            cv2.putText(out, f"axis({cx},{cy}) r={r}", (cx + 10, cy - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_BODY, 1, cv2.LINE_AA)
    else:
        corners = quad("plate_corners")
        if corners is not None:
            cv2.polylines(out, [np.rint(corners).astype(np.int32)], True, COLOR_PLATE, 2, cv2.LINE_AA)
            for index, (px, py) in enumerate(corners):
                cv2.putText(out, str(index), (int(px) + 4, int(py) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_PLATE, 1, cv2.LINE_AA)

    # Legend, wrapped into the top-left corner.
    legend = [
        ("anchor_polygon (match region)", COLOR_ANCHOR),
        ("search_polygon (excluded)", COLOR_SEARCH),
        (
            "plate_corners" if geometry != "round_knob" else "body circle",
            COLOR_PLATE if geometry != "round_knob" else COLOR_BODY,
        ),
    ]
    if config.get("anchor_exclusion_polygons"):
        legend.append(("anchor_exclusion", COLOR_EXCLUDE))
    for index, (label, color) in enumerate(legend):
        y = 18 + index * 18
        cv2.rectangle(out, (8, y - 10), (22, y + 2), color, -1)
        cv2.putText(out, label, (28, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(out, label, (28, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)

    cv2.putText(out, f"{config.get('target_id')} | {geometry} | {width}x{height}",
                (8, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(out, f"{config.get('target_id')} | {geometry} | {width}x{height}",
                (8, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def build_sheet(panels: list[tuple[str, np.ndarray]], columns: int) -> np.ndarray:
    if not panels:
        raise ValueError("No panels to render")
    target_width = max(image.shape[1] for _, image in panels)
    scaled: list[tuple[str, np.ndarray]] = []
    for title, image in panels:
        if image.shape[1] != target_width:
            factor = target_width / image.shape[1]
            image = cv2.resize(image, (target_width, int(round(image.shape[0] * factor))),
                               interpolation=cv2.INTER_AREA)
        banner = np.zeros((26, target_width, 3), dtype=np.uint8)
        cv2.putText(banner, title, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        scaled.append((title, np.vstack([banner, image])))

    rows = []
    for start in range(0, len(scaled), columns):
        chunk = scaled[start:start + columns]
        height = max(image.shape[0] for _, image in chunk)
        padded = [
            np.vstack([image, np.zeros((height - image.shape[0], target_width, 3), dtype=np.uint8)])
            for _, image in chunk
        ]
        while len(padded) < columns:
            padded.append(np.zeros((height, target_width, 3), dtype=np.uint8))
        rows.append(np.hstack(padded))
    return np.vstack(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=str(PACKAGE_ROOT / "vision_debug" / "template_preview"))
    parser.add_argument("--columns", type=int, default=3)
    parser.add_argument("--individual", action="store_true",
                        help="Also write one full-resolution PNG per template")
    args = parser.parse_args()

    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    written: list[Path] = []
    for group, (folder, names) in TEMPLATE_GROUPS.items():
        panels = []
        for name in names:
            template_path = folder / name
            if not template_path.is_file():
                print(f"skip (missing): {template_path}")
                continue
            config = json.loads(template_path.read_text(encoding="utf-8"))
            reference_path = resolve_reference(template_path, str(config.get("reference_image", "")))
            if reference_path is None:
                print(f"skip (no reference image): {template_path}")
                continue
            image = cv2.imread(str(reference_path), cv2.IMREAD_COLOR)
            if image is None:
                print(f"skip (unreadable): {reference_path}")
                continue
            panels.append((name, draw_annotations(image, config)))
            if args.individual:
                individual_dir = out_dir / group
                individual_dir.mkdir(parents=True, exist_ok=True)
                single = individual_dir / f"{Path(name).stem}.png"
                cv2.imwrite(str(single), draw_annotations(image, config))

        if not panels:
            continue
        sheet = build_sheet(panels, max(1, args.columns))
        destination = out_dir / f"{group}.png"
        cv2.imwrite(str(destination), sheet)
        written.append(destination)
        print(f"wrote {destination} ({len(panels)} templates, {sheet.shape[1]}x{sheet.shape[0]})")

    if not written:
        print("nothing rendered")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
