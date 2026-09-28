#!/usr/bin/env python3
"""Test YOLOE detection on images in vision_debug/png/.

Usage:
    cd /home/sunrise/wheel_S200_src
    source src/zhizhen/venv/bin/activate
    python3 test_yoloe_knob.py
"""
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent / "src" / "vision_foundation"))
from vision_foundation.yoloe_detector import YOLOEDetector

# ── config ──────────────────────────────────────────────────────────────────
MODEL_PATH = "src/vision_foundation/models/yoloe-11s-seg.pt"
IMAGE_DIR = Path("src/vision_debug/png")
OUTPUT_DIR = Path("vision_debug/yoloe_output")
CONFIDENCE = 0.15  # low threshold to see what it picks up

# Try multiple prompts to see which works best
PROMPTS = [
    ["black knob"],
    ["rotary knob", "black knob", "round knob"],
    ["knob", "dial", "rotary switch"],
    ["black round knob", "switch knob", "control knob"],
]


def draw_boxes(img, detections, color=(0, 255, 0)):
    """Draw bounding boxes and labels on image."""
    out = img.copy()
    for d in detections:
        x1, y1, x2, y2 = d.bbox
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        label = f"{d.class_name} {d.confidence:.2f}"
        cv2.putText(out, label, (x1, max(y1 - 8, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    return out


def main():
    # Find images
    images = sorted(IMAGE_DIR.glob("*.png")) + sorted(IMAGE_DIR.glob("*.jpg"))
    if not images:
        print(f"No images found in {IMAGE_DIR}/")
        print("Please put your knob images there first.")
        return

    print(f"Found {len(images)} image(s)")

    # Load model
    print("Loading YOLOE model...")
    t0 = time.time()
    detector = YOLOEDetector(MODEL_PATH, device="cpu")
    print(f"Loaded in {time.time() - t0:.1f}s\n")

    OUTPUT_DIR.mkdir(exist_ok=True)

    for img_path in images:
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"  [SKIP] Cannot read {img_path.name}")
            continue

        print(f"=== {img_path.name} ({img.shape[1]}x{img.shape[0]}) ===")

        for prompt in PROMPTS:
            prompt_str = ", ".join(prompt)
            t0 = time.time()
            detections = detector.detect(img, prompt, confidence_threshold=CONFIDENCE)
            elapsed = time.time() - t0

            print(f"  [{elapsed:.1f}s] '{prompt_str}' → {len(detections)} objects")
            for d in detections:
                print(f"      {d.class_name}: {d.confidence:.3f} @ {d.bbox}")

            # Save best result per prompt
            annotated = draw_boxes(img, detections)
            safe_name = prompt[0].replace(" ", "_")
            out_name = f"{img_path.stem}_{safe_name}.jpg"
            cv2.imwrite(str(OUTPUT_DIR / out_name), annotated)

        print()

    print(f"Results saved to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
