import os
import sys
import traceback

import requests
from PIL import Image, ImageDraw

print("Script started")

url = "https://5gsxznrs-84dfx38v-5000.ahrestapi.gpufree.cn:8443/locate"
image_path = r"/home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260605_185314_624660_result.jpg"
out_path = r"/home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png"

print("Python:", sys.executable)
print("Image path:", image_path)
print("Image exists:", os.path.exists(image_path))

if not os.path.exists(image_path):
    raise FileNotFoundError(image_path)

headers = {
    "x-api-key": "change_this_key"
}

data = {
    "query":  "Changeover switch",
    "mode": "ground"
}

try:
    print("Sending request...")

    with open(image_path, "rb") as f:
        files = {
            "image": f
        }

        res = requests.post(
            url,
            headers=headers,
            files=files,
            data=data,
            timeout=60
        )

    print("HTTP status:", res.status_code)
    print("Response text:")
    print(res.text)

    res.raise_for_status()
    result = res.json()

    print("Opening image...")
    img = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(img)

    boxes = result.get("boxes", [])
    print("Num boxes:", len(boxes))

    for i, box in enumerate(boxes):
        x1 = float(box["x1"])
        y1 = float(box["y1"])
        x2 = float(box["x2"])
        y2 = float(box["y2"])
        label = box.get("label", "")

        print(f"Box {i}: {label}, ({x1}, {y1}), ({x2}, {y2})")

        draw.rectangle([x1, y1, x2, y2], outline="red", width=5)
        draw.text((x1, max(0, y1 - 40)), f"{i}: {label}", fill="red")

    img.save(out_path)

    print("Saved result:", out_path)

except Exception:
    print("ERROR:")
    traceback.print_exc()