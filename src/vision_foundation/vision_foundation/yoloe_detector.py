"""Core YOLOE open-vocabulary detector - no ROS dependencies.

Usage:
    detector = YOLOEDetector("models/yoloe-v8s.pt", device="cpu")
    detections = detector.detect(image, ["meter", "switch"], conf=0.25)
"""

from __future__ import annotations

import logging
from typing import List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


class Detection:
    """A single detection result."""

    __slots__ = ("class_name", "confidence", "bbox")

    def __init__(self, class_name: str, confidence: float, bbox: Tuple[int, int, int, int]):
        self.class_name = class_name
        self.confidence = confidence
        self.bbox = bbox  # (x1, y1, x2, y2)

    def __repr__(self):
        return (f"Detection(class={self.class_name!r}, conf={self.confidence:.3f}, "
                f"bbox={self.bbox})")


class YOLOEDetector:
    """Open-vocabulary object detector using YOLOE (YOLO for Everything).

    YOLOE supports text-prompt-based detection: specify target categories as
    natural language strings and the model detects them without re-training.
    """

    def __init__(self, model_path: str, device: str = "cpu"):
        """Load YOLOE model.

        Args:
            model_path: Path to model weights (.pt file).
            device: Inference device ("cpu", "cuda:0", etc.).
        """
        self._model_path = model_path
        self._device = device
        self._model = None
        self._model = self._load_model()

    def _load_model(self):
        """Lazy-load the YOLOE model."""
        try:
            from ultralytics import YOLOE
        except ImportError:
            raise ImportError(
                "ultralytics YOLOE not available. "
                "Install: pip install ultralytics>=8.4.0"
            )

        # Ensure the CLIP text encoder (mobileclip_blt.ts) is findable.
        # ultralytics looks in SETTINGS["weights_dir"] for it, but that
        # directory may not exist or be a file on this system.
        self._ensure_text_encoder()

        model = YOLOE(self._model_path)
        logger.info(f"YOLOE model loaded from {self._model_path} (device={self._device})")
        return model

    def _ensure_text_encoder(self):
        """Make sure mobileclip_blt.ts is reachable by ultralytics.

        ultralytics' attempt_download_asset checks (in order):
          1. CWD / mobileclip_blt.ts
          2. SETTINGS["weights_dir"] / mobileclip_blt.ts
        """
        import os
        from pathlib import Path

        encoder_name = "mobileclip_blt.ts"

        # Already reachable from CWD
        if Path(encoder_name).exists():
            logger.debug(f"{encoder_name} found in CWD")
            return

        # Already in weights_dir — nothing to do
        try:
            from ultralytics.utils import SETTINGS
            weights_dir = SETTINGS.get("weights_dir", "")
            if weights_dir and (Path(weights_dir) / encoder_name).exists():
                logger.debug(f"{encoder_name} found in {weights_dir}")
                return
        except Exception:
            pass

        # Look alongside the model file
        model_dir = Path(self._model_path).resolve().parent
        local_encoder = model_dir / encoder_name
        if local_encoder.exists():
            # Copy to CWD so ultralytics finds it via Path.exists()
            import shutil
            shutil.copy2(local_encoder, encoder_name)
            logger.info(f"Copied {encoder_name} to CWD from {model_dir}")

        logger.info(f"{encoder_name} not found locally; ultralytics will attempt download")

    @property
    def is_loaded(self) -> bool:
        return self._model is not None

    def detect(
        self,
        image: np.ndarray,
        categories: List[str],
        confidence_threshold: float = 0.25,
    ) -> List[Detection]:
        """Run open-vocabulary detection on an image.

        Args:
            image: BGR or RGB image as numpy array (H, W, 3).
            categories: List of text prompts to detect, e.g. ["meter", "switch"].
            confidence_threshold: Minimum confidence score (0.0-1.0).

        Returns:
            List of Detection objects.
        """
        if self._model is None:
            self._model = self._load_model()

        if not categories:
            logger.warning("No categories specified, returning empty result")
            return []

        # Set target classes via text prompt embeddings
        try:
            text_pe = self._model.get_text_pe(categories)
            self._model.set_classes(categories, text_pe)
        except AttributeError:
            # Fallback: older ultralytics YOLOE API
            logger.debug("get_text_pe/set_classes not found, using predict kwargs")
            pass

        # Run inference
        results = self._model.predict(
            image,
            conf=confidence_threshold,
            device=self._device,
            verbose=False,
        )

        return self._parse_results(results, categories)

    def _parse_results(self, results, categories: List[str]) -> List[Detection]:
        """Parse ultralytics Results into Detection list."""
        detections = []

        if not results or len(results) == 0:
            return detections

        result = results[0]

        if result.boxes is None:
            return detections

        boxes = result.boxes
        for i in range(len(boxes)):
            conf = float(boxes.conf[i])
            cls_id = int(boxes.cls[i])
            xyxy = boxes.xyxy[i].cpu().numpy()

            # Map class ID to name
            if cls_id < len(categories):
                class_name = categories[cls_id]
            else:
                class_name = result.names.get(cls_id, f"class_{cls_id}")

            detections.append(Detection(
                class_name=class_name,
                confidence=conf,
                bbox=(int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])),
            ))

        return detections
