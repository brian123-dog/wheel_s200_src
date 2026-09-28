from __future__ import annotations

from pathlib import Path
from typing import Any


def load_task_config(path: str | Path) -> dict[str, Any]:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            "PyYAML is required to load cabinet task YAML files. "
            "Install this package or run tests with direct Python configs."
        ) from exc

    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Task config must be a mapping: {config_path}")
    return data
