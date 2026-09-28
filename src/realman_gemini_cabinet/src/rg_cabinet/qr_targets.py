from __future__ import annotations

import copy
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rg_cabinet.geometry import Pose


@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    def as_dict(self) -> dict[str, float]:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "w": self.w,
        }


@dataclass(frozen=True)
class QrTargetSet:
    source_file: str
    poses: list[Pose]
    quaternions: list[Quaternion | None] | None = None
    names: list[str | None] | None = None

    def quaternion_at(self, index: int) -> Quaternion | None:
        if self.quaternions is None or index >= len(self.quaternions):
            return None
        return self.quaternions[index]

    def name_at(self, index: int) -> str | None:
        if self.names is None or index >= len(self.names):
            return None
        return self.names[index]

    def has_named_targets(self) -> bool:
        return any(self.name_at(index) for index in range(len(self.poses)))


def load_qr_target_set(
    qr_result_file: str | Path | None = None,
    qr_results_dir: str | Path | None = None,
    frame: str = "base_link",
) -> QrTargetSet:
    path = _select_qr_file(qr_result_file, qr_results_dir)
    outer = _read_json(path)
    qr_payload = _parse_qr_data(outer, path)
    positions = _extract_positions(qr_payload, path)
    parsed = []
    names: list[str | None] = []
    for index, item in enumerate(positions):
        names.append(_read_optional_name(item, path, index))
        parsed.append(_position_to_target(item, frame=frame, path=path, index=index))
    targets = QrTargetSet(
        source_file=str(path),
        poses=[item[0] for item in parsed],
        quaternions=[item[1] for item in parsed],
        names=names,
    )
    _target_indexes_by_name(targets)
    return targets


def overlay_qr_observation_poses(
    config: dict[str, Any],
    targets: QrTargetSet,
) -> dict[str, Any]:
    sequence = config.get("sequence")
    if not isinstance(sequence, list) or not sequence:
        raise ValueError("Task config must contain a non-empty sequence")

    updated = copy.deepcopy(config)
    updated_sequence = updated["sequence"]
    # Explicit fixed observations are not overwritten by QR positions. Validate
    # all of them before any robot adapter/motion is created by the caller.
    for index, step in enumerate(updated_sequence):
        observation = step.get('observation_pose', {})
        if isinstance(observation, dict) and observation.get('source') == 'fixed':
            if observation.get('qr_name') or observation.get('skip'):
                raise ValueError('Fixed observation cannot also set qr_name or skip')
            frame = observation.get('frame', 'base_link')
            if frame != 'base_link':
                raise ValueError('Fixed observation must use the driver base_link frame')
            pose, quaternion = _position_to_target(observation, frame, Path('<fixed observation>'), index)
            values = [pose.x, pose.y, pose.z]
            if quaternion is not None:
                values.extend(quaternion.as_dict().values())
            else:
                values.extend([pose.rx, pose.ry, pose.rz])
            if not all(math.isfinite(v) for v in values):
                raise ValueError('Fixed observation must have finite pose values')
    if targets.has_named_targets() or _sequence_uses_qr_names(updated_sequence):
        named_targets = _target_indexes_by_name(targets)
        for index, step in enumerate(updated_sequence):
            if _step_skips_qr_observation(step):
                continue
            qr_name = _step_qr_name(step, index)
            if qr_name not in named_targets:
                available = ", ".join(sorted(named_targets)) or "<none>"
                raise ValueError(
                    f"Missing QR target name '{qr_name}' for sequence step "
                    f"{index} ({step.get('name', '<unnamed>')}); "
                    f"available names: {available}"
                )
            target_index = named_targets[qr_name]
            _write_observation_pose(
                step,
                targets.poses[target_index],
                targets.quaternion_at(target_index),
            )
    else:
        # Joint-space steps do not consume a QR position, so compare only the
        # steps that actually observe a QR target.
        observing = [
            index
            for index, step in enumerate(updated_sequence)
            if not _step_skips_qr_observation(step)
        ]
        if len(observing) != len(targets.poses):
            raise ValueError(
                "QR target count does not match task sequence length: "
                f"targets={len(targets.poses)} sequence={len(observing)}"
            )
        for qr_index, step_index in enumerate(observing):
            _write_observation_pose(
                updated_sequence[step_index],
                targets.poses[qr_index],
                targets.quaternion_at(qr_index),
            )
    updated["qr_source_file"] = targets.source_file
    return updated


def overlay_qr_targets(config: dict[str, Any], targets: QrTargetSet) -> dict[str, Any]:
    return overlay_qr_observation_poses(config, targets)


def _select_qr_file(
    qr_result_file: str | Path | None,
    qr_results_dir: str | Path | None,
) -> Path:
    if qr_result_file:
        path = Path(qr_result_file).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"QR result file does not exist: {path}")
        return path

    if not qr_results_dir:
        raise ValueError("Either qr_result_file or qr_results_dir must be set")

    directory = Path(qr_results_dir).expanduser()
    if not directory.is_dir():
        raise FileNotFoundError(f"QR results directory does not exist: {directory}")

    preferred = directory / "position.json"
    if preferred.is_file():
        return preferred

    candidates = [p for p in directory.glob("qr_*.json") if p.is_file()]
    if not candidates:
        raise FileNotFoundError(
            f"No position.json or qr_*.json files found in {directory}"
        )
    return max(candidates, key=lambda p: (p.stat().st_mtime_ns, p.name))


def _read_json(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid QR result JSON in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"QR result must be a JSON object: {path}")
    return data


def _parse_qr_data(outer: dict[str, Any], path: Path) -> dict[str, Any]:
    if "qr_data" not in outer:
        raise ValueError(f"QR result is missing qr_data: {path}")
    raw = outer["qr_data"]
    if isinstance(raw, dict):
        return raw
    if not isinstance(raw, str):
        raise ValueError(f"qr_data must be a JSON string or object: {path}")
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"qr_data is not valid JSON in {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"qr_data must decode to a JSON object: {path}")
    return payload


def _extract_positions(payload: dict[str, Any], path: Path) -> list[dict[str, Any]]:
    if "position" not in payload:
        raise ValueError(f"qr_data is missing position: {path}")
    positions = payload["position"]
    if isinstance(positions, dict):
        positions = [positions]
    if not isinstance(positions, list) or not positions:
        raise ValueError(f"qr_data.position must be a non-empty list: {path}")
    for index, item in enumerate(positions):
        if not isinstance(item, dict):
            raise ValueError(f"qr_data.position[{index}] must be an object: {path}")
    return positions


def _read_optional_name(item: dict[str, Any], path: Path, index: int) -> str | None:
    if "name" not in item or item["name"] is None:
        return None
    raw = item["name"]
    if not isinstance(raw, str):
        raise ValueError(f"qr_data.position[{index}].name must be a string: {path}")
    name = raw.strip()
    if not name:
        raise ValueError(f"qr_data.position[{index}].name must not be empty: {path}")
    return name


def _sequence_uses_qr_names(sequence: list[dict[str, Any]]) -> bool:
    for step in sequence:
        if _step_skips_qr_observation(step):
            continue
        observation_pose = step.get("observation_pose")
        if isinstance(observation_pose, dict) and "qr_name" in observation_pose:
            return True
    return False


def _step_skips_qr_observation(step: dict[str, Any]) -> bool:
    """Fixed observations and joint-space steps do not consume QR positions.

    Fixed observations are still visited by the task node; only QR overlay is
    skipped. Joint-space steps carry their own absolute joint target.
    """
    observation_pose = step.get("observation_pose")
    if isinstance(observation_pose, dict) and observation_pose.get("source") == "fixed":
        return True
    if isinstance(observation_pose, dict) and observation_pose.get("skip"):
        return True
    if observation_pose is not None and not isinstance(observation_pose, dict):
        return False
    return str(step.get("skill", "")) == "move_joint"


def _step_qr_name(step: dict[str, Any], index: int) -> str:
    observation_pose = step.get("observation_pose")
    if isinstance(observation_pose, dict) and "qr_name" in observation_pose:
        raw = observation_pose["qr_name"]
        source = "observation_pose.qr_name"
    else:
        raw = step.get("name")
        source = "name"
    if not isinstance(raw, str):
        raise ValueError(
            f"Named QR matching requires sequence[{index}].{source} to be a string"
        )
    name = raw.strip()
    if not name:
        raise ValueError(
            f"Named QR matching requires sequence[{index}].{source} to be non-empty"
        )
    return name


def _target_indexes_by_name(targets: QrTargetSet) -> dict[str, int]:
    indexes: dict[str, int] = {}
    for index in range(len(targets.poses)):
        name = targets.name_at(index)
        if name is None:
            continue
        if name in indexes:
            raise ValueError(
                f"Duplicate QR target name '{name}' in {targets.source_file}"
            )
        indexes[name] = index
    return indexes


def _write_observation_pose(
    step: dict[str, Any],
    pose: Pose,
    quaternion: Quaternion | None,
) -> None:
    observation_pose = dict(step.get("observation_pose", {}))
    observation_pose["frame"] = pose.frame
    if quaternion is None:
        observation_pose["pose"] = pose.as_list()
        observation_pose.pop("position", None)
        observation_pose.pop("orientation", None)
    else:
        observation_pose["position"] = {
            "x": pose.x,
            "y": pose.y,
            "z": pose.z,
        }
        observation_pose["orientation"] = quaternion.as_dict()
        observation_pose.pop("pose", None)
    step["observation_pose"] = observation_pose


def _position_to_target(
    item: dict[str, Any],
    frame: str,
    path: Path,
    index: int,
) -> tuple[Pose, Quaternion | None]:
    if _has_rpy_pose(item):
        return _position_to_rpy_pose(item, frame, path, index), None
    if _has_nested_quaternion_pose(item):
        x, y, z = _read_nested_position(item, path, index)
        quaternion = _read_nested_quaternion(item, path, index)
        return Pose(x=x, y=y, z=z, frame=frame), quaternion
    if _has_flat_quaternion_pose(item):
        x, y, z = _read_flat_position(item, path, index)
        quaternion = _normalize_quaternion(
            item["qx"],
            item["qy"],
            item["qz"],
            item["qw"],
            path,
            index,
        )
        return Pose(x=x, y=y, z=z, frame=frame), quaternion
    if _has_partial_rpy_pose(item):
        return _position_to_rpy_pose(item, frame, path, index), None
    if _has_partial_flat_quaternion_pose(item):
        x, y, z = _read_flat_position(item, path, index)
        missing = [name for name in ("qx", "qy", "qz", "qw") if name not in item]
        if missing:
            raise ValueError(
                f"qr_data.position[{index}] is missing fields {missing}: {path}"
            )
        quaternion = _normalize_quaternion(
            item["qx"],
            item["qy"],
            item["qz"],
            item["qw"],
            path,
            index,
        )
        return Pose(x=x, y=y, z=z, frame=frame), quaternion

    accepted = (
        "x/y/z/roll/pitch/yaw, "
        "position{x/y/z}+orientation{x/y/z/w}, "
        "or x/y/z/qx/qy/qz/qw"
    )
    raise ValueError(
        f"qr_data.position[{index}] must use one of: {accepted}: {path}"
    )


def _position_to_pose(item: dict[str, Any], frame: str, path: Path, index: int) -> Pose:
    return _position_to_target(item, frame, path, index)[0]


def _position_to_rpy_pose(
    item: dict[str, Any],
    frame: str,
    path: Path,
    index: int,
) -> Pose:
    required = ("x", "y", "z", "roll", "pitch", "yaw")
    missing = [name for name in required if name not in item]
    if missing:
        raise ValueError(
            f"qr_data.position[{index}] is missing fields {missing}: {path}"
        )
    try:
        return Pose(
            x=float(item["x"]),
            y=float(item["y"]),
            z=float(item["z"]),
            rx=float(item["roll"]),
            ry=float(item["pitch"]),
            rz=float(item["yaw"]),
            frame=frame,
        )
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"qr_data.position[{index}] contains non-numeric values: {path}"
        ) from exc


def _has_rpy_pose(item: dict[str, Any]) -> bool:
    return all(name in item for name in ("x", "y", "z", "roll", "pitch", "yaw"))


def _has_partial_rpy_pose(item: dict[str, Any]) -> bool:
    return any(name in item for name in ("roll", "pitch", "yaw"))


def _has_nested_quaternion_pose(item: dict[str, Any]) -> bool:
    return "position" in item or "orientation" in item


def _has_flat_quaternion_pose(item: dict[str, Any]) -> bool:
    return all(name in item for name in ("x", "y", "z", "qx", "qy", "qz", "qw"))


def _has_partial_flat_quaternion_pose(item: dict[str, Any]) -> bool:
    return any(name in item for name in ("qx", "qy", "qz", "qw"))


def _read_flat_position(
    item: dict[str, Any],
    path: Path,
    index: int,
) -> tuple[float, float, float]:
    missing = [name for name in ("x", "y", "z") if name not in item]
    if missing:
        raise ValueError(
            f"qr_data.position[{index}] is missing fields {missing}: {path}"
        )
    try:
        return float(item["x"]), float(item["y"]), float(item["z"])
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"qr_data.position[{index}] contains non-numeric position values: {path}"
        ) from exc


def _read_nested_position(
    item: dict[str, Any],
    path: Path,
    index: int,
) -> tuple[float, float, float]:
    position = item.get("position")
    if not isinstance(position, dict):
        raise ValueError(
            f"qr_data.position[{index}].position must be an object: {path}"
        )
    missing = [name for name in ("x", "y", "z") if name not in position]
    if missing:
        raise ValueError(
            f"qr_data.position[{index}].position is missing fields {missing}: {path}"
        )
    try:
        return (
            float(position["x"]),
            float(position["y"]),
            float(position["z"]),
        )
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"qr_data.position[{index}].position contains non-numeric values: {path}"
        ) from exc


def _read_nested_quaternion(
    item: dict[str, Any],
    path: Path,
    index: int,
) -> Quaternion:
    orientation = item.get("orientation")
    if not isinstance(orientation, dict):
        raise ValueError(
            f"qr_data.position[{index}].orientation must be an object: {path}"
        )
    missing = [name for name in ("x", "y", "z", "w") if name not in orientation]
    if missing:
        raise ValueError(
            f"qr_data.position[{index}].orientation is missing fields {missing}: {path}"
        )
    return _normalize_quaternion(
        orientation["x"],
        orientation["y"],
        orientation["z"],
        orientation["w"],
        path,
        index,
    )


def _normalize_quaternion(
    x_value: Any,
    y_value: Any,
    z_value: Any,
    w_value: Any,
    path: Path,
    index: int,
) -> Quaternion:
    try:
        x = float(x_value)
        y = float(y_value)
        z = float(z_value)
        w = float(w_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"qr_data.position[{index}] contains non-numeric quaternion values: {path}"
        ) from exc

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError(
            f"qr_data.position[{index}] quaternion must not be zero: {path}"
        )
    return Quaternion(x=x / norm, y=y / norm, z=z / norm, w=w / norm)
