"""Bounded alignment rerouting with unchanged final pose and fixed orientation.

Only base Y/Z translation is rerouted. No MoveJ, alternate IK seeds, wrist flips,
or X motion towards the panel. Environment clearance must be supplied separately.
"""
from __future__ import annotations

import math

from rg_cabinet_ros2.movel_alignment_check import check_alignment_path


def route_candidates(moves, z_offsets):
    original = [(a, float(d)) for a, d in moves if abs(d) > 1e-9]
    yield "original", original
    if any(a not in ("y", "z") for a, _ in original):
        return
    totals = {axis: sum(d for a, d in original if a == axis) for axis in ("y", "z")}
    seen = {tuple(original)}
    routes = [("reordered", [("z", totals["z"]), ("y", totals["y"])])]
    for offset in z_offsets:
        routes.append((f"z_detour_{offset:+.3f}", [("z", offset), ("y", totals["y"]),
                                                   ("z", totals["z"]-offset)]))
    for name, route in routes:
        route = [(a, d) for a, d in route if abs(d) > 1e-9]
        key = tuple(route)
        if key not in seen:
            seen.add(key)
            yield name, route


def route_in_z_bounds(start_z, moves, bounds):
    if len(bounds) != 2 or not all(math.isfinite(v) for v in bounds) or bounds[0] >= bounds[1]:
        return False
    if not bounds[0] <= start_z <= bounds[1]:
        return False
    z = start_z
    for axis, d in moves:
        z += d if axis == "z" else 0
        if not bounds[0] <= z <= bounds[1]:
            return False
    return True


def search_alignment_route(model, start, joints_deg, moves, *,
                           z_offsets=(-.025, .025, -.05, .05, -.075, .075, -.10, .10, -.125, .125, -.15, .15),
                           z_bounds=(), step_m=.001, min_margin_deg=5., max_joint_step_deg=5.):
    """Find a sampled continuous route from the ACTUAL joint branch.

    With no surveyed z bounds, search is diagnostic only. This function never
    sends motion commands. z bounds are a constraint, not an obstacle model.
    """
    if any(not math.isfinite(v) or abs(v) > .20 for v in z_offsets) or len(z_offsets) > 32:
        raise ValueError("Reroute offsets must be finite, within 0.20 m, at most 32")
    if z_bounds and (len(z_bounds) != 2 or not all(math.isfinite(v) for v in z_bounds) or z_bounds[0] >= z_bounds[1]):
        raise ValueError("Invalid surveyed reroute Z bounds")
    validator = getattr(model, "self_collision_free", None)
    if not callable(validator):
        raise RuntimeError("Rerouting requires the SDK self-collision check")
    candidates, accepted = [], []
    for name, route in route_candidates(moves, z_offsets):
        if z_bounds and not route_in_z_bounds(start.z, route, z_bounds):
            candidates.append({"name": name, "moves": route, "status": "REJECTED",
                               "reason": "OUTSIDE_SURVEYED_Z_BOUNDS"})
            continue
        result = check_alignment_path(model, start, joints_deg, route, step_m=step_m,
                                      min_margin_deg=min_margin_deg,
                                      max_joint_step_deg=max_joint_step_deg,
                                      state_validator=validator)
        summary = {"name": name, "moves": route, "status": result["status"],
                   "reason": result["reason"], "length_m": sum(abs(d) for _, d in route),
                   "minimum_margin_deg": result.get("minimum_margin_deg"),
                   "failed_axis": result.get("failed_axis"),
                   "failed_distance_m": result.get("failed_distance_m")}
        candidates.append(summary)
        if result["status"] == "PASSED":
            accepted.append((summary, result))
            if name == "original":
                break  # Keep the original motion when it is already feasible.
    report = {"scope": "alignment_movel_reroute", "status": "REJECTED",
              "reason": "NO_VALID_ROUTE_IN_SEARCH_SET", "candidates": candidates,
              "environment_collision_checked": False, "surveyed_z_bounds_m": list(z_bounds),
              "execution_clearance_configured": bool(z_bounds)}
    if not accepted:
        return report
    selected, detailed = min(accepted, key=lambda pair: (pair[0]["length_m"],
                                                       -pair[0]["minimum_margin_deg"]))
    report.update(status="PASSED", reason="ROUTE_FOUND", selected=selected,
                  path_check=detailed)
    return report
