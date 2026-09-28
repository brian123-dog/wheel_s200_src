"""Bounded Y/Z centering at the existing fast-approach endpoint.

Always re-observe after a correction. This does not change X stand-off/contact
travel or calibrate TCP/depth bias, and cannot use alignment detours near a panel.
"""
from dataclasses import replace
import math
import time


def validate_precontact_centering(cfg, context):
    templates = cfg.precontact_panel_layout_templates
    if not isinstance(templates, (list, tuple)) or len(templates) > 8 or any(
            not isinstance(p, str) or not p.strip() for p in templates):
        raise ValueError('precontact_panel_layout_templates must contain up to eight nonempty paths')
    if not callable(context.target_refresher):
        raise ValueError('接触前居中需要实时视觉刷新')
    if cfg.approach_axis != 'x' or cfg.approach_frame != 'base' or set(cfg.align_axes) != {'y', 'z'}:
        raise ValueError('接触前居中仅支持基坐标 X 进给、Y/Z 修正')
    if cfg.intermediate_check_distance_m != 0:
        raise ValueError('接触前居中不能与旧版中途二次检测同时启用')
    limits = {
        'precontact_centering_tolerance_m': (.0001, .005),
        'precontact_centering_max_axis_m': (.001, .020),
        'precontact_centering_max_total_m': (.001, .040),
        'precontact_centering_speed_mps': (.001, .010),
        'precontact_centering_settle_s': (.1, 2.),
    }
    for name, (lo, hi) in limits.items():
        value = getattr(cfg, name)
        if isinstance(value, bool) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f'{name} must be finite and in [{lo}, {hi}]')
    capture = cfg.precontact_centering_max_error_m
    if capture is not None and (isinstance(capture, bool) or not math.isfinite(capture)
            or not cfg.precontact_centering_max_axis_m <= capture <= .030):
        raise ValueError('precontact_centering_max_error_m must be between the step limit and 0.030 m')
    n = cfg.precontact_centering_max_corrections
    if type(n) is not int or not 1 <= n <= 3:
        raise ValueError('precontact_centering_max_corrections must be in [1, 3]')
    if min(cfg.precontact_centering_max_axis_m, cfg.precontact_centering_max_total_m) < cfg.precontact_centering_tolerance_m:
        raise ValueError('接触前居中修正上限不能小于居中容差')
    if not callable(getattr(context.robot, 'preflight_movel_alignment', None)):
        raise ValueError('接触前居中需要支持 MoveL 关节预算的驱动')


def center_before_contact(skill, context, target):
    from .knob_camera_servo import KnobCameraServoSkill, _report_stage
    cfg, robot = skill.config, context.robot
    # No 10-cm reroutes here: only small translations parallel to the panel.
    fine = KnobCameraServoSkill(replace(
        cfg, alignment_reroute=False, alignment_preflight=True,
        align_speed_mps=cfg.precontact_centering_speed_mps,
        max_align_axis_m=cfg.precontact_centering_max_axis_m,
        max_align_total_m=cfg.precontact_centering_max_total_m))
    capture = (cfg.precontact_centering_max_error_m if cfg.precontact_centering_max_error_m is not None
               else cfg.precontact_centering_max_axis_m)
    report = {'enabled': True, 'tolerance_m': cfg.precontact_centering_tolerance_m,
              'max_observed_error_m': capture,
              'max_step_axis_m': cfg.precontact_centering_max_axis_m,
              'max_total_travel_m': cfg.precontact_centering_max_total_m,
              'corrections': 0, 'total_travel_m': 0., 'history': []}
    old_mode = context.precontact_centering
    context.precontact_centering = True
    try:
        for attempt in range(cfg.precontact_centering_max_corrections + 1):
            _report_stage(robot, 'precontact_centering')
            time.sleep(cfg.precontact_centering_settle_s)
            try:
                refreshed = context.target_refresher()
            except Exception as exc:
                raise RuntimeError(f'接触前居中识别失败，未进入慢速接触：{exc}') from exc
            if refreshed is None or (refreshed.name, refreshed.kind, refreshed.pose.frame) != (target.name, target.kind, target.pose.frame):
                raise RuntimeError('接触前居中目标缺失或身份/坐标系不一致，未进入慢速接触')
            details = refreshed.details
            sampling = details.get('fresh_sampling', {})
            if sampling.get('enabled') is not True or sampling.get('confirmations', 0) < 2:
                raise RuntimeError('接触前居中缺少两帧新图确认，未进入慢速接触')
            raw = details.get('alignment_error_base_m')
            if (details.get('alignment_reference') != 'tcp_frame'
                    or raw is None or len(raw) != 3
                    or not all(math.isfinite(float(v)) for v in raw)):
                raise RuntimeError('接触前居中缺少有效夹爪中心误差，未进入慢速接触')
            delta = {'x': 0., 'y': float(raw[1]), 'z': float(raw[2])}
            residual = max(abs(delta['y']), abs(delta['z']))
            item = {'observation': attempt + 1, 'error_base_m': delta,
                    'pixel': details.get('pixel'), 'depth_m': details.get('depth_m'),
                    'fresh_sampling': sampling, 'debug_images': details.get('debug_images')}
            report['history'].append(item)
            if residual <= cfg.precontact_centering_tolerance_m:
                item['status'] = 'centered'
                report['final_error_base_m'] = delta
                report['status'] = 'VERIFIED'
                return refreshed, report
            if residual > capture:
                raise RuntimeError(f'接触前居中观测偏差超限：Y={delta["y"]*1000:.1f} mm，Z={delta["z"]*1000:.1f} mm，允许每轴 {capture*1000:.1f} mm；未进入慢速接触')
            if attempt >= cfg.precontact_centering_max_corrections:
                raise RuntimeError(f'接触前居中复查未通过：Y={delta["y"]*1000:.1f} mm，Z={delta["z"]*1000:.1f} mm；未进入慢速接触')
            desired = {axis: value if abs(value) > cfg.precontact_centering_tolerance_m else 0.
                       for axis, value in delta.items()}
            remaining = cfg.precontact_centering_max_total_m - report['total_travel_m']
            # Do not start a correction already known to exceed remaining travel.
            if sum(abs(v) for v in desired.values()) > remaining + 1e-9:
                raise RuntimeError(f'接触前居中剩余修正预算不足：需要 {sum(abs(v) for v in desired.values())*1000:.1f} mm，剩余 {remaining*1000:.1f} mm；未进入慢速接触')
            limit = cfg.precontact_centering_max_axis_m
            move = {axis: max(-limit, min(limit, value)) for axis, value in desired.items()}
            item['step_limited'] = move != desired
            travel = sum(abs(v) for v in move.values())
            if report['total_travel_m'] + travel > cfg.precontact_centering_max_total_m + 1e-9:
                raise RuntimeError('接触前居中累计修正超过距离上限，未进入慢速接触')
            fine._validate_alignment_delta(move)
            fine._servo_alignment(robot, move, min_distance_m=0.)
            report['corrections'] += 1
            report['total_travel_m'] += travel
            item.update(status='corrected', move_delta_base_m=move)
    finally:
        context.precontact_centering = old_mode
