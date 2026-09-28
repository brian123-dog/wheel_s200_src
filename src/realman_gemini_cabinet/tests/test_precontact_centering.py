from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock, patch
import math
import pytest
import yaml
from rg_cabinet.adapters.mock import MockRobotAdapter
from rg_cabinet.geometry import Pose
from rg_cabinet.perception.element_detector import ElementTarget
from rg_cabinet.skills.base import SkillContext
from rg_cabinet.skills.knob_camera_servo import KnobCameraServoConfig, KnobCameraServoSkill
from rg_cabinet.task_runner import _step_for_refresh, _build_skill_config
from rg_cabinet.skills.base import ContactMotionConfig


class Robot(MockRobotAdapter):
    def __init__(self):
        super().__init__(initial_pose=Pose(-.3,0,.5,frame='base_link'))
        self.trace=[]
    def report_task_stage(self, stage): self.trace.append(('stage',stage))
    def preflight_movel_alignment(self,moves,settings):
        assert settings.alignment_reroute is False
        assert all(axis in ('y','z') for axis,_ in moves)
        self.trace.append(('check',list(moves)))
    def execute_movel_alignment_route(self,*args):
        raise AssertionError('near-contact motion must never use rerouting')
    def servo_twist(self,twist,duration_s,frame='tool'):
        self.trace.append(('move',twist,duration_s,frame))
        super().servo_twist(twist,duration_s,frame)


def target(y=0.,z=0.,**extra):
    details={'alignment_reference':'tcp_frame','alignment_error_base_m':[-.12,y,z],
             'fresh_sampling':{'enabled':True,'confirmations':2},
             'depth_m':.5,'tcp_center_camera_m':[0.,0.,.204]}
    details.update(extra)
    return ElementTarget(name='right',kind='knob',pose=Pose(-.8,0,.5,frame='base_link'),details=details)


def make_case(near,**overrides):
    robot=Robot()
    cfg=KnobCameraServoConfig(precontact_centering=True,intermediate_check_distance_m=0.,
        actuation_rpy_base=None,contact_mode='open_loop',open_loop_advance_m=.085,
        retract_m=.3,gripper_settle_s=0.,turn_angle_rad=math.pi/3)
    cfg=replace(cfg,**overrides)
    values=iter([target(),target(),*near])
    context=SkillContext(robot)
    def refresh():
        robot.trace.append(('observe',context.precontact_centering))
        value=next(values)
        if isinstance(value,Exception):raise value
        return value
    context.target_refresher=refresh
    return robot,context,KnobCameraServoSkill(cfg)


def run(skill,context):
    with patch('rg_cabinet.skills.precontact_centering.time.sleep'):
        return skill.execute(context,target())


def no_slow_contact_or_grip(robot):
    assert ('stage','contact') not in robot.trace
    assert not any(c.name=='set_gripper' and c.payload['position']==1. for c in robot.commands)
    assert not any(c.name=='servo_twist' and c.payload['twist'].angular_z for c in robot.commands)


def test_centering_occurs_after_fast_before_slow_and_rechecks_actual_residual():
    robot,ctx,skill=make_case([target(.008,-.005),target(.001,-.0004)])
    result=run(skill,ctx)
    trace=robot.trace
    fast=next(i for i,e in enumerate(trace) if e[0]=='move' and e[1].linear_x<-.01)
    observes=[i for i,e in enumerate(trace) if e==('observe',True)]
    corrections=[(i,e) for i,e in enumerate(trace) if e[0]=='move' and (e[1].linear_y or e[1].linear_z)]
    slow=next(i for i,e in enumerate(trace) if e[0]=='move' and e[1].linear_x==-.004)
    assert fast<observes[0]<corrections[0][0]<observes[1]<slow
    assert len(corrections)==2
    assert corrections[0][1][1].linear_y*corrections[0][1][2]==pytest.approx(.008)
    assert corrections[1][1][1].linear_z*corrections[1][1][2]==pytest.approx(-.005)
    assert result.details['precontact_centering']['corrections']==1
    assert result.details['precontact_centering']['status']=='VERIFIED'
    assert result.details['precontact_centering']['final_error_base_m']['y']==.001
    assert result.details['turn_angle_rad']==pytest.approx(math.pi/3)
    rotation=next(e for e in trace if e[0]=='move' and e[1].angular_z)
    assert rotation[1].angular_z*rotation[2]==pytest.approx(math.pi/3)
    assert not ctx.precontact_centering


def test_already_centered_goes_to_contact_without_zero_motion():
    robot,ctx,skill=make_case([target(.001,.001)])
    result=run(skill,ctx)
    assert result.details['precontact_centering']['corrections']==0
    assert not any(e[0]=='check' for e in robot.trace)


@pytest.mark.parametrize('near',[
    [None], [RuntimeError('No complete dark circular body')],
    [target(.016,0)], [target(.014,.014)], [target(float('nan'),0)],
    [target(fresh_sampling={'enabled':True,'confirmations':1})],
    [target(alignment_reference='desired_pixel')],
    [target(.006),target(.006),target(.006)],
    [target(.012),target(.012)],
])
def test_failed_evidence_or_nonconvergence_never_advances_or_closes(near):
    robot,ctx,skill=make_case(near)
    with pytest.raises(RuntimeError):run(skill,ctx)
    no_slow_contact_or_grip(robot)
    assert not ctx.precontact_centering


def test_joint_preflight_rejection_blocks_contact_and_lateral_motion():
    robot,ctx,skill=make_case([target(.006)])
    robot.preflight_movel_alignment=Mock(side_effect=RuntimeError('joint limit'))
    with pytest.raises(RuntimeError,match='joint limit'):run(skill,ctx)
    no_slow_contact_or_grip(robot)
    assert not any(e[0]=='move' and e[1].linear_y for e in robot.trace)


def test_success_after_two_bounded_corrections_requires_third_observation():
    robot,ctx,skill=make_case([target(.007),target(.003),target(.0002)])
    result=run(skill,ctx)
    assert result.details['precontact_centering']['corrections']==2
    assert len(result.details['precontact_centering']['history'])==3


@pytest.mark.parametrize('change',[{'precontact_centering_tolerance_m':float('nan')},
    {'precontact_centering_speed_mps':.1},{'intermediate_check_distance_m':.2},
    {'precontact_centering_max_corrections':0},{'align_axes':['x','y','z']}])
def test_invalid_policy_rejected_before_any_robot_action(change):
    robot,ctx,skill=make_case([],**change)
    with pytest.raises(ValueError):run(skill,ctx)
    assert robot.commands==[]


def test_disabled_mode_preserves_old_flow_and_does_not_request_near_frames():
    robot,ctx,skill=make_case([],precontact_centering=False)
    result=run(skill,ctx)
    assert 'precontact_centering' not in result.details
    assert not any(e==('observe',True) for e in robot.trace)
    assert result.success


def test_near_refresh_saves_images_without_relaxing_constraints_or_mutating_step():
    import copy
    ctx=SkillContext(Robot(),precontact_centering=True,relax_vision=True)
    step={'target':{'source':'camera','detection':'right','min_depth_m':.1,
                    'workspace_bounds':{'y':[-.4,.4]},'detection_config':{'roi':[1,2,3,4]}}}
    before=copy.deepcopy(step);new=_step_for_refresh(step,ctx)
    assert new['target'].pop('save_debug_images') is True
    assert new==before==step


def test_task_config_only_enables_new_stage_for_right_knob():
    root=Path(__file__).resolve().parents[1]
    cfg=yaml.safe_load((root/'config/ros2_qr_panel_layout_two_knobs.yaml').read_text())
    steps={s['name']:s for s in cfg['sequence']}
    assert steps['upper_row_right_black_knob']['params']['precontact_centering'] is True
    built=_build_skill_config(KnobCameraServoConfig,steps['upper_row_right_black_knob']['params'],ContactMotionConfig())
    assert built.turn_angle_rad==pytest.approx(math.pi/3)
    assert not steps['middle_black_knob']['params'].get('precontact_centering',False)
    assert steps['middle_black_knob']['params']['turn_angle_rad']==1.5708
    assert cfg['sequence'][-1]['name']=='safe_pose'


def test_sequence_runner_passes_near_capture_flag_through_actual_refresher():
    from rg_cabinet.task_runner import run_sequence
    root=Path(__file__).resolve().parents[1]
    cfg=yaml.safe_load((root/'config/ros2_qr_panel_layout_two_knobs.yaml').read_text())
    step=next(s for s in cfg['sequence'] if s['name']=='upper_row_right_black_knob')
    robot=Robot();near_calls=[];calls=[]
    def resolver(incoming):
        calls.append(incoming)
        if incoming['target'].get('save_debug_images'):
            near_calls.append(incoming)
            return target(.006 if len(near_calls)==1 else .0001)
        return target()
    with patch('rg_cabinet.skills.precontact_centering.time.sleep'):
        result=run_sequence({'sequence':[step]},robot,target_resolver=resolver)
    assert len(calls)==3 and len(near_calls)==2
    assert all(s['target']['detection']=='upper_row_right_black_knob' for s in near_calls)
    assert not step['target'].get('save_debug_images',False)
    assert result[0]['success']
    assert result[0]['details']['precontact_centering']['status']=='VERIFIED'
