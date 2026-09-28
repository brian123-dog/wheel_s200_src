"""Use measured failures with a position-feedback plant, not pre-canned success."""
from pathlib import Path
import pytest,yaml
from test_precontact_centering import make_case,target,run,no_slow_contact_or_grip


def configured_case(outcomes):
    root=Path(__file__).resolve().parents[1]
    cfg=yaml.safe_load((root/'config/ros2_qr_panel_layout_two_knobs.yaml').read_text())
    params=next(s['params'] for s in cfg['sequence'] if s['name']=='upper_row_right_black_knob')
    keys=['precontact_centering_max_axis_m','precontact_centering_max_total_m',
          'precontact_centering_max_error_m','precontact_centering_max_corrections']
    return make_case(outcomes,**{k:params[k] for k in keys})


@pytest.mark.parametrize('dy,dz',[
    (-.010051,.018175),  # 13:24 failure
    (-.012423,.020619),  # 13:28 failure
    (-.013995,.022921),  # 13:32 failure
    (.020,.020),        # exact cumulative budget
])
def test_actual_error_converges_via_bounded_steps_and_fresh_feedback(dy,dz):
    robot,ctx,skill=configured_case([])
    anchor=[]
    def refresh():
        robot.trace.append(('observe',ctx.precontact_centering))
        if not ctx.precontact_centering:return target()
        if not anchor:anchor.extend([robot.current_pose.y+dy,robot.current_pose.z+dz])
        return target(anchor[0]-robot.current_pose.y,anchor[1]-robot.current_pose.z)
    ctx.target_refresher=refresh
    result=run(skill,ctx)
    report=result.details['precontact_centering']
    assert report['corrections']==2 and report['status']=='VERIFIED'
    assert report['total_travel_m']==pytest.approx(abs(dy)+abs(dz))
    assert max(abs(report['final_error_base_m'][a]) for a in ['y','z'])<=.002
    moves=[e for e in robot.trace if e[0]=='move' and (e[1].linear_y or e[1].linear_z)]
    assert all(abs(e[1].linear_y*e[2])<=.015000001 and abs(e[1].linear_z*e[2])<=.015000001 for e in moves)
    observations=[i for i,e in enumerate(robot.trace) if e==('observe',True)]
    assert len(observations)==3
    assert observations[-1]<robot.trace.index(('stage','contact'))
    assert report['history'][0]['step_limited'] is True


def test_second_correction_uses_new_error_and_can_reverse_direction():
    robot,ctx,skill=configured_case([target(-.014,.023),target(.004,.005),target()])
    result=run(skill,ctx)
    report=result.details['precontact_centering']
    assert report['history'][1]['move_delta_base_m']=={'x':0.,'y':.004,'z':.005}
    assert report['total_travel_m']==pytest.approx(.038)


@pytest.mark.parametrize('outcomes',[
    [target(0.,.0301)],
    [target(.025,.025)],
    [target(-.014,.023),target(.012,0.)],
    [target(-.014,.023),target(.003,0.),target(.003,0.)],
    [target(-.014,.023),RuntimeError('frame rejected after first correction')],
])
def test_limits_or_failed_postmove_observation_never_allow_contact(outcomes):
    robot,ctx,skill=configured_case(outcomes)
    with pytest.raises(RuntimeError):run(skill,ctx)
    no_slow_contact_or_grip(robot)


@pytest.mark.parametrize('capture',[float('nan'),float('inf'),.05,.001,True])
def test_bad_capture_envelope_fails_before_motion(capture):
    robot,ctx,skill=make_case([],precontact_centering_max_error_m=capture)
    with pytest.raises(ValueError):run(skill,ctx)
    assert robot.commands==[]
