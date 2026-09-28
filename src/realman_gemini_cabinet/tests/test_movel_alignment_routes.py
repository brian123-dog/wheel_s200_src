from dataclasses import replace
from types import SimpleNamespace

import pytest

from rg_cabinet.geometry import Pose
from rg_cabinet.skills.knob_camera_servo import KnobCameraServoConfig, KnobCameraServoSkill
from rg_cabinet_ros2.movel_alignment_check import check_alignment_path
from rg_cabinet_ros2.movel_alignment_routes import search_alignment_route, route_candidates
from rg_cabinet_ros2.alignment_preflight_runtime import AlignmentPreflight


class Model:
    model_source = "test_only"
    limits = [(-180., 180.)]*6
    blocked = True
    collision_below = None

    def forward(self, q, frame):
        return Pose(q[0]/100, q[1]/100, q[2]/100, frame=frame)

    def inverse(self, pose, seed):
        q = [pose.x*100, pose.y*100, pose.z*100, 0., 0., 0.]
        blocked = self.blocked and .002 < pose.y < .008 and pose.z > -.06
        return int(blocked), q

    def self_collision_free(self, q):
        return self.collision_below is None or q[2]/100 >= self.collision_below


def search(model, **kwargs):
    return search_alignment_route(model, Pose(0,0,0), [0.]*6,
                                  [("y", .01), ("z", .005)], **kwargs)


def test_finds_new_movel_route_from_same_branch_and_same_goal():
    result = search(Model(), z_offsets=[-.025, -.05, -.075, -.1], z_bounds=(-.1, .01))
    assert result["status"] == "PASSED"
    assert result["selected"]["moves"] == [("z", -.075), ("y", .01), ("z", .08)]
    assert result["path_check"]["end_pose"][:3] == pytest.approx([0,.01,.005])
    assert result["path_check"]["start_joints_deg"] == [0.]*6
    assert result["path_check"]["self_collision_checked"]
    assert not result["environment_collision_checked"]


def test_original_route_is_kept_when_it_works():
    model = Model();model.blocked = False
    result = search(model, z_bounds=(-.1,.01))
    assert result["selected"]["name"] == "original"
    assert len(result["candidates"]) == 1


def test_clearance_bound_rejects_an_otherwise_feasible_route():
    result = search(Model(), z_offsets=[-.075], z_bounds=(-.05,.01))
    assert result["status"] == "REJECTED"
    assert result["candidates"][-1]["reason"] == "OUTSIDE_SURVEYED_Z_BOUNDS"


def test_self_collision_rejects_kinematically_feasible_detour():
    model=Model();model.collision_below=-.05
    result=search(model, z_offsets=[-.075], z_bounds=(-.1,.01))
    assert result["status"] == "REJECTED"
    assert result["candidates"][-1]["reason"] == "SELF_COLLISION"


def test_candidates_do_not_change_x_depth_or_final_yz():
    for _, moves in route_candidates([('y',-.15),('z',.045)],[-.025,-.1]):
        assert all(a in ('y','z') for a,_ in moves)
        assert sum(d for a,d in moves if a=='y') == pytest.approx(-.15)
        assert sum(d for a,d in moves if a=='z') == pytest.approx(.045)


def test_excessive_search_offsets_rejected():
    with pytest.raises(ValueError):search(Model(),z_offsets=[-.8])


class SimRobot:
    def __init__(self):
        self.pose=Pose(0,0,0)
        self.q=[0.]*6
        self.commands=[]
        self.stops=0

    def get_tcp_pose(self):return self.pose

    def servo_twist(self,twist,duration_s,frame):
        assert frame=='base'
        self.commands.append(twist.as_list())
        self.pose=self.pose.translated(twist.linear_x*duration_s,
                                       twist.linear_y*duration_s,twist.linear_z*duration_s)
        self.q=[self.pose.x*100,self.pose.y*100,self.pose.z*100,0.,0.,0.]

    def stop(self):self.stops+=1


def runtime(robot,model):
    check=AlignmentPreflight.__new__(AlignmentPreflight)
    check.node=SimpleNamespace(last_alignment_preflight_debug=None,_publish_status=lambda _:None)
    check._snapshot=lambda timeout=2: (robot.pose,list(robot.q))
    def compute(moves,settings,reroute=False,self_collision=False):
        if reroute:
            result=search_alignment_route(model,robot.pose,robot.q,moves,
                                          z_offsets=[-.075,-.098],z_bounds=(-.1,.01))
            result.update(start_pose=robot.pose.as_list(),start_joints_deg=list(robot.q),
                          base_frame=robot.pose.frame)
        else:
            result=check_alignment_path(model,robot.pose,robot.q,moves,
                                        state_validator=model.self_collision_free)
        if result['status']!='PASSED':raise RuntimeError(result['reason'])
        return result
    check.check=compute
    return check


def test_execution_actually_uses_selected_movel_detour_and_reaches_original_goal():
    robot=SimRobot();checker=runtime(robot,Model())
    report=checker.execute_route(robot,[('y',.01),('z',.005)],KnobCameraServoConfig())
    assert report['execution_status']=='COMPLETED'
    assert len(robot.commands)==3
    assert robot.commands[0][2]<0
    assert robot.commands[1][1]>0
    assert robot.commands[2][2]>0
    assert robot.pose.as_list()[:3] == pytest.approx([0,.01,.005])


def test_no_solution_sends_no_motion():
    robot=SimRobot();model=Model();model.collision_below=-.01
    with pytest.raises(RuntimeError,match='NO_VALID_ROUTE'):
        runtime(robot,model).execute_route(robot,[('y',.01),('z',.005)],KnobCameraServoConfig())
    assert robot.commands==[]


def test_feedback_failure_stops_before_next_leg():
    robot=SimRobot();checker=runtime(robot,Model());calls=[]
    def wait(*args,**kwargs):
        calls.append(1)
        if len(calls)>1:raise RuntimeError('wrong branch')
        return robot.pose,robot.q
    checker._wait_at=wait
    with pytest.raises(RuntimeError,match='wrong branch'):
        checker.execute_route(robot,[('y',.01),('z',.005)],KnobCameraServoConfig())
    assert len(robot.commands)==1
    assert checker.node.last_alignment_preflight_debug['execution_status']=='FAILED'
    assert robot.stops>=1


def test_skill_only_redirects_alignment_when_new_mode_enabled():
    calls=[]
    robot=SimpleNamespace(execute_movel_alignment_route=lambda m,s:calls.append(m))
    cfg=KnobCameraServoConfig(alignment_reroute=True,alignment_reroute_drop_clearance_m=.1)
    KnobCameraServoSkill(cfg)._servo_alignment(robot,{'x':0,'y':-.15,'z':.045})
    assert calls==[[('y',-.15),('z',.045)]]
    assert not KnobCameraServoConfig().alignment_reroute
