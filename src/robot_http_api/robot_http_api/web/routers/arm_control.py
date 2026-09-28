"""arm_control.py —— P1 机械臂控制 REST 端点。

依赖: multi_action_node 的 NavArmControl Action Server (/nav_arm_control)。
"""

from fastapi import APIRouter, Request, HTTPException
from pydantic import BaseModel, Field


class ArmRotateRequest(BaseModel):
    degrees: float = Field(default=10.0, ge=-180.0, le=180.0)


class ArmExecuteRequest(BaseModel):
    waypoint_index: int = 0
    run_full_task: bool = False


class GripperRequest(BaseModel):
    action: str = Field(default="open", pattern="^(open|close)$")


router = APIRouter(prefix="/api/arm", tags=["Arm Control"])


@router.post("/rotate")
async def rotate_end_effector(request: Request, body: ArmRotateRequest):
    """旋转末端执行器 (简化为调 NavArmControl Action)。"""
    ros2 = request.app.state.ros2_interface
    try:
        data = ros2.start_arm_task(waypoint_index=int(body.degrees), run_full_task=False)
        return {"code": 0, "message": "success", "data": data}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/execute_task")
async def execute_arm_task(request: Request, body: ArmExecuteRequest):
    """执行完整机械臂任务流水线。"""
    ros2 = request.app.state.ros2_interface
    try:
        data = ros2.start_arm_task(
            waypoint_index=body.waypoint_index,
            run_full_task=body.run_full_task,
        )
        return {"code": 0, "message": "success", "data": data}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/stop")
async def stop_arm_task(request: Request):
    """取消正在执行的机械臂任务。"""
    ros2 = request.app.state.ros2_interface
    try:
        data = ros2.stop_arm_task()
        return {"code": 0, "message": "success", "data": data}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
