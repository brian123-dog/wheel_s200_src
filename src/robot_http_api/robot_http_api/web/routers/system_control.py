"""system_control.py —— P2 系统控制 REST 端点。"""

import os
import subprocess
import platform
from fastapi import APIRouter, Request, HTTPException
from pydantic import BaseModel, Field

router = APIRouter(prefix="/api/system", tags=["System"])


class SetMaxSpeedRequest(BaseModel):
    max_speed: float = Field(gt=0.0, le=5.0)
    max_reverse_speed: float = Field(default=0.5, ge=0.0, le=5.0)


@router.get("/info")
async def system_info(request: Request):
    ros2 = request.app.state.ros2_interface
    node = ros2._node
    return {
        "code": 0, "message": "success",
        "data": {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            "task_status": ros2._task_status,
            "current_task_id": ros2._current_task_id,
        },
    }


@router.post("/poweroff")
async def poweroff(request: Request):
    try:
        subprocess.Popen(["sudo", "shutdown", "-h", "now"])
        return {"code": 0, "message": "shutting down"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/set_max_speed")
async def set_max_speed(request: Request, body: SetMaxSpeedRequest):
    """通过 ros2 param set 动态修改 Nav2 controller 最大速度。"""
    try:
        subprocess.run(
            ["ros2", "param", "set", "/controller_server",
             "FollowPath.max_vel_x", str(body.max_speed)],
            capture_output=True, timeout=5,
        )
        return {"code": 0, "message": "speed updated", "data": {"max_speed": body.max_speed}}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
