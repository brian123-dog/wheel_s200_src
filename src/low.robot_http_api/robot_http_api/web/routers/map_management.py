"""map_management.py —— P2 地图管理 REST 端点。

操作: 扫描文件系统 → 列表/保存/删除/切换地图文件。
"""

import os
import glob
from fastapi import APIRouter, Request, HTTPException
from pydantic import BaseModel

router = APIRouter(prefix="/api/maps", tags=["Maps"])

DEFAULT_MAP_DIR = os.path.expanduser("~/wheel_S200_src/maps")


class SaveMapRequest(BaseModel):
    name: str


class ChangeMapRequest(BaseModel):
    map_name: str


def _get_map_dir(node) -> str:
    try:
        return node.get_parameter('map_directory').get_parameter_value().string_value
    except Exception:
        return DEFAULT_MAP_DIR


@router.get("")
async def list_maps(request: Request):
    map_dir = _get_map_dir(request.app.state.ros2_interface._node)
    maps = []
    if os.path.isdir(map_dir):
        for yaml_file in sorted(glob.glob(os.path.join(map_dir, "*.yaml"))):
            name = os.path.splitext(os.path.basename(yaml_file))[0]
            maps.append({"name": name, "path": yaml_file})
    return {"code": 0, "message": "success", "data": {"maps": maps}}


@router.post("/save")
async def save_map(request: Request, body: SaveMapRequest):
    ros2 = request.app.state.ros2_interface
    try:
        result = ros2.save_map(body.name)
        return {"code": 0, "message": "success", "data": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/{map_name}")
async def delete_map(request: Request, map_name: str):
    map_dir = _get_map_dir(request.app.state.ros2_interface._node)
    yaml_path = os.path.join(map_dir, f"{map_name}.yaml")
    pgm_path = os.path.join(map_dir, f"{map_name}.pgm")

    deleted = []
    for p in [yaml_path, pgm_path]:
        if os.path.exists(p):
            os.remove(p)
            deleted.append(p)

    if not deleted:
        raise HTTPException(status_code=404, detail=f"map {map_name} not found")
    return {"code": 0, "message": f"deleted {len(deleted)} files", "data": {"deleted": deleted}}


@router.post("/change")
async def change_map(request: Request, body: ChangeMapRequest):
    raise HTTPException(
        status_code=501,
        detail="change_map 需要重启 Nav2 (带新 map 参数), 请通过 ros2 launch 或在节点上操作",
    )
