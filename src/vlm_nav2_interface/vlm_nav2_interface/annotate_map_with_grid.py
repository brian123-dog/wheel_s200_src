#!/usr/bin/env python3
"""
annotate_map_with_grid.py
读取Nav2地图，绘制世界坐标网格，供VLM直接读取世界坐标
"""

import os
import yaml
import cv2
import numpy as np


def annotate_map_with_grid(map_yaml_path, output_path=None, grid_interval=1.0):
    """
    在地图图片上绘制世界坐标网格
    
    Args:
        map_yaml_path: Nav2地图yaml路径
        output_path: 输出图片路径，默认在yaml同目录下生成 xxx_grid.png
        grid_interval: 网格间隔（米），默认1.0米
    """
    # 读取地图元数据
    with open(map_yaml_path, 'r') as f:
        map_meta = yaml.safe_load(f)
    
    resolution = map_meta['resolution']
    origin_x, origin_y, _ = map_meta['origin']
    
    # 处理图片路径
    image_path = map_meta['image']
    if not os.path.isabs(image_path):
        yaml_dir = os.path.dirname(os.path.abspath(map_yaml_path))
        image_path = os.path.join(yaml_dir, image_path)
    
    # 读取灰度图
    img_gray = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img_gray is None:
        raise FileNotFoundError(f"无法读取地图图片: {image_path}")
    
    height, width = img_gray.shape
    img_color = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
    
    # 计算世界坐标范围
    world_x_min = origin_x
    world_x_max = origin_x + width * resolution
    world_y_min = origin_y
    world_y_max = origin_y + height * resolution
    
    print(f"[Annotate] 地图尺寸: {width}x{height} 像素")
    print(f"[Annotate] 分辨率: {resolution} m/px")
    print(f"[Annotate] 世界坐标范围: X[{world_x_min:.2f}, {world_x_max:.2f}], Y[{world_y_min:.2f}, {world_y_max:.2f}]")
    
    # 计算原点像素坐标
    origin_px = int((0.0 - origin_x) / resolution)
    origin_py = int(height - (0.0 - origin_y) / resolution)  # Y轴翻转
    
    # 绘制网格线（保持原有风格）
    x_start = np.ceil(world_x_min / grid_interval) * grid_interval
    x_end = np.floor(world_x_max / grid_interval) * grid_interval
    y_start = np.ceil(world_y_min / grid_interval) * grid_interval
    y_end = np.floor(world_y_max / grid_interval) * grid_interval
    
    # 垂直线（x = const）
    for wx in np.arange(x_start, x_end + grid_interval / 2, grid_interval):
        px = int((wx - origin_x) / resolution)
        if 0 <= px < width:
            cv2.line(img_color, (px, 0), (px, height), (180, 180, 180), 1)
            # 顶部标注
            label = f"{wx:.1f}"
            cv2.putText(img_color, label, (px + 2, 12),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
    
    # 水平线（y = const）
    for wy in np.arange(y_start, y_end + grid_interval / 2, grid_interval):
        py = int(height - (wy - origin_y) / resolution)
        if 0 <= py < height:
            cv2.line(img_color, (0, py), (width, py), (180, 180, 180), 1)
            # 左侧标注
            label = f"{wy:.1f}"
            cv2.putText(img_color, label, (2, py - 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

    # 坐标轴细化：仅调整轴线绘制，不改变底图风格
    axis_thickness = 1

    # 标记原点 (0, 0)
    if 0 <= origin_px < width and 0 <= origin_py < height:
        cv2.drawMarker(img_color, (origin_px, origin_py), (0, 0, 255),
                      cv2.MARKER_CROSS, 28, 2, cv2.LINE_AA)
        cv2.putText(img_color, "ORIGIN(0,0)", (origin_px + 12, origin_py - 12),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        
        # X轴箭头（世界X+方向 = 图像右方）
        cv2.arrowedLine(img_color, (origin_px, origin_py),
                       (min(origin_px + 70, width - 10), origin_py),
                       (255, 0, 0), axis_thickness, cv2.LINE_AA, tipLength=0.25)
        cv2.putText(img_color, "X+", (min(origin_px + 74, width - 18), origin_py + 6),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1, cv2.LINE_AA)
        
        # Y轴箭头（世界Y+方向 = 图像上方）
        cv2.arrowedLine(img_color, (origin_px, origin_py),
                       (origin_px, max(origin_py - 70, 10)),
                       (0, 180, 0), axis_thickness, cv2.LINE_AA, tipLength=0.25)
        cv2.putText(img_color, "Y+", (origin_px + 6, max(origin_py - 74, 14)),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 180, 0), 1, cv2.LINE_AA)
    
    # 保存
    if output_path is None:
        base, _ = os.path.splitext(image_path)
        output_path = base + "_grid.png"
    
    cv2.imwrite(output_path, img_color)
    print(f"[Annotate] 带网格地图已保存: {output_path}")
    
    return {
        'output_path': output_path,
        'origin_pixel': (origin_px, origin_py),
        'world_range': {
            'x': (world_x_min, world_x_max),
            'y': (world_y_min, world_y_max)
        },
        'resolution': resolution,
        'image_size': (width, height)
    }


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("用法: python3 annotate_map_with_grid.py <地图yaml路径> [网格间隔(米)]")
        print("示例: python3 annotate_map_with_grid.py ~/maps/my_map.yaml 0.5")
        sys.exit(1)
    
    yaml_path = sys.argv[1]
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    
    annotate_map_with_grid(yaml_path, grid_interval=interval)