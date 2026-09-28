# VLA-Thinker 子目标消息
# action 类型: detect / move_to / grasp / release / look_at
string action

# 操作对象描述, e.g. "cup"
string target

# 可选目标位姿（对 move_to 有效）
geometry_msgs/Pose pose

# 可选图像关注区域，归一化坐标 [x1, y1, x2, y2]（对 look_at 有效）
float32[4] region

# 额外参数（JSON字符串，兜底扩展）
string params
