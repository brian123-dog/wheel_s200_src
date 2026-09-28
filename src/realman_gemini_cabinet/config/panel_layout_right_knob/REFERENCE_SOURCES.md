# 右侧上排黑色旋钮参考图

目标 ID：`upper_row_right_black_knob`。中间旋钮不读取本目录。

- `template_observation.json` → `reference_observation.png`；来源：`/home/sunrise/wheel_S200_src/vision_debug/upper_row_right_black_knob_20260923_211211_172262_failed_color.png`
- `template_upper_row_right.json` → `reference_left.png`；来源：`middle_black_knob_20260923_150424_946633_panel_layout.json`
- `template_upper_row_right_right.json` → `reference_right.png`；来源：`middle_black_knob_20260923_153749_279208_panel_layout.json`
- `template_upper_row_right_smallest.json` → `reference_smallest.png`；来源：`middle_black_knob_20260922_174825_451699_panel_layout.json`
- `template_user_front_1.json` → `reference_user_front_1.png`；来源：`/home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260606_191706_216943_overlay.png`
- `template_user_front_2.json` → `reference_user_front_2.png`；来源：`/home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260606_194959_166161_overlay.png`
- `template_user_front_3.json` → `reference_user_front_3.png`；来源：`/home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_125702_139651_overlay.png`
- `template_user_front_4.json` → `reference_user_front_4.png`；来源：`/home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_142323_910530_overlay.png`

观测位模板来自 2026-09-23 21:12:08 左右采样的原始 RGB，不是本次重新拍摄：尝试实时采样时相机节点已退出。
四张 user_front 图来自用户指定的 PNG 目录，保留原文件内容，但人工标记区域及其描述子邻域不参与匹配。

另保留 `reference_distant_context.png` 远景用于查看布局；旋钮在源图中半径不足 8 像素，未加入精定位模板列表。
