# 接触前居中专用参考图

仅用于快速接近结束后的 precontact_centering。远处观察继续使用原 8 张参考。
reference.png 来自 20260924_125537_466993_failed_color.png 原始 RGB。
锚点为近处可见的按钮和绿色开关；排除转动的中间手柄、目标本身、下排及夹爪。
沿用全局至少 20 内点、0.45 内点率、二维锚点跨度等原始质量门限。
同次失败前四帧作为独立于参考图的留出回放，不能视为不同停靠姿态的实机验证。
完整来源和 SHA256 见 template.json。


2026-09-24 14:01 view supplement: template_1401.json / reference_1401.png.
The fifth accepted raw frame is the reference; the first four rejected raw
frames are held out as tests/data/right_knob_near_1401_holdout_*.png.
No global or local quality gate is lowered. With the two-reference bank,
held-out frames 1, 3 and 4 pass; frame 2 still fails anchor coverage.
Frames 3 and 4 have centers about 0.46 px apart (2D replay only).
The template is enabled only during right-knob precontact centering.
Middle-knob sampling in the two-knob mode now has a 12 s total budget;
its two-frame confirmation, freshness and consistency thresholds are unchanged.

2026-09-24 14:17 view supplement: template_1417.json / reference_1417.png.
Source is the fifth failed raw frame from task 20260924_141735_018017_a2e886ab.
Visual inspection showed a complete black body; Otsu merged its right-hand
gray bezel shadow with the body. A reference-local body_threshold_scale=0.8
extracts darker pixels before fitting. Existing references default to 1.0.
The 78% boundary coverage, 20% maximum gap, color/contrast, radius/center and
ambiguity checks remain unchanged. Scale is validated in [0.5, 1.0].
The other four task frames are regression replay images, not independent
validation of this segmentation setting. New live views remain to be tested.
