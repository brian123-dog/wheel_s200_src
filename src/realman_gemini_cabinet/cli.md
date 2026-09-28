cd /home/jxs/JxsWorkspace/manipulator_ws
colcon build --base-paths realman_gemini_cabinet --packages-select realman_gemini_cabinet
source install/setup.bash
ros2 launch realman_gemini_cabinet mock_demo.launch.py


source /home/jxs/JxsWorkspace/manipulator_ws/install/setup.bash
ros2 service call /rg_cabinet/run_sequence std_srvs/srv/Trigger "{}"



cd /home/jxs/JxsWorkspace/manipulator_ws
colcon build --base-paths src
source install/setup.bash

# 只跑 mock 上层任务
colcon build --base-paths src --packages-up-to realman_gemini_cabinet

# 要接 Orbbec 相机
colcon build --base-paths src --packages-up-to orbbec_camera realman_gemini_cabinet

# 要接 RealMan bringup / driver
colcon build --base-paths src --packages-up-to rm_bringup rm_driver realman_gemini_cabinet

# 要接 MoveIt2，例如 RM65
colcon build --base-paths src --packages-up-to rm_65_config realman_gemini_cabinet


pgrep -af "ros2 launch realman_gemini_cabinet|rm_driver|rviz2|move_group|component_container|cabinet_task_node"


ros2 launch realman_gemini_cabinet hardware_bringup.launch.py   task_config:=/home/jxs/JxsWorkspace/manipulator_ws/install/realman_gemini_cabinet/share/realman_gemini_cabinet/config/ros2_vision_button_demo.yaml


###  tmux操作指令：
操作	            命令
新建并命名会话	     tmux new -s robot
附加到已有会话	     tmux attach -t robot
附加（自动选最近的）	tmux attach
分离（会话内按键）	  Ctrl-b 然后 d
列出会话	          tmux ls
结束会话	          tmux kill-session -t robot

### 发布导航到点信息

ros2 service call /rg_cabinet/resolve_targets std_srvs/srv/Trigger "{}"

ros2 service call /rg_cabinet/run_sequence_from_qr std_srvs/srv/Trigger "{}"

### 查看位姿信息：

ros2 topic echo --once /rm_driver/udp_arm_position

ros2 launch get_arm_state get_arm_state_demo.launch.py

### 控制夹爪开合
ros2 service call /rg_cabinet/open_gripper std_srvs/srv/Trigger "{}"

ros2 service call /rg_cabinet/close_gripper std_srvs/srv/Trigger "{}"

zip -r realman_gemini_cabinet.zip realman_gemini_cabinet/


### fastapi gpufree
(base) root@gpufree-container:~# conda activate locany
cd ~/eagle/Embodied

uvicorn server:app --host 0.0.0.0 --port 5000



curl.exe -X POST "https://5gsxznrs-84dfx38v-5000.ahrestapi.gpufree.cn:8443/locate" -H "x-api-key: change_this_key" -F "image=@F:\User\Desktop\研电赛文档\_ppt_assets\image12.jpeg" -F "query=the red button" -F "mode=ground"

python src/realman_gemini_cabinet/test_client_draw.py





### test log

sunrise@ubuntu:~/wheel_S200_src$ python test_client_draw.py
python: can't open file '/mnt/data/wheel_S200_src/test_client_draw.py': [Errno 2] No such file or directory
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"the red button","mode":"ground","raw_answer":"<ref>the red button</ref><box><369><719><408><767></box><box><577><712><617><765></box><box><578><823><619><877></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"the red button","x1":236.16,"y1":345.12,"x2":261.12,"y2":368.16,"cx":248.64000000000001,"cy":356.64,"w":24.96,"h":23.04},{"label":"the red button","x1":369.28,"y1":341.76,"x2":394.88,"y2":367.2,"cx":382.08,"cy":354.48,"w":25.6,"h":25.439999999999998},{"label":"the red button","x1":369.91999999999996,"y1":395.03999999999996,"x2":396.15999999999997,"y2":420.96,"cx":383.04,"cy":408.0,"w":26.240000000000002,"h":25.919999999999998}],"stats":"\nStatistic Info, num_tokens=24; generate_time(s)=0.7867; tps=30.5063; forward_step=10; num_boxes=3; bps=3.8133; prefill_time=0.6237; switch_to_ar=2\n"}
Opening image...
Num boxes: 3
Box 0: the red button, (236.16, 345.12), (261.12, 368.16)
Box 1: the red button, (369.28, 341.76), (394.88, 367.2)
Box 2: the red button, (369.91999999999996, 395.03999999999996), (396.15999999999997, 420.96)
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"the middle black knob","mode":"ground","raw_answer":"<ref>the middle black knob</ref><box>None</box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.1715; tps=58.3003; forward_step=3; num_boxes=1; bps=5.8300; prefill_time=0.1352; switch_to_ar=0\n"}
Opening image...
Num boxes: 0
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"the black knob","mode":"ground","raw_answer":"<ref>the black knob</ref><box><716><710><755><783></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"the black knob","x1":458.24,"y1":340.79999999999995,"x2":483.2,"y2":375.84000000000003,"cx":470.72,"cy":358.32000000000005,"w":24.96,"h":35.04}],"stats":"\nStatistic Info, num_tokens=12; generate_time(s)=0.1718; tps=69.8347; forward_step=3; num_boxes=1; bps=5.8196; prefill_time=0.1352; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: the black knob, (458.24, 340.79999999999995), (483.2, 375.84000000000003)
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"meter","mode":"ground","raw_answer":"<ref>meter</ref><box><523><154><634><304></box><box><670><152><784><302></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"meter","x1":334.72,"y1":73.92,"x2":405.76,"y2":145.92,"cx":370.24,"cy":109.92,"w":71.04,"h":72.0},{"label":"meter","x1":428.8,"y1":72.96,"x2":501.76,"y2":144.96,"cx":465.28,"cy":108.96000000000001,"w":72.96000000000001,"h":72.0}],"stats":"\nStatistic Info, num_tokens=16; generate_time(s)=0.3090; tps=51.7733; forward_step=12; num_boxes=2; bps=6.4717; prefill_time=0.1355; switch_to_ar=2\n"}
Opening image...
Num boxes: 2
Box 0: meter, (334.72, 73.92), (405.76, 145.92)
Box 1: meter, (428.8, 72.96), (501.76, 144.96)
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"the left black knob","mode":"ground","raw_answer":"<ref>the left black knob</ref><box>None</box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.1708; tps=58.5313; forward_step=3; num_boxes=1; bps=5.8531; prefill_time=0.1354; switch_to_ar=0\n"}
Opening image...
Num boxes: 0
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"the right black knob","mode":"ground","raw_answer":"<ref>the right black knob</ref><box><716><708><756><782></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"the right black knob","x1":458.24,"y1":339.84,"x2":483.84000000000003,"y2":375.36,"cx":471.03999999999996,"cy":357.6,"w":25.6,"h":35.519999999999996}],"stats":"\nStatistic Info, num_tokens=13; generate_time(s)=0.2032; tps=63.9659; forward_step=5; num_boxes=1; bps=4.9205; prefill_time=0.1366; switch_to_ar=1\n"}
Opening image...
Num boxes: 1
Box 0: the right black knob, (458.24, 339.84), (483.84000000000003, 375.36)
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"the right green knob","mode":"ground","raw_answer":"<ref>the right green knob</ref><box><508><708><548><881></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"the right green knob","x1":325.12,"y1":339.84,"x2":350.72,"y2":422.88,"cx":337.92,"cy":381.36,"w":25.6,"h":83.03999999999999}],"stats":"\nStatistic Info, num_tokens=13; generate_time(s)=0.1729; tps=75.1899; forward_step=3; num_boxes=1; bps=5.7838; prefill_time=0.1350; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: the right green knob, (325.12, 339.84), (350.72, 422.88)
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"the right green knob","mode":"ground","raw_answer":"<ref>the right green knob</ref><box><508><708><548><881></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"the right green knob","x1":325.12,"y1":339.84,"x2":350.72,"y2":422.88,"cx":337.92,"cy":381.36,"w":25.6,"h":83.03999999999999}],"stats":"\nStatistic Info, num_tokens=13; generate_time(s)=0.1717; tps=75.7170; forward_step=3; num_boxes=1; bps=5.8244; prefill_time=0.1349; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: the right green knob, (325.12, 339.84), (350.72, 422.88)
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"meter","mode":"ground","raw_answer":"<ref>meter</ref><box><523><154><634><302></box><box><670><152><782><298></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"meter","x1":334.72,"y1":73.92,"x2":405.76,"y2":144.96,"cx":370.24,"cy":109.44,"w":71.04,"h":71.03999999999999},{"label":"meter","x1":428.8,"y1":72.96,"x2":500.48,"y2":143.04,"cx":464.64,"cy":108.0,"w":71.68,"h":70.08}],"stats":"\nStatistic Info, num_tokens=16; generate_time(s)=0.3079; tps=51.9631; forward_step=12; num_boxes=2; bps=6.4954; prefill_time=0.1349; switch_to_ar=2\n"}
Opening image...
Num boxes: 2
Box 0: meter, (334.72, 73.92), (405.76, 144.96)
Box 1: meter, (428.8, 72.96), (500.48, 143.04)
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":" pointer","mode":"ground","raw_answer":"<ref> pointer</ref><box>None</box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[],"stats":"\nStatistic Info, num_tokens=7; generate_time(s)=0.1693; tps=41.3444; forward_step=3; num_boxes=1; bps=5.9063; prefill_time=0.1350; switch_to_ar=0\n"}
Opening image...
Num boxes: 0
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":" rotary switch","mode":"ground","raw_answer":"<ref> rotary switch</ref><box>None</box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[],"stats":"\nStatistic Info, num_tokens=8; generate_time(s)=0.1697; tps=47.1365; forward_step=3; num_boxes=1; bps=5.8921; prefill_time=0.1353; switch_to_ar=0\n"}
Opening image...
Num boxes: 0
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"knob switch","mode":"ground","raw_answer":"<ref>knob switch</ref><box><714><708><756><783></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"knob switch","x1":456.96,"y1":339.84,"x2":483.84000000000003,"y2":375.84000000000003,"cx":470.4,"cy":357.84000000000003,"w":26.880000000000003,"h":36.0}],"stats":"\nStatistic Info, num_tokens=12; generate_time(s)=0.1721; tps=69.7443; forward_step=3; num_boxes=1; bps=5.8120; prefill_time=0.1353; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: knob switch, (456.96, 339.84), (483.84000000000003, 375.84000000000003)
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"rotary selector switch","mode":"ground","raw_answer":"<ref>rotary selector switch</ref><box><0><0><950><1000></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"rotary selector switch","x1":0.0,"y1":0.0,"x2":608.0,"y2":480.0,"cx":304.0,"cy":240.0,"w":608.0,"h":480.0}],"stats":"\nStatistic Info, num_tokens=13; generate_time(s)=0.2450; tps=53.0653; forward_step=8; num_boxes=1; bps=4.0819; prefill_time=0.1349; switch_to_ar=1\n"}
Opening image...
Num boxes: 1
Box 0: rotary selector switch, (0.0, 0.0), (608.0, 480.0)
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/vision_debug/png/middle_black_knob_20260607_143622_923390_overlay.png
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"meter","mode":"ground","raw_answer":"<ref>meter</ref><box><523><154><634><304></box><box><670><152><783><301></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"meter","x1":334.72,"y1":73.92,"x2":405.76,"y2":145.92,"cx":370.24,"cy":109.92,"w":71.04,"h":72.0},{"label":"meter","x1":428.8,"y1":72.96,"x2":501.12,"y2":144.48,"cx":464.96000000000004,"cy":108.72,"w":72.32000000000001,"h":71.52}],"stats":"\nStatistic Info, num_tokens=16; generate_time(s)=0.3089; tps=51.7884; forward_step=12; num_boxes=2; bps=6.4736; prefill_time=0.1356; switch_to_ar=2\n"}
Opening image...
Num boxes: 2
Box 0: meter, (334.72, 73.92), (405.76, 145.92)
Box 1: meter, (428.8, 72.96), (501.12, 144.48)
Saved result: /home/sunrise/wheel_S200_src/vision_debug/png/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"robot","mode":"ground","raw_answer":"<ref>robot</ref><box><316><362><455><862></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"robot","x1":202.24,"y1":173.76,"x2":291.2,"y2":413.76,"cx":246.72,"cy":293.76,"w":88.96000000000001,"h":240.0}],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.1708; tps=58.5600; forward_step=3; num_boxes=1; bps=5.8560; prefill_time=0.1351; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: robot, (202.24, 173.76), (291.2, 413.76)
ERROR:
Traceback (most recent call last):
  File "/mnt/data/wheel_S200_src/src/realman_gemini_cabinet/test_client_draw.py", line 72, in <module>
    img.save(out_path)
  File "/usr/lib/python3/dist-packages/PIL/Image.py", line 2209, in save
    fp = builtins.open(filename, "w+b")
FileNotFoundError: [Errno 2] No such file or directory: '/home/sunrise/wheel_S200_src/meter_result/debug/freegpu_out_result.png'
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"robot","mode":"ground","raw_answer":"<ref>robot</ref><box><316><362><455><862></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"robot","x1":202.24,"y1":173.76,"x2":291.2,"y2":413.76,"cx":246.72,"cy":293.76,"w":88.96000000000001,"h":240.0}],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.1706; tps=58.6134; forward_step=3; num_boxes=1; bps=5.8613; prefill_time=0.1351; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: robot, (202.24, 173.76), (291.2, 413.76)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"man","mode":"ground","raw_answer":"<ref>man</ref><box><0><412><41><752></box><box><38><402><67><708></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"man","x1":0.0,"y1":197.76,"x2":26.240000000000002,"y2":360.96,"cx":13.120000000000001,"cy":279.35999999999996,"w":26.240000000000002,"h":163.20000000000002},{"label":"man","x1":24.32,"y1":192.96,"x2":42.88,"y2":339.84,"cx":33.6,"cy":266.40000000000003,"w":18.560000000000002,"h":146.88}],"stats":"\nStatistic Info, num_tokens=16; generate_time(s)=0.1890; tps=84.6591; forward_step=4; num_boxes=2; bps=10.5824; prefill_time=0.1350; switch_to_ar=0\n"}
Opening image...
Num boxes: 2
Box 0: man, (0.0, 197.76), (26.240000000000002, 360.96)
Box 1: man, (24.32, 192.96), (42.88, 339.84)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"chair","mode":"ground","raw_answer":"<ref>chair</ref><box><522><498><636><706></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"chair","x1":334.08000000000004,"y1":239.04,"x2":407.04,"y2":338.88,"cx":370.55999999999995,"cy":288.96,"w":72.96000000000001,"h":99.83999999999999}],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.1707; tps=58.5882; forward_step=3; num_boxes=1; bps=5.8588; prefill_time=0.1350; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: chair, (334.08000000000004, 239.04), (407.04, 338.88)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"table","mode":"ground","raw_answer":"<ref>table</ref><box><0><515><328><804></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"table","x1":0.0,"y1":247.20000000000002,"x2":209.92000000000002,"y2":385.92,"cx":104.96000000000001,"cy":316.56,"w":209.92000000000002,"h":138.72}],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.1705; tps=58.6523; forward_step=3; num_boxes=1; bps=5.8652; prefill_time=0.1350; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: table, (0.0, 247.20000000000002), (209.92000000000002, 385.92)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"rubsh","mode":"ground","raw_answer":"<ref>rubsh</ref><box>None</box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[],"stats":"\nStatistic Info, num_tokens=8; generate_time(s)=0.1691; tps=47.3016; forward_step=3; num_boxes=1; bps=5.9127; prefill_time=0.1349; switch_to_ar=0\n"}
Opening image...
Num boxes: 0
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"bottle","mode":"ground","raw_answer":"<ref>bottle</ref><box><683><477><705><540></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"bottle","x1":437.12,"y1":228.95999999999998,"x2":451.2,"y2":259.20000000000005,"cx":444.15999999999997,"cy":244.07999999999998,"w":14.079999999999998,"h":30.240000000000002}],"stats":"\nStatistic Info, num_tokens=11; generate_time(s)=0.2493; tps=44.1151; forward_step=8; num_boxes=1; bps=4.0105; prefill_time=0.1349; switch_to_ar=1\n"}
Opening image...
Num boxes: 1
Box 0: bottle, (437.12, 228.95999999999998), (451.2, 259.20000000000005)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ 
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"half","mode":"ground","raw_answer":"<ref>half</ref><box><0><412><41><754></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"half","x1":0.0,"y1":197.76,"x2":26.240000000000002,"y2":361.92,"cx":13.120000000000001,"cy":279.84,"w":26.240000000000002,"h":164.16000000000003}],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.2163; tps=46.2296; forward_step=6; num_boxes=1; bps=4.6230; prefill_time=0.1353; switch_to_ar=1\n"}
Opening image...
Num boxes: 1
Box 0: half, (0.0, 197.76), (26.240000000000002, 361.92)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"bhalf","mode":"ground","raw_answer":"<ref>balf</ref><box>None</box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[],"stats":"\nStatistic Info, num_tokens=8; generate_time(s)=0.1730; tps=46.2463; forward_step=3; num_boxes=1; bps=5.7808; prefill_time=0.1368; switch_to_ar=0\n"}
Opening image...
Num boxes: 0
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"stand","mode":"ground","raw_answer":"<ref>stand</ref><box><0><452><745><998></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"stand","x1":0.0,"y1":216.96,"x2":476.8,"y2":479.04,"cx":238.4,"cy":348.0,"w":476.8,"h":262.08000000000004}],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.2493; tps=40.1104; forward_step=8; num_boxes=1; bps=4.0110; prefill_time=0.1372; switch_to_ar=1\n"}
Opening image...
Num boxes: 1
Box 0: stand, (0.0, 216.96), (476.8, 479.04)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"mount","mode":"ground","raw_answer":"<ref>mount</ref><box><318><0><422><862></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"mount","x1":203.52,"y1":0.0,"x2":270.08,"y2":413.76,"cx":236.8,"cy":206.88,"w":66.56,"h":413.76}],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.2047; tps=48.8440; forward_step=5; num_boxes=1; bps=4.8844; prefill_time=0.1373; switch_to_ar=1\n"}
Opening image...
Num boxes: 1
Box 0: mount, (203.52, 0.0), (270.08, 413.76)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"box","mode":"ground","raw_answer":"<ref>box</ref><box><566><629><700><794></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"box","x1":362.23999999999995,"y1":301.92,"x2":448.0,"y2":381.12,"cx":405.12,"cy":341.52,"w":85.76,"h":79.2}],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.2421; tps=41.2975; forward_step=8; num_boxes=1; bps=4.1298; prefill_time=0.1349; switch_to_ar=1\n"}
Opening image...
Num boxes: 1
Box 0: box, (362.23999999999995, 301.92), (448.0, 381.12)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"wall","mode":"ground","raw_answer":"<ref>wall</ref><box><0><0><1000><1000></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"wall","x1":0.0,"y1":0.0,"x2":640.0,"y2":480.0,"cx":320.0,"cy":240.0,"w":640.0,"h":480.0}],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.2046; tps=48.8708; forward_step=5; num_boxes=1; bps=4.8871; prefill_time=0.1371; switch_to_ar=1\n"}
Opening image...
Num boxes: 1
Box 0: wall, (0.0, 0.0), (640.0, 480.0)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"bracket","mode":"ground","raw_answer":"<ref>bracket</ref><box><361><242><390><304></box><box><931><67><1000><122></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"bracket","x1":231.04,"y1":116.16,"x2":249.60000000000002,"y2":145.92,"cx":240.32,"cy":131.04000000000002,"w":18.560000000000002,"h":29.759999999999998},{"label":"bracket","x1":595.84,"y1":32.160000000000004,"x2":640.0,"y2":58.56,"cx":617.9200000000001,"cy":45.36,"w":44.160000000000004,"h":26.4}],"stats":"\nStatistic Info, num_tokens=17; generate_time(s)=0.3330; tps=51.0441; forward_step=14; num_boxes=2; bps=6.0052; prefill_time=0.1349; switch_to_ar=2\n"}
Opening image...
Num boxes: 2
Box 0: bracket, (231.04, 116.16), (249.60000000000002, 145.92)
Box 1: bracket, (595.84, 32.160000000000004), (640.0, 58.56)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"support","mode":"ground","raw_answer":"<ref>support</ref><box><318><363><454><861></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"support","x1":203.52,"y1":174.24,"x2":290.56,"y2":413.28,"cx":247.04000000000002,"cy":293.76,"w":87.04,"h":239.04}],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.2423; tps=41.2725; forward_step=8; num_boxes=1; bps=4.1272; prefill_time=0.1351; switch_to_ar=1\n"}
Opening image...
Num boxes: 1
Box 0: support, (203.52, 174.24), (290.56, 413.28)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"bin","mode":"ground","raw_answer":"<ref>bin</ref><box><52><704><125><825></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"bin","x1":33.28,"y1":337.91999999999996,"x2":80.0,"y2":396.0,"cx":56.64,"cy":366.96,"w":46.72,"h":58.08}],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.1706; tps=58.6083; forward_step=3; num_boxes=1; bps=5.8608; prefill_time=0.1349; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: bin, (33.28, 337.91999999999996), (80.0, 396.0)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"wastebasket","mode":"ground","raw_answer":"<ref>wastebasket</ref><box><52><704><125><825></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"wastebasket","x1":33.28,"y1":337.91999999999996,"x2":80.0,"y2":396.0,"cx":56.64,"cy":366.96,"w":46.72,"h":58.08}],"stats":"\nStatistic Info, num_tokens=13; generate_time(s)=0.1716; tps=75.7618; forward_step=3; num_boxes=1; bps=5.8278; prefill_time=0.1348; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: wastebasket, (33.28, 337.91999999999996), (80.0, 396.0)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"litter bin","mode":"ground","raw_answer":"<ref>litter bin</ref><box><52><704><125><825></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"litter bin","x1":33.28,"y1":337.91999999999996,"x2":80.0,"y2":396.0,"cx":56.64,"cy":366.96,"w":46.72,"h":58.08}],"stats":"\nStatistic Info, num_tokens=12; generate_time(s)=0.1721; tps=69.7181; forward_step=3; num_boxes=1; bps=5.8098; prefill_time=0.1351; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: litter bin, (33.28, 337.91999999999996), (80.0, 396.0)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"dumpster","mode":"ground","raw_answer":"<ref>dumpster</ref><box>None</box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[],"stats":"\nStatistic Info, num_tokens=8; generate_time(s)=0.1692; tps=47.2889; forward_step=3; num_boxes=1; bps=5.9111; prefill_time=0.1349; switch_to_ar=0\n"}
Opening image...
Num boxes: 0
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"垃圾桶","mode":"ground","raw_answer":"<ref>垃圾桶</ref><box><52><704><125><825></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"垃圾桶","x1":33.28,"y1":337.91999999999996,"x2":80.0,"y2":396.0,"cx":56.64,"cy":366.96,"w":46.72,"h":58.08}],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.1715; tps=58.2921; forward_step=3; num_boxes=1; bps=5.8292; prefill_time=0.1361; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: 垃圾桶, (33.28, 337.91999999999996), (80.0, 396.0)
ERROR:
Traceback (most recent call last):
  File "/usr/lib/python3/dist-packages/PIL/ImageDraw.py", line 428, in draw_text
    mask, offset = font.getmask2(
AttributeError: 'ImageFont' object has no attribute 'getmask2'

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/mnt/data/wheel_S200_src/src/realman_gemini_cabinet/test_client_draw.py", line 70, in <module>
    draw.text((x1, max(0, y1 - 40)), f"{i}: {label}", fill="red")
  File "/usr/lib/python3/dist-packages/PIL/ImageDraw.py", line 483, in text
    draw_text(ink)
  File "/usr/lib/python3/dist-packages/PIL/ImageDraw.py", line 443, in draw_text
    mask = font.getmask(
  File "/usr/lib/python3/dist-packages/PIL/ImageFont.py", line 158, in getmask
    return self.font.getmask(text, mode)
UnicodeEncodeError: 'latin-1' codec can't encode characters in position 3-5: ordinal not in range(256)
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260527_213110_639838_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"silver square plate","mode":"ground","raw_answer":"<ref>silver square plate</ref><box>None</box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[],"stats":"\nStatistic Info, num_tokens=9; generate_time(s)=0.6333; tps=14.2114; forward_step=3; num_boxes=1; bps=1.5790; prefill_time=0.5887; switch_to_ar=0\n"}
Opening image...
Num boxes: 0
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260605_185314_624660_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"silver square plate","mode":"ground","raw_answer":"<ref>silver square plate</ref><box>None</box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[],"stats":"\nStatistic Info, num_tokens=9; generate_time(s)=0.1710; tps=52.6291; forward_step=3; num_boxes=1; bps=5.8477; prefill_time=0.1355; switch_to_ar=0\n"}
Opening image...
Num boxes: 0
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260605_185314_624660_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"silver square block","mode":"ground","raw_answer":"<ref>silver square block</ref><box>None</box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[],"stats":"\nStatistic Info, num_tokens=9; generate_time(s)=0.1705; tps=52.7934; forward_step=3; num_boxes=1; bps=5.8659; prefill_time=0.1351; switch_to_ar=0\n"}
Opening image...
Num boxes: 0
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260605_185314_624660_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"silver square sheet","mode":"ground","raw_answer":"<ref>silver square sheet</ref><box><0><0><997><998></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"silver square sheet","x1":0.0,"y1":0.0,"x2":638.08,"y2":479.04,"cx":319.04,"cy":239.52,"w":638.08,"h":479.04}],"stats":"\nStatistic Info, num_tokens=12; generate_time(s)=0.2562; tps=46.8414; forward_step=8; num_boxes=1; bps=3.9035; prefill_time=0.1350; switch_to_ar=1\n"}
Opening image...
Num boxes: 1
Box 0: silver square sheet, (0.0, 0.0), (638.08, 479.04)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260605_185314_624660_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"silver square faceplate","mode":"ground","raw_answer":"<ref>silver square faceplate</ref><box><550><0><683><83></box><box><730><0><870><75></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"silver square faceplate","x1":352.0,"y1":0.0,"x2":437.12,"y2":39.84,"cx":394.56000000000006,"cy":19.92,"w":85.12,"h":39.84},{"label":"silver square faceplate","x1":467.2,"y1":0.0,"x2":556.8,"y2":36.0,"cx":512.0,"cy":18.0,"w":89.60000000000001,"h":36.0}],"stats":"\nStatistic Info, num_tokens=19; generate_time(s)=0.2547; tps=74.6010; forward_step=8; num_boxes=2; bps=7.8527; prefill_time=0.1350; switch_to_ar=1\n"}
Opening image...
Num boxes: 2
Box 0: silver square faceplate, (352.0, 0.0), (437.12, 39.84)
Box 1: silver square faceplate, (467.2, 0.0), (556.8, 36.0)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260605_185314_624660_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"silver square metal plate","mode":"ground","raw_answer":"<ref>silver square metal plate</ref><box>None</box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[],"stats":"\nStatistic Info, num_tokens=10; generate_time(s)=0.1704; tps=58.6995; forward_step=3; num_boxes=1; bps=5.8700; prefill_time=0.1349; switch_to_ar=0\n"}
Opening image...
Num boxes: 0
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260605_185314_624660_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"screw-in leveling pad","mode":"ground","raw_answer":"<ref>screw-in leveling pad</ref><box>None</box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[],"stats":"\nStatistic Info, num_tokens=11; generate_time(s)=0.1876; tps=58.6395; forward_step=4; num_boxes=1; bps=5.3309; prefill_time=0.1350; switch_to_ar=0\n"}
Opening image...
Num boxes: 0
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260605_185314_624660_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"Selector switch","mode":"ground","raw_answer":"<ref>Selector switch</ref><box><768><394><775><459></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"Selector switch","x1":491.52,"y1":189.12,"x2":496.0,"y2":220.32000000000002,"cx":493.76,"cy":204.72,"w":4.48,"h":31.200000000000003}],"stats":"\nStatistic Info, num_tokens=11; generate_time(s)=0.2195; tps=50.1036; forward_step=6; num_boxes=1; bps=4.5549; prefill_time=0.1350; switch_to_ar=1\n"}
Opening image...
Num boxes: 1
Box 0: Selector switch, (491.52, 189.12), (496.0, 220.32000000000002)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ python src/realman_gemini_cabinet/test_client_draw.py
Script started
Python: /usr/bin/python
Image path: /home/sunrise/wheel_S200_src/meter_results/debug/upper_meters_20260605_185314_624660_result.jpg
Image exists: True
Sending request...
HTTP status: 200
Response text:
{"query":"Changeover switch","mode":"ground","raw_answer":"<ref>Changeover switch</ref><box><477><375><555><485></box><|im_end|>","original_width":640,"original_height":480,"inference_width":640,"inference_height":480,"max_side":768,"boxes":[{"label":"Changeover switch","x1":305.28,"y1":180.0,"x2":355.20000000000005,"y2":232.79999999999998,"cx":330.24,"cy":206.4,"w":49.92,"h":52.8}],"stats":"\nStatistic Info, num_tokens=12; generate_time(s)=0.1714; tps=70.0004; forward_step=3; num_boxes=1; bps=5.8334; prefill_time=0.1349; switch_to_ar=0\n"}
Opening image...
Num boxes: 1
Box 0: Changeover switch, (305.28, 180.0), (355.20000000000005, 232.79999999999998)
Saved result: /home/sunrise/wheel_S200_src/meter_results/debug/freegpu_out_result.png
sunrise@ubuntu:~/wheel_S200_src$ 