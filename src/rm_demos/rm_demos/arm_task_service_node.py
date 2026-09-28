#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPlanningScene
from moveit_msgs.msg import RobotTrajectory, Constraints, JointConstraint, PlanningScene, AttachedCollisionObject, CollisionObject, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Int32  # [新增] 引入 Int32
from std_srvs.srv import Empty
from base_interfaces.srv import ArmTaskExecute
import math
import time
import copy
import socket
import json
import threading

import numpy as np
from tf2_ros import TransformListener, Buffer

# ================= 真机硬件配置 =================
PLANNING_GROUP = "rm_group"   
BASE_LINK      = "base_link"  
END_EFFECTOR   = "link6"      

# 夹爪 Socket 参数
GRIPPER_IP = '192.168.1.18'
GRIPPER_PORT = 8080
DEVICE_ID = 1
POS_OPEN =  3300  
POS_CLOSE = 4700  

# ================= 运动速度配置 =================
# 1.0 为默认速度，>1.0 加速，<1.0 减速
# 建议范围: 0.1 ~ 5.0 (取决于机器人及 MoveIt 限制)
SPEED_SCALE = 1.0
ACCEL_SCALE = 1.0  # [新增] 独立控制加速度，建议略低于 SPEED_SCALE 以减少抖动

# ================= 场景参数 =================
CHASSIS_HEIGHT = 0.0
JOINT_GAP      = 0.00
TUBE_HEIGHT    = 0.0
STEP_HEIGHT    = 0.130  
CUP_HEIGHT     = 0.07
CUP_RADIUS     = 0.03
CUP_GAP        = 0.12
CUP_COUNT      = 5   
BASE_LEN  = 0.4843 - 0.013
TOP_LEN   = 0.20
THICKNESS = 0.005
GRIPPER_LENGTH = 0.186 
PLATFORM_OFFSET = 0.00 

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# ... [省略了数学辅助函数 slerp, get_arc_waypoints, get_arc_waypoints_1 以节省篇幅，保持原样] ...
# (请确保这里的数学函数与你之前提供的一致)
def slerp(q1, q2, t):
    q1 = np.array(q1); q2 = np.array(q2)
    cos_theta = np.dot(q1, q2)
    if cos_theta < 0.0: q1 = -q1; cos_theta = -cos_theta
    if cos_theta > 0.9995: return (1.0 - t) * q1 + t * q2
    theta = math.acos(cos_theta)
    return (math.sin((1.0 - t) * theta) * q1 + math.sin(t * theta) * q2) / math.sin(theta)

def get_arc_waypoints(start_pose, mid_pose, end_pose, num_points=50):
    """ 
    改进版：单次扫描生成圆弧 (消除中间点停顿)
    """
    # 1. 转为 numpy 数组
    p1 = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
    p2 = np.array([mid_pose.position.x, mid_pose.position.y, mid_pose.position.z])
    p3 = np.array([end_pose.position.x, end_pose.position.y, end_pose.position.z])

    # 2. 三点定圆 (计算圆心 center 和 法向量 n)
    v1 = p2 - p1
    v2 = p3 - p2
    n = np.cross(v1, v2)
    
    if np.linalg.norm(n) < 1e-6:
        print("❌ 三点共线，无法构成圆弧")
        return []
    n = n / np.linalg.norm(n) # 单位法向量

    # 计算圆心
    m1 = (p1 + p2) / 2
    m2 = (p2 + p3) / 2
    dir1 = np.cross(v1, n)
    dir2 = np.cross(v2, n)
    
    A = np.vstack((dir1, -dir2)).T
    b = m2 - m1
    try:
        x = np.linalg.lstsq(A, b, rcond=None)[0]
        center = m1 + x[0] * dir1
    except:
        return []

    # 3. 准备向量
    vec_s = p1 - center # 起点向量
    vec_m = p2 - center # 中间点向量
    vec_e = p3 - center # 终点向量
    # radius = np.linalg.norm(vec_s)

    # 4. 计算总夹角 (Total Angle)
    def get_angle(v_a, v_b, n_axis):
        # 计算带符号角度
        cross = np.cross(v_a, v_b)
        sin_val = np.linalg.norm(cross) * np.sign(np.dot(cross, n_axis))
        cos_val = np.dot(v_a, v_b)
        return math.atan2(sin_val, cos_val)

    # === 稳健方案：总角度由累加得出 ===
    raw_angle1 = math.atan2(np.linalg.norm(np.cross(vec_s, vec_m)), np.dot(vec_s, vec_m))
    if np.dot(np.cross(vec_s, vec_m), n) < 0: raw_angle1 = 2*math.pi - raw_angle1

    raw_angle2 = math.atan2(np.linalg.norm(np.cross(vec_m, vec_e)), np.dot(vec_m, vec_e))
    if np.dot(np.cross(vec_m, vec_e), n) < 0: raw_angle2 = 2*math.pi - raw_angle2
    
    total_angle = raw_angle1 + raw_angle2

    # 5. 生成插值点
    wps = []
    q_start = [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w]
    q_end   = [end_pose.orientation.x, end_pose.orientation.y, end_pose.orientation.z, end_pose.orientation.w]

    for i in range(num_points):
        # 归一化进度 (0.0 ~ 1.0)
        fraction = (i + 1) / num_points 
        
        # A. 几何旋转 (Rodrigues 旋转公式简化版：在 2D 平面上旋转)
        current_angle = fraction * total_angle
        v_curr = vec_s * math.cos(current_angle) + np.cross(n, vec_s) * math.sin(current_angle)
        pt = center + v_curr
        
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pt[0], pt[1], pt[2]
        
        # B. 姿态插值 (Slerp)
        q_res = slerp(q_start, q_end, fraction)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q_res
        
        wps.append(pose)
        
    return wps


def get_arc_waypoints_1(start_pose, mid_pose, end_pose, num_points=50):
    """ 
    输入: 起点、中间点、终点 (geometry_msgs/Pose)
    输出: 包含 num_points 个 Pose 的列表，形成圆弧轨迹
    """
    # 1. 提取位置 (转化为 numpy 数组)
    p1 = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
    p2 = np.array([mid_pose.position.x, mid_pose.position.y, mid_pose.position.z])
    p3 = np.array([end_pose.position.x, end_pose.position.y, end_pose.position.z])

    # 2. 计算圆心和半径 (三点定圆)
    # 定义两个弦向量
    v1 = p2 - p1
    v2 = p3 - p2
    # 这里的算法利用两弦的中垂线交点即为圆心
    # 为了简化计算，构建局部坐标系
    n = np.cross(v1, v2) # 法向量
    if np.linalg.norm(n) < 1e-6:
        print("❌ 三点共线，无法构成圆弧")
        return [] # 直线不应调用此函数
    n = n / np.linalg.norm(n)
    
    # 求解圆心 (利用两弦中垂面与三点所在平面的交点)
    # 中点
    m1 = (p1 + p2) / 2
    m2 = (p2 + p3) / 2
    # 向量积求解方向
    dir1 = np.cross(v1, n)
    dir2 = np.cross(v2, n)
    
    # 构建方程求解参数 t1, t2: m1 + t1*dir1 = m2 + t2*dir2
    # 这是一个线性方程组
    A = np.vstack((dir1, -dir2)).T
    b = m2 - m1
    # 使用最小二乘法求解 (只取前两维或利用伪逆，因为都在同一平面)
    # 实际上由于是平面几何，这必然有解
    # 简化：直接用 numpy linalg solve 需要降维，这里用伪逆通用性更好
    try:
        x = np.linalg.lstsq(A, b, rcond=None)[0]
        center = m1 + x[0] * dir1
    except:
        return []

    radius = np.linalg.norm(p1 - center)

    # 3. 生成圆弧点
    # 定义从圆心指向各点的向量
    vc1 = p1 - center
    vc3 = p3 - center
    
    # 建立圆平面的基向量 (u, v)
    u = vc1 / np.linalg.norm(vc1)
    w = np.cross(u, n) # 切向向量
    
    # 计算起点和终点的角度 (相对于 u)
    # 起点角度定义为 0
    # 终点角度:
    angle_end = math.atan2(np.dot(vc3, w), np.dot(vc3, u))
    
    # 确保经过中间点 (判断方向)
    vc2 = p2 - center
    angle_mid = math.atan2(np.dot(vc2, w), np.dot(vc2, u))
    
    # 处理角度跨越 (例如从 350度 到 10度)
    if angle_mid < 0 and angle_end > 0 and angle_end > angle_mid: pass # 正常
    # 简单的方向逻辑修正：我们总是希望 angle 顺着 mid 走
    # 这里的逻辑比较复杂，简单处理：如果 mid 的角度不在 0~end 之间，说明可能跨越了周期，需要调整
    # 为保证鲁棒性，使用插值法更直接：
    
    # --- 简化版生成策略 (Slerp 向量) ---
    # 这比纯角度计算更稳定，类似于四元数插值，但在 2D 平面上做
    # 我们不仅位置要插值，姿态(Orientation)也要插值
    
    waypoints = []
    q_start = [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w]
    q_end   = [end_pose.orientation.x, end_pose.orientation.y, end_pose.orientation.z, end_pose.orientation.w]

    # 总弧度
    total_angle = angle_end # 这是基于 u 轴的，可能不准
    # 重新计算更加鲁棒的总夹角 (v1 到 v3)
    # 使用带符号角度
    # 这里我们简单线性分割从 Start -> Mid -> End
    # 为了保证正好经过 Mid，我们分两段生成：Start->Mid 和 Mid->End
    
    # 这里为了代码简洁，使用直接对圆周角进行插值的方法
    # 假设我们走的是劣弧或优弧由 mid 决定
    
    # === 最终采用方案：分两段插值以确保精度 ===
    # 第一段 Start -> Mid
    vec_s = p1 - center
    vec_m = p2 - center
    vec_e = p3 - center
    
    # 计算 Start 到 Mid 的旋转轴和角度
    # 但因为三点共面，轴就是 n
    
    def get_points_on_segment(v_start, v_end, count, q_s, q_e):
        seg_points = []
        # 计算两向量夹角
        dot = np.dot(v_start, v_end) / (np.linalg.norm(v_start)*np.linalg.norm(v_end))
        dot = np.clip(dot, -1.0, 1.0)
        segment_angle = math.acos(dot)
        
        # 判断旋转方向 (叉积与 n 同向为正)
        cross = np.cross(v_start, v_end)
        if np.dot(cross, n) < 0:
            segment_angle = -segment_angle

        for i in range(count):
            fraction = (i + 1) / count # +1 是为了不包含重复起点
            # 罗德里格斯旋转公式 或 简单的平面旋转
            curr_angle = fraction * segment_angle
            # 旋转矩阵 (绕 n 轴)
            # R = I + sin(th)K + (1-cos(th))K^2
            # 简化: v_curr = v_start * cos + (n x v_start) * sin
            v_curr = v_start * math.cos(curr_angle) + np.cross(n, v_start) * math.sin(curr_angle)
            pt = center + v_curr
            
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pt[0], pt[1], pt[2]
            
            # 姿态插值
            q_res = slerp(q_s, q_e, fraction)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q_res
            seg_points.append(pose)
        return seg_points

    # 估算两段步数比例
    dist1 = np.linalg.norm(p2 - p1)
    dist2 = np.linalg.norm(p3 - p2)
    total_dist = dist1 + dist2
    n1 = int(num_points * (dist1 / total_dist))
    n2 = num_points - n1
    
    # 获取 Mid 的姿态四元数
    q_mid = [mid_pose.orientation.x, mid_pose.orientation.y, mid_pose.orientation.z, mid_pose.orientation.w]

    wps = []
    wps.extend(get_points_on_segment(vec_s, vec_m, n1, q_start, q_mid)) # Start -> Mid
    wps.extend(get_points_on_segment(vec_m, vec_e, n2, q_mid, q_end))   # Mid -> End
    
    return wps
# def get_arc_waypoints_1(start_pose, mid_pose, end_pose, num_points=50):
#     # ... (保持之前的 get_arc_waypoints_1 代码不变，此处省略具体实现以聚焦修改点) ...
#     # 请确保这里是你原有的分段插值逻辑
#     return get_arc_waypoints(start_pose, mid_pose, end_pose, num_points) # 临时回退，实际请用你的代码

class RealManSimLogicClient(Node):
    def __init__(self):
        super().__init__('realman_sim_logic_client')
        self._cb_group = ReentrantCallbackGroup()
        # [新增] 取消控制属性（移到父类，因为send_goal_and_wait在这里）
        self._cancel_requested = False
        self._current_goal_handle = None  
        self._current_cup_id = -1
        # --- MoveIt 初始化 ---
        self._action_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self._cb_group)
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory', callback_group=self._cb_group)
        self._cartesian_srv = self.create_client(GetCartesianPath, 'compute_cartesian_path', callback_group=self._cb_group)
        self._scene_srv = self.create_client(GetPlanningScene, 'get_planning_scene', callback_group=self._cb_group)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # --- [新增] 杯子删除指令发布者 ---
        # self.cup_delete_pub = self.create_publisher(Int32, '/delete_cup_id', 10)
        self.cup_delete_pub = self.create_publisher(Int32, '/rm_group/cup_delete', 10)

        # [新增]添加杯子发布者 (Topic 名称需与你的场景节点(RealScenePublisher)对应)
        self.cup_add_pub = self.create_publisher(Int32, '/rm_group/cup_add', 10)    

        # [新增]机械臂状态发布者（供 nav_to_arm_bridge 订阅，建立节点图连线）
        self.arm_status_pub = self.create_publisher(Int32, '/rm_group/arm_status', 10)
        # 发布初始状态，确保话题在 rqt_graph 中可见
        init_msg = Int32()
        init_msg.data = 0
        self.arm_status_pub.publish(init_msg)
        
        self.get_logger().info('>>> 正在连接 MoveIt...')
        move_action_ok = self._action_client.wait_for_server(timeout_sec=8.0)
        execute_ok = self._execute_client.wait_for_server(timeout_sec=8.0)
        cartesian_ok = self._cartesian_srv.wait_for_service(timeout_sec=8.0)
        scene_ok = self._scene_srv.wait_for_service(timeout_sec=8.0)
        self._moveit_ready = move_action_ok and execute_ok and cartesian_ok and scene_ok
        if self._moveit_ready:
            self.get_logger().info('>>> MoveIt 连接成功！')
        else:
            self.get_logger().error(
                f'❌ MoveIt 未就绪: move_action={move_action_ok}, '
                f'execute_trajectory={execute_ok}, compute_cartesian_path={cartesian_ok}, '
                f'get_planning_scene={scene_ok}'
            )

        # --- TF 初始化 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  

        # --- 夹爪 Socket 初始化 ---
        self.gripper_sock = None
        try:
            self.gripper_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.gripper_sock.settimeout(2.0)
            self.gripper_sock.connect((GRIPPER_IP, GRIPPER_PORT))
            self.init_gripper_hardware() 
            self.get_logger().info('>>> 夹爪连接成功！')
        except Exception as e:
            self.get_logger().error(f'❌ 夹爪连接失败: {e}')

    # --- [新增] 删除杯子的辅助函数 ---
    def delete_cup_obstacle(self, cup_id):
        """
        发布指令给 RealScenePublisher 节点，让其移除指定 ID 的杯子
        """
        msg = Int32()
        msg.data = int(cup_id)
        self.cup_delete_pub.publish(msg)
        self.get_logger().info(f"⚡已发送移除指令: Remove Cup {cup_id}")
        
        # ⚠️ 关键: 稍微等待一下，让 RealScenePublisher 接收消息并更新 PlanningScene
        # 否则 MoveIt 可能在下一行代码规划时还认为障碍物存在
        time.sleep(0.2) 

    # --- [新增] 添加杯子的辅助函数 ---
    def add_cup_obstacle(self, cup_id):
        """
        发布指令给 RealScenePublisher 节点，让其重新添加指定 ID 的杯子
        """
        msg = Int32()
        msg.data = int(cup_id)
        
        # 使用刚才定义的 publisher 发送
        self.cup_add_pub.publish(msg)
        
        self.get_logger().info(f"⚡已发送添加指令: Add Cup {cup_id}")
        
        # ⚠️ 关键: 稍微等待一下，让 MoveIt 场景有时间更新
        # 否则如果是最后一个杯子，马上结束程序可能会导致消息没发出去
        time.sleep(0.2)

    # ... [send_json, init_gripper_hardware, control_gripper 保持不变] ...
    def send_json(self, command_dict):
        if not self.gripper_sock: return None
        try:
            cmd_str = json.dumps(command_dict) + '\r\n'
            self.gripper_sock.send(cmd_str.encode('utf-8'))
            return self.gripper_sock.recv(1024).decode()
        except Exception: return None

    def init_gripper_hardware(self):
        self.send_json({"command":"set_tool_voltage","voltage_type":3})
        time.sleep(0.5)
        self.send_json({"command":"set_modbus_mode","port":1,"baudrate":115200,"timeout":2})
        time.sleep(0.5)
        self.send_json({"command":"write_single_register","port":1,"address":257,"data":1,"device":DEVICE_ID}) 
        self.send_json({"command":"write_single_register","port":1,"address":256,"data":1,"device":DEVICE_ID}) 
        time.sleep(1.0)
        self.send_json({"command":"write_single_register","port":1,"address":260,"data":100, "device":DEVICE_ID})
        self.send_json({"command":"write_single_register","port":1,"address":261,"data":50, "device":DEVICE_ID})
        self.send_json({"command":"write_single_register","port":1,"address":262, "data":1000,"device":DEVICE_ID})

    def control_gripper(self, state="open"):
        target = POS_CLOSE if state == "close" else POS_OPEN
        binary_value = bin(target)[2:].zfill(16)
        low, high = int(binary_value[-8:], 2), int(binary_value[:-8], 2)
        cmd = {"command": "write_registers", "port": 1, "address": 258, "num": 2, "data": [0, 0, high, low], "device": DEVICE_ID}
        self.send_json(cmd)
        time.sleep(0.02)
        self.send_json({"command":"write_single_register","port":1,"address":264,"data":1,"device":DEVICE_ID}) 
        return True

    def add_scene_objects(self):
        table = CollisionObject()
        table.id = "table_base"
        table.header.frame_id = BASE_LINK
        table.operation = CollisionObject.ADD
        box = SolidPrimitive(); box.type = SolidPrimitive.BOX; box.dimensions = [2.0, 2.0, 0.1]
        pose = Pose(); pose.position.z = -0.05 - 0.01 
        table.primitives.append(box); table.primitive_poses.append(pose)
        scene_msg = PlanningScene(); scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(table)
        self.scene_pub.publish(scene_msg)
        time.sleep(0.5)
    
    # ... [get_cup_pose, move_linear, move_arc, get_current_pose, move_to_pose, send_goal_and_wait, rotate_end_effector 保持不变] ...
    def get_cup_pose(self, cup_id):
        table_z = CHASSIS_HEIGHT + JOINT_GAP + TUBE_HEIGHT + STEP_HEIGHT + PLATFORM_OFFSET 
        x = -(BASE_LEN + THICKNESS + (TOP_LEN / 2.0))
        total_width = (CUP_COUNT * CUP_RADIUS * 2) + ((CUP_COUNT - 1) * CUP_GAP)
        start_y = -(total_width / 2.0) + CUP_RADIUS
        cup_y = start_y + cup_id * (CUP_RADIUS * 2 + CUP_GAP)
        wrist_x = x + GRIPPER_LENGTH 
        wrist_y = cup_y  
        wrist_z = table_z + (CUP_HEIGHT / 2.0) - 0.01
        q_base = get_quaternion_from_euler(math.pi / 2, -math.pi / 2, -math.pi / 2)
        grasp_pose = Pose()
        grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z = wrist_x, wrist_y, wrist_z
        grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w = q_base
        pose_pre_grasp = copy.deepcopy(grasp_pose)
        pose_pre_grasp.position.x = wrist_x + 0.09
        pose_up_grasp = copy.deepcopy(grasp_pose)
        # pose_up_grasp.position.z = wrist_z + 0.110 
        pose_up_grasp.position.z = wrist_z + 0.130
        return pose_pre_grasp, grasp_pose, pose_up_grasp

    def scale_trajectory(self, trajectory, velocity_scaling=1.0, acceleration_scaling=1.0):
        """
        手动缩放轨迹的速度和加速度 (MoveIt 2 的 GetCartesianPath 在某些版本不支持 scaling_factor)
        """
        # 如果缩放系数极其接近 1.0，直接返回以节省计算
        if abs(velocity_scaling - 1.0) < 1e-4 and abs(acceleration_scaling - 1.0) < 1e-4:
            return trajectory
            
        # 计算时间缩放倍数
        # 如果 velocity_scaling 为 2.0 (想变快)，则时间应变为原来的 0.5 倍
        scale_factor = 1.0 / velocity_scaling
        
        new_traj = copy.deepcopy(trajectory)
        for point in new_traj.joint_trajectory.points:
            # 缩放从起点开始的时间 (time_from_start)
            total_nanos = (point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec) * scale_factor
            point.time_from_start.sec = int(total_nanos // 1_000_000_000)
            point.time_from_start.nanosec = int(total_nanos % 1_000_000_000)
            
            # 缩放速度 (v_new = v_old * velocity_scaling)
            point.velocities = [v * velocity_scaling for v in point.velocities]
            
            # 缩放加速度 (a_new = a_old * acceleration_scaling)
            point.accelerations = [a * acceleration_scaling for a in point.accelerations]
            
        return new_traj

    def move_linear(self, target_pose, vel_scale=SPEED_SCALE, acc_scale=ACCEL_SCALE):
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_LINK
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = PLANNING_GROUP
        req.waypoints = [target_pose]
        req.max_step = 0.02 
        req.jump_threshold = 0.0 
        req.avoid_collisions = True 
        future = self._cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.error_code.val != 1 or response.fraction < 0.90:
            self.get_logger().error(f"❌ 直线规划失败 (覆盖率 {response.fraction*100:.1f}%)")
            return False
        
        # 核心修改：缩放轨迹
        scaled_trajectory = self.scale_trajectory(response.solution, vel_scale, acc_scale)
            
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = scaled_trajectory
        ex_future = self._execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, ex_future)
        goal_handle = ex_future.result()
        if not goal_handle.accepted: return False
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().result.error_code.val == 1
    
    def move_arc(self, mid_pose, end_pose, steps=50, continuous=True, vel_scale=SPEED_SCALE, acc_scale=ACCEL_SCALE):
        start_pose = self.get_current_pose()
        if start_pose is None: return False
        if continuous: waypoints = get_arc_waypoints(start_pose, mid_pose, end_pose, num_points=steps)
        else: waypoints = get_arc_waypoints_1(start_pose, mid_pose, end_pose, num_points=steps)
        if not waypoints: return False
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_LINK
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = PLANNING_GROUP
        req.waypoints = waypoints
        req.max_step = 0.02
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        future = self._cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.error_code.val != 1 or response.fraction < 0.90: return False
        
        # 核心修改：缩放轨迹
        scaled_trajectory = self.scale_trajectory(response.solution, vel_scale, acc_scale)

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = scaled_trajectory
        ex_future = self._execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, ex_future)
        goal_handle = ex_future.result()
        if not goal_handle.accepted: return False
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().result.error_code.val == 1

    def get_current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(BASE_LINK, END_EFFECTOR, rclpy.time.Time())
            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            return p
        except Exception: return None

    def move_to_pose(self, pose):
        if not self._moveit_ready:
            self.get_logger().error("❌ MoveIt 未就绪，无法执行 move_to_pose")
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = PLANNING_GROUP
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 12.0
        goal_msg.request.max_velocity_scaling_factor = 0.9
        goal_msg.request.max_acceleration_scaling_factor = 0.9
        goal_msg.request.start_state.is_diff = True
        pos_con = PositionConstraint()
        pos_con.header.frame_id = BASE_LINK
        pos_con.link_name = END_EFFECTOR
        region = BoundingVolume(); box = SolidPrimitive(); box.type = SolidPrimitive.BOX; box.dimensions = [0.005, 0.005, 0.005]
        region.primitives.append(box); region.primitive_poses.append(pose)
        pos_con.constraint_region = region; pos_con.weight = 1.0
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = BASE_LINK
        ori_con.link_name = END_EFFECTOR
        ori_con.orientation = pose.orientation
        ori_con.absolute_x_axis_tolerance = 0.02
        ori_con.absolute_y_axis_tolerance = 0.02
        ori_con.absolute_z_axis_tolerance = 0.02
        ori_con.weight = 1.0
        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos_con)
        goal_constraints.orientation_constraints.append(ori_con)
        goal_msg.request.goal_constraints.append(goal_constraints)
        path_constraints = Constraints()
        joint_con = JointConstraint()
        joint_con.joint_name = "joint1"  
        joint_con.position = 0.0         
        joint_con.tolerance_above = 1.57 
        joint_con.tolerance_below = 1.57 
        joint_con.weight = 1.0
        path_constraints.joint_constraints.append(joint_con)
        goal_msg.request.path_constraints = path_constraints
        if self.send_goal_and_wait(goal_msg):
            return True

        # 严格约束失败后，自动放宽一次，降低因实机抖动/标定误差导致的规划失败概率
        self.get_logger().warn("⚠️ move_to_pose 严格约束失败，尝试放宽约束重新规划...")
        fallback_goal = MoveGroup.Goal()
        fallback_goal.request.group_name = PLANNING_GROUP
        fallback_goal.request.num_planning_attempts = 30
        fallback_goal.request.allowed_planning_time = 15.0
        fallback_goal.request.max_velocity_scaling_factor = 0.7
        fallback_goal.request.max_acceleration_scaling_factor = 0.7
        fallback_goal.request.start_state.is_diff = True

        pos_con2 = PositionConstraint()
        pos_con2.header.frame_id = BASE_LINK
        pos_con2.link_name = END_EFFECTOR
        region2 = BoundingVolume(); box2 = SolidPrimitive(); box2.type = SolidPrimitive.BOX; box2.dimensions = [0.01, 0.01, 0.01]
        region2.primitives.append(box2); region2.primitive_poses.append(pose)
        pos_con2.constraint_region = region2; pos_con2.weight = 1.0

        ori_con2 = OrientationConstraint()
        ori_con2.header.frame_id = BASE_LINK
        ori_con2.link_name = END_EFFECTOR
        ori_con2.orientation = pose.orientation
        ori_con2.absolute_x_axis_tolerance = 0.08
        ori_con2.absolute_y_axis_tolerance = 0.08
        ori_con2.absolute_z_axis_tolerance = 0.08
        ori_con2.weight = 1.0

        goal_constraints2 = Constraints()
        goal_constraints2.position_constraints.append(pos_con2)
        goal_constraints2.orientation_constraints.append(ori_con2)
        fallback_goal.request.goal_constraints.append(goal_constraints2)

        return self.send_goal_and_wait(fallback_goal)

    def send_goal_and_wait(self, goal_msg):
        """发送目标并等待完成，支持取消检测"""
        # 检查是否已请求取消
        if self._cancel_requested:
            self.get_logger().warn("⛔ 检测到取消请求，中止发送新目标")
            return False
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle or not goal_handle.accepted:
            return False
        
        # 保存当前goal handle以便外部取消
        self._current_goal_handle = goal_handle
        
        # 等待结果，同时检查取消标志
        res_future = goal_handle.get_result_async()
        
        # 使用非阻塞方式等待，可以响应取消
        while rclpy.ok() and not res_future.done():
            if self._cancel_requested:
                self.get_logger().warn("⛔ 任务执行中被取消")
                goal_handle.cancel_goal_async()
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not res_future.done():
            return False
            
        result = res_future.result()
        self._current_goal_handle = None
        return result.result.error_code.val == 1

    def rotate_end_effector(self):
        req = GetPlanningScene.Request()
        req.components.components = req.components.ROBOT_STATE
        future = self._scene_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if not res: return False
        state_name = res.scene.robot_state.joint_state.name
        state_pos = res.scene.robot_state.joint_state.position
        target_joint = "joint6" 
        if "arm_joint6" in state_name: target_joint = "arm_joint6"
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = PLANNING_GROUP
        constraints = Constraints()
        found = False
        for i, name in enumerate(state_name):
            if "joint" in name and "gripper" not in name and "lift" not in name:
                jc = JointConstraint(); jc.joint_name = name; jc.weight = 1.0
                if name == target_joint:
                    current_val = state_pos[i]
                    target_val = current_val - math.pi if current_val > 0 else current_val + math.pi
                    jc.position = target_val
                    jc.tolerance_above = 0.001; jc.tolerance_below = 0.001
                    found = True
                else:
                    jc.position = state_pos[i]
                    jc.tolerance_above = 0.001; jc.tolerance_below = 0.001
                constraints.joint_constraints.append(jc)
        if not found: return False
        goal_msg.request.goal_constraints.append(constraints)
        return self.send_goal_and_wait(goal_msg)
    
    # [新增] 通用重试包装器
    def move_with_retry(self, move_func, target, retries=5, description="运动"):
        """
        包装各种移动函数，自动重试
        :param move_func: 要调用的函数名 (例如 self.move_linear)
        :param target: 目标点 (Pose)
        :param retries: 重试次数
        """
        for i in range(retries):
            if i > 0:
                self.get_logger().warn(f"⚠️ [{description}] 规划失败，正在进行第 {i+1}/{retries} 次重试...")
                time.sleep(2.0) # 稍微歇一下再试
            
            # 调用传入的函数
            if move_func(target):
                return True
        
        self.get_logger().error(f"❌ [{description}] 重试 {retries} 次后彻底失败！")
        return False

    def process_cup(self, cup_id, current_idx, total_count):
        """
        处理单个杯子的抓取、倒水和放回流程
        :param cup_id: 杯子的 ID (用于位置计算和障碍物删除)
        :param current_idx: 当前是第几个杯子 (用于日志显示, 0-indexed)
        :param total_count: 总杯子数 (用于日志显示)
        """
        self._current_cup_id = cup_id  # [新增] 记录当前处理的cup
        
        self.get_logger().info(f"\n========== [Loop] 开始处理第 {current_idx+1} / {total_count} 个杯子 (ID: {cup_id}) ==========")

        # [新增] Step 0: 初始取消检查
        if self._cancel_requested:
            self.get_logger().warn(f"⛔ Cup {cup_id} 在开始前被取消")
            return False

        # 计算点位
        pose_pre_grasp, grasp_pose, pose_up_grasp = self.get_cup_pose(cup_id)

        # ==================== [Step 2] 前往预备点 ====================
        self.get_logger().info(f">>> [ID:{cup_id}] 前往预备位姿...")
        
        # [新增] 运动前检查取消
        if self._cancel_requested:
            self.get_logger().warn(f"⛔ Cup {cup_id} 在前往预备点前被取消")
            return False
        
        if not self.move_with_retry(self.move_to_pose, pose_pre_grasp, retries=5, description="前往预备点"):
            self.get_logger().error(f"❌ [ID:{cup_id}] 连预备点都去不了，跳过此杯子！")
            self.add_cup_obstacle(cup_id)
            return False
        
        # [新增] 运动后检查取消（关键！）
        if self._cancel_requested:
            self.get_logger().warn(f"⛔ Cup {cup_id} 到达预备点后被取消，恢复障碍物并退出")
            self.add_cup_obstacle(cup_id)  # 把障碍物加回去
            return False

        # ================= [关键逻辑] =================
        # 到达 Pre-Grasp 后，立即移除杯子碰撞体
        self.get_logger().info(f">>> [ID:{cup_id}] 到达预备点，正在移除场景障碍物...")
        self.delete_cup_obstacle(cup_id)
        # ============================================

        # [Step 3] 张开夹爪
        self.control_gripper("open")

        # [Step 4] 进场抓取
        self.get_logger().info(f">>> [ID:{cup_id}] 进场抓取...")
        if not self.move_with_retry(self.move_linear, grasp_pose, retries=5, description="正在抓取"):
            self.get_logger().error("❌ 下探失败")
            return False

        # [Step 5] 闭合夹爪
        self.get_logger().info(f">>> [ID:{cup_id}] 闭合夹爪...")
        time.sleep(1.0)
        self.control_gripper("close")
        time.sleep(0.5) 

        # [Step 6] 垂直抬起
        self.get_logger().info(f">>> [ID:{cup_id}] 垂直抬起...")
        if not self.move_linear(pose_up_grasp): 
            return False
        
        # [Step 7] 水平运送 
        self.get_logger().info(f">>> [ID:{cup_id}] 水平运送...")
        mid_2_pose = copy.deepcopy(pose_up_grasp)
        mid_2_pose.position.x = -0.40
        mid_2_pose.position.y = -0.50
        q_mid_2 = get_quaternion_from_euler(math.pi / 2, -math.pi / 2, 0.0)
        mid_2_pose.orientation.x = q_mid_2[0]; mid_2_pose.orientation.y = q_mid_2[1]
        mid_2_pose.orientation.z = q_mid_2[2]; mid_2_pose.orientation.w = q_mid_2[3]

        target_pose_1 = copy.deepcopy(pose_up_grasp)
        target_pose_1.position.x = -0.015
        target_pose_1.position.y = -0.85  
        target_pose_1.position.z = pose_up_grasp.position.z + 0.0
        q_new = get_quaternion_from_euler(math.pi / 2, -math.pi / 2, 0.0)
        target_pose_1.orientation.x = q_new[0]; target_pose_1.orientation.y = q_new[1]
        target_pose_1.orientation.z = q_new[2]; target_pose_1.orientation.w = q_new[3]

        target_pose = copy.deepcopy(target_pose_1) 

        if cup_id < (CUP_COUNT + 1) // 2:
            self.get_logger().info(f"   * ID {cup_id} < {(CUP_COUNT + 1) // 2}, 走直线路径 (Skip Mid)...")
            if not self.move_with_retry(self.move_linear, target_pose_1, retries=5, description="直接移动到终点"):
                self.get_logger().error("❌ 直接移动到终点失败"); return False
        else:
            self.get_logger().info(f"   * ID {cup_id} >= {(CUP_COUNT + 1) // 2}, 走折线路径 (Via Mid)...")
            if not self.move_arc(mid_2_pose, target_pose, steps=50, continuous=True):
                self.get_logger().error("❌ 圆弧移动到终点失败"); return False
            
        # [Step 8] 末端旋转 180度 (倒水)
        self.get_logger().info(f">>> [ID:{cup_id}] 旋转倒水...")
        self.rotate_end_effector()
        time.sleep(1.0)

        # [Step 10] 原路返回 (同时恢复姿态)
        self.get_logger().info(f">>> [ID:{cup_id}] 移动返回 (Move & Recover)...")
        
        if cup_id < (CUP_COUNT + 1) // 2:
            if not self.move_with_retry(self.move_to_pose, pose_up_grasp, retries=5, description="回到起点"):
                 self.get_logger().error("❌ 返回 Pose Up 失败"); return False
        else:
            if not self.move_arc(mid_2_pose, pose_up_grasp, steps=50, continuous=False):
                self.get_logger().error("❌ 圆弧移动到终点失败"); return False
        
        time.sleep(0.5)
        
        # [Step 11] 放置
        self.get_logger().info(f">>> [ID:{cup_id}] 下探放置...")
        if not self.move_with_retry(self.move_linear, grasp_pose, retries=5, description="放不下去"):
            return False    
        time.sleep(0.5)

        self.control_gripper("open")
        time.sleep(0.5)

        # [Step 12] 抬起离开
        self.move_linear(pose_pre_grasp)
        
        # ================= [加回碰撞体] =================
        self.get_logger().info(f">>> [ID:{cup_id}] 重新添加场景障碍物...")
        self.add_cup_obstacle(cup_id) 
        # ===============================================

        self.get_logger().info(f">>> [ID:{cup_id}] 完成！")
        return True


class ArmTaskServiceNode(RealManSimLogicClient):
    """启动后等待服务触发，到点后执行完整机械臂流程。"""

    def __init__(self):
        super().__init__()
        self._task_running = False
        self._task_id = 0
        self._next_cup_id = 0
        self._task_lock = threading.Lock()
        self._task_cond = threading.Condition(self._task_lock)
        self._service = self.create_service(
            Empty,
            '/rm_group/arm_task_execute',
            self._handle_arm_task_execute,
            callback_group=self._cb_group,
        )
        self._service_detailed = self.create_service(
            ArmTaskExecute,
            '/rm_group/arm_task_execute_detailed',
            self._handle_arm_task_execute_detailed,
            callback_group=self._cb_group,
        )
        self.get_logger().info('Arm task service is ready: /rm_group/arm_task_execute')
        self.get_logger().info('Detailed service is ready: /rm_group/arm_task_execute_detailed')
        self.get_logger().info('节点已启动，等待导航到点后触发 /rm_group/arm_task_execute ...')

    def _run_single_task(self, cup_id: int) -> bool:
        """每次触发只执行 1 次抓取投喂流程。"""
        self.get_logger().info(f"\n>>>>>>> [单次] 开始执行抓取投喂流程 (Cup ID: {cup_id}) <<<<<<<")
        ok = self.process_cup(cup_id, 0, 1)
        if ok:
            self.get_logger().info(f"✅ [单次] 抓取投喂流程完成 (Cup ID: {cup_id})")
        else:
            self.get_logger().error(f"❌ [单次] 抓取投喂流程失败 (Cup ID: {cup_id})")
        return ok

    def _request_cancel_current_task(self):
        """请求取消当前正在执行的任务"""
        if not self._task_running:
            return False
        
        self.get_logger().warn("⚠️ 正在请求取消当前任务...")
        self._cancel_requested = True
        
        # 如果正在执行MoveIt action，尝试取消
        if self._current_goal_handle is not None:
            try:
                cancel_future = self._current_goal_handle.cancel_goal_async()
                self.get_logger().info("📤 已发送MoveIt取消请求")
            except Exception as e:
                self.get_logger().error(f"取消MoveIt目标失败: {e}")
        
        return True

    def _reset_cancel_flag(self):
        """重置取消标志"""
        self._cancel_requested = False
        self._current_goal_handle = None
        self._current_cup_id = -1

    def _run_full_task(self) -> bool:
        self.get_logger().info(f"\n>>>>>>> [正向] 开始抓取序列 (共 {CUP_COUNT} 个杯子) <<<<<<<")
        for i in range(CUP_COUNT):
            if not self.process_cup(i, i, CUP_COUNT):
                self.get_logger().error(f"❌ 第 {i+1} 个杯子任务失败，停止正向序列")
                return False

        self.get_logger().info("\n>>>>>>> [正向] 所有杯子任务完成！ <<<<<<<")
        time.sleep(2.0)

        self.get_logger().info(f"\n>>>>>>> [逆向] 开始抓取序列 (共 {CUP_COUNT} 个杯子) <<<<<<<")
        for i in range(CUP_COUNT - 1, -1, -1):
            if not self.process_cup(i, CUP_COUNT - 1 - i, CUP_COUNT):
                self.get_logger().error(f"❌ 逆向第 {CUP_COUNT - i} 个杯子 (ID:{i}) 任务失败，停止逆向序列")
                return False

        self.get_logger().info("\n✅ [全部完成] 正向 & 逆向 任务全部结束！")
        return True

    def _handle_arm_task_execute(self, request, response):
        del request

        with self._task_cond:
            while self._task_running:
                self.get_logger().warn('收到重复任务请求，当前任务正在执行，进入排队等待...')
                self._task_cond.wait()

            self._task_running = True
            self._task_id += 1
            task_id = self._task_id

        self.get_logger().info(f'接收到机械臂任务请求，task_id={task_id}')
        try:
            cup_id = self._next_cup_id
            self._next_cup_id = (self._next_cup_id + 1) % CUP_COUNT
            ok = self._run_single_task(cup_id)
            if ok:
                self.get_logger().info(f'机械臂任务完成，task_id={task_id}')
            else:
                self.get_logger().error(f'机械臂任务失败，task_id={task_id}')
        except Exception as exc:
            self.get_logger().error(f'机械臂任务异常，task_id={task_id}: {exc}')
        finally:
            with self._task_cond:
                self._task_running = False
                self._task_cond.notify_all()

        return response

    def _handle_arm_task_execute_detailed(self, request, response):
        req_waypoint_index = request.waypoint_index
        req_run_full_task = request.run_full_task

        with self._task_cond:
            # [方案3] 如果有任务在运行，取消它
            if self._task_running:
                self.get_logger().warn('🔴 收到新任务，正在打断当前任务...')
                self._request_cancel_current_task()
                
                # 等待当前任务真正结束（带超时，避免死等）
                wait_count = 0
                max_wait = 50  # 最多等5秒 (50 * 0.1)
                while self._task_running and wait_count < max_wait:
                    self._task_cond.wait(timeout=0.1)
                    wait_count += 1
                
                if self._task_running:
                    self.get_logger().error('❌ 无法取消旧任务，强制重置状态')
                    # 强制重置，避免死锁
                    self._task_running = False
                    self._reset_cancel_flag()
                else:
                    self.get_logger().info('✅ 旧任务已取消，开始新任务')
            
            # 开始新任务
            self._task_running = True
            self._reset_cancel_flag()  # 确保新任务从干净状态开始
            self._task_id += 1
            task_id = self._task_id

        response.task_id = task_id
        response.arm_task_start = True
        response.arm_task_done = False
        response.success = False
        response.failed_stage = 0
        response.message = 'running'

        self.get_logger().info(f'🚀 启动新机械臂任务，task_id={task_id}')

        try:
            if req_run_full_task:
                ok = self._run_full_task()
            else:
                if req_waypoint_index > 0:
                    cup_id = int((req_waypoint_index - 1) % CUP_COUNT)
                else:
                    cup_id = self._next_cup_id
                    self._next_cup_id = (self._next_cup_id + 1) % CUP_COUNT
                
                self._current_cup_id = cup_id  # 记录当前cup
                ok = self._run_single_task(cup_id)

            response.arm_task_done = True
            response.success = bool(ok)
            response.failed_stage = 0 if ok else -1
            response.message = 'completed' if ok else 'failed'
            
        except Exception as exc:
            response.arm_task_done = True
            response.success = False
            response.failed_stage = -2
            response.message = str(exc)
            self.get_logger().error(f'任务异常: {exc}')
        
        finally:
            with self._task_cond:
                self._task_running = False
                self._reset_cancel_flag()
                self._task_cond.notify_all()  # 通知等待的线程

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmTaskServiceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node.gripper_sock:
            node.gripper_sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

