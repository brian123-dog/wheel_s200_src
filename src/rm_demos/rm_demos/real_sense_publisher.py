#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene

class RealScenePublisher(Node):
    def __init__(self):
        super().__init__('real_scene_publisher')
        
        # 创建发布者，发布到 /planning_scene 话题
        self.publisher_ = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # 定时器：每 1.0 秒发布一次环境信息
        # 持续发布是为了防止 Rviz 重启后障碍物消失
        self.timer = self.create_timer(1.0, self.publish_scene)
        
        self.get_logger().info(">>> 真机环境发布节点已启动 (发布地面和桌子)...")

    def publish_scene(self):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True  # 增量模式，不清除已有物体

        # ==========================================
        # 1. 添加地面 (Floor) - 防止撞地
        # ==========================================
        floor = CollisionObject()
        floor.header.stamp = self.get_clock().now().to_msg()
        floor.header.frame_id = "base_link"  # 参考机械臂底座
        floor.id = "floor"
        floor.operation = CollisionObject.ADD

        # 创建一个巨大的薄板作为地面
        floor_prim = SolidPrimitive()
        floor_prim.type = SolidPrimitive.BOX
        # 长3米，宽3米，厚0.1米
        floor_prim.dimensions = [3.0, 3.0, 0.1]

        floor_pose = Pose()
        # 假设底座固定在桌子上，桌面高度为 0
        # 如果你想定义真正的地面，需要根据桌子高度设置 Z
        # 这里演示：在底座下方 0.05米处加一个保护层
        floor_pose.position.x = 0.0
        floor_pose.position.y = 0.0
        # Z = -0.05 (中心点)，厚度0.1，所以表面刚好在 Z=0 (紧贴底座底部)
        floor_pose.position.z = -0.051 
        floor_pose.orientation.w = 1.0

        floor.primitives.append(floor_prim)
        floor.primitive_poses.append(floor_pose)
        scene_msg.world.collision_objects.append(floor)

        # ==========================================
        # 2. 添加前方障碍 (比如水槽壁或墙)
        # ==========================================
        # 如果你前方有水槽，可以把这个打开
        
        # wall = CollisionObject()
        # wall.header.frame_id = "base_link"
        # wall.id = "front_wall"
        # wall.operation = CollisionObject.ADD
        
        # wall_prim = SolidPrimitive()
        # wall_prim.type = SolidPrimitive.BOX
        # wall_prim.dimensions = [0.1, 1.0, 0.5] # 厚0.1，宽1米，高0.5米
        
        # wall_pose = Pose()
        # wall_pose.position.x = 0.6  # 前方 0.6米
        # wall_pose.position.y = 0.0
        # wall_pose.position.z = 0.25 # 高度中心
        # wall_pose.orientation.w = 1.0
        
        # wall.primitives.append(wall_prim)
        # wall.primitive_poses.append(wall_pose)
        # scene_msg.world.collision_objects.append(wall)
        

        # ==========================================
        # 2. 添加右侧开口货架 (Right Side Shelf)
        # ==========================================
        shelf = CollisionObject()
        shelf.header.frame_id = "base_link"
        shelf.id = "shelf_rack_right"  # 改个ID，避免和旧的冲突
        shelf.operation = CollisionObject.ADD

        # --- 货架尺寸参数 (单位: 米) ---
        # 注意：因为旋转了90度，现在的 Length 是沿 X轴 的，Depth 是沿 Y轴 的
        shelf_len_x   = 0.625 + 0.625 # 货架左右宽度 (沿 X轴)
        shelf_depth_y = 0.500  # 货架进深 (沿 Y轴)
        shelf_h       = 0.350  # 高度
        thick         = 0.005  # 板厚

        # --- 货架中心位置 ---
        center_x = -0.3125 -0.15    # X=0 (在机械臂正侧面)
        center_y = -0.70     # Y=-0.6 (机械臂右侧 0.6米处)
        base_z   = -0.05 + 0.13 + 0.09 #+ 0.350 / 2    # 放在桌面上
        
        # --- 拼装货架的5个面 (旋转后) ---
        
        # 1. 底板 (Bottom)
        prim_bottom = SolidPrimitive()
        prim_bottom.type = SolidPrimitive.BOX
        prim_bottom.dimensions = [shelf_len_x, shelf_depth_y, thick]
        pose_bottom = Pose()
        pose_bottom.position.x = center_x
        pose_bottom.position.y = center_y
        pose_bottom.position.z = base_z + (thick / 2.0)
        pose_bottom.orientation.w = 1.0
        shelf.primitives.append(prim_bottom)
        shelf.primitive_poses.append(pose_bottom)

        # 2. 顶板 (Top)
        prim_top = SolidPrimitive()
        prim_top.type = SolidPrimitive.BOX
        prim_top.dimensions = [shelf_len_x, shelf_depth_y, thick]
        pose_top = Pose()
        pose_top.position.x = center_x
        pose_top.position.y = center_y
        pose_top.position.z = base_z + shelf_h - (thick / 2.0)
        pose_top.orientation.w = 1.0
        shelf.primitives.append(prim_top)
        shelf.primitive_poses.append(pose_top)

        # # 3. 背板 (Back) 
        # # 开口朝左(Y+)，所以背板在最右侧 (Y-)
        # prim_back = SolidPrimitive()
        # prim_back.type = SolidPrimitive.BOX
        # prim_back.dimensions = [shelf_len_x, thick, shelf_h]
        # pose_back = Pose()
        # pose_back.position.x = center_x
        # pose_back.position.y = center_y - (shelf_depth_y / 2.0) + (thick / 2.0)
        # pose_back.position.z = base_z + (shelf_h / 2.0)
        # pose_back.orientation.w = 1.0
        # shelf.primitives.append(prim_back)
        # shelf.primitive_poses.append(pose_back)

        # 4. 前侧板 (Front Side - X+)
        # 也就是坐在机械臂“前方”的那块侧板
        prim_side_front = SolidPrimitive()
        prim_side_front.type = SolidPrimitive.BOX
        prim_side_front.dimensions = [thick, shelf_depth_y, shelf_h]
        pose_side_front = Pose()
        pose_side_front.position.x = center_x + (shelf_len_x / 2.0) - (thick / 2.0)
        pose_side_front.position.y = center_y
        pose_side_front.position.z = base_z + (shelf_h / 2.0)
        pose_side_front.orientation.w = 1.0
        shelf.primitives.append(prim_side_front)
        shelf.primitive_poses.append(pose_side_front)

        # 5. 后侧板 (Rear Side - X-)
        # 也就是坐在机械臂“身后”的那块侧板
        prim_side_rear = SolidPrimitive()
        prim_side_rear.type = SolidPrimitive.BOX
        prim_side_rear.dimensions = [thick, shelf_depth_y, shelf_h]
        pose_side_rear = Pose()
        pose_side_rear.position.x = center_x - (shelf_len_x / 2.0) + (thick / 2.0)
        pose_side_rear.position.y = center_y
        pose_side_rear.position.z = base_z + (shelf_h / 2.0)
        pose_side_rear.orientation.w = 1.0
        shelf.primitives.append(prim_side_rear)
        shelf.primitive_poses.append(pose_side_rear)

        scene_msg.world.collision_objects.append(shelf)

        # ==========================================
        # 3. 添加后方双柱障碍物 (Rear Obstacles) - 修正版
        #    位置：机械臂正后方 (X轴负方向)
        #    排列：左右分布 (Y轴方向)
        # ==========================================
        rear_obs = CollisionObject()
        rear_obs.header.frame_id = "base_link"
        rear_obs.id = "rear_double_pillars"
        rear_obs.operation = CollisionObject.ADD

        # --- 参数定义 ---
        obs_height  = 1.530
        obs_thick   = 0.132  # 柱子厚度
        obs_dist    = 0.1115 # 距离底座中心的距离
        block_len   = 0.097  # 柱子宽度
        block_x_off = 0.1515 # 左右间距偏移量
        
        # --- 1. 计算中心位置 ---
        # 放在 X轴 正方向 (前方)
        # center_x = (距离 + 厚度的一半)
        center_x = obs_dist + (obs_thick / 2.0)
        
        # Z轴高度不变
        center_z = (obs_height / 2.0) + 0.01 

        # --- 2. 修正尺寸 (旋转90度) ---
        # 原来是 [block_len, obs_thick, height]
        # 现在旋转90度：X变成厚度，Y变成宽度
        # 这样柱子的扁平面是朝向机械臂的
        new_dims = [obs_thick, block_len, obs_height]

        # --- 3. 创建左柱 (Left Pillar, Y+) ---
        prim_L = SolidPrimitive()
        prim_L.type = SolidPrimitive.BOX
        prim_L.dimensions = new_dims
        
        pose_L = Pose()
        pose_L.position.x = center_x      # 在身后
        pose_L.position.y = block_x_off   # Y+ (左侧)
        pose_L.position.z = center_z
        pose_L.orientation.w = 1.0
        
        rear_obs.primitives.append(prim_L)
        rear_obs.primitive_poses.append(pose_L)

        # --- 4. 创建右柱 (Right Pillar, Y-) ---
        prim_R = SolidPrimitive()
        prim_R.type = SolidPrimitive.BOX
        prim_R.dimensions = new_dims
        
        pose_R = Pose()
        pose_R.position.x = center_x      # 在身后
        pose_R.position.y = -block_x_off  # Y- (右侧)
        pose_R.position.z = center_z
        pose_R.orientation.w = 1.0

        rear_obs.primitives.append(prim_R)
        rear_obs.primitive_poses.append(pose_R)

        scene_msg.world.collision_objects.append(rear_obs)


        # ==========================================
        # 3. 关于杯子 (Cups)
        # ==========================================
        # 在真机上，通常不建议把还没抓到的杯子添加为 CollisionObject，
        # 除非你的视觉非常精准，且杯子不动。
        # 否则，如果实物和模型有 1cm 的偏差，MoveIt 会认为“手还没碰到杯子就撞墙了”，
        # 导致无法规划抓取路径。
        # 
        # 最佳实践：
        # 1. 抓取前：只把桌子当作障碍物。
        # 2. 抓取后：使用 attach_object (我们主程序里写的) 把杯子连在手上。

        # 发布

        # ==========================================
        # 4. [NEW] 添加后方墙体 (X轴负方向)
        #    中心: -57cm, 0, 0
        #    尺寸: 长(Y)90cm, 宽(X)20cm, 高119cm
        # ==========================================
        back_wall = CollisionObject()
        back_wall.header.frame_id = "base_link"
        back_wall.id = "back_wall_neg_x"
        back_wall.operation = CollisionObject.ADD

        wall_prim = SolidPrimitive()
        wall_prim.type = SolidPrimitive.BOX
        # dimensions: [x, y, z]
        # 题目要求：长(Y轴)=0.9，宽(X轴)=0.2，高(Z)=1.19
        wall_prim.dimensions = [0.2, 0.9, 0.13]

        wall_pose = Pose()
        wall_pose.position.x = -0.59  # X轴负方向 57cm
        wall_pose.position.y = 0.0
        
        # 这里的Z是指几何中心的高度。
        # 如果你想让物体"站在" Z=0 的平面上，Z中心应该设为 高度/2
        # 即 1.19 / 2 = 0.595
        wall_pose.position.z = 0.13 / 2.0 
        wall_pose.orientation.w = 1.0

        back_wall.primitives.append(wall_prim)
        back_wall.primitive_poses.append(wall_pose)
        
        scene_msg.world.collision_objects.append(back_wall)

        # # ==========================================
        # # 4.1 [NEW] 添加后饵料架 (X轴负方向)
        # #    中心: -57cm, 0, 0
        # #    尺寸: 长(Y)90cm, 宽(X)20cm, 高119cm
        # # ==========================================
        # back_wall = CollisionObject()
        # back_wall.header.frame_id = "base_link"
        # back_wall.id = "back_shelf_neg_x"
        # back_wall.operation = CollisionObject.ADD

        # wall_prim = SolidPrimitive()
        # wall_prim.type = SolidPrimitive.BOX
        # # dimensions: [x, y, z]
        # # 题目要求：长(Y轴)=0.9，宽(X轴)=0.2，高(Z)=1.19
        # wall_prim.dimensions = [0.2, 0.9, 0.5]

        # wall_pose = Pose()
        # wall_pose.position.x = -0.87  # X轴负方向 57cm
        # wall_pose.position.y = 0.0
        
        # # 这里的Z是指几何中心的高度。
        # # 如果你想让物体"站在" Z=0 的平面上，Z中心应该设为 高度/2
        # # 即 1.19 / 2 = 0.595
        # wall_pose.position.z = 0.5 / 2.0 
        # wall_pose.orientation.w = 1.0

        # back_wall.primitives.append(wall_prim)
        # back_wall.primitive_poses.append(wall_pose)
        
        # scene_msg.world.collision_objects.append(back_wall)

        # ==========================================
        
        # ==========================================
        # # 5. [NEW] 添加 10 个杯子
        # #    直径5cm, 间隔2cm, 依次排开在平台(高13.5cm)上
        # # ==========================================
        # cups = CollisionObject()
        # cups.header.frame_id = "base_link"
        # cups.id = "row_of_cups"
        # cups.operation = CollisionObject.ADD

        # # 杯子参数
        # cup_count = 10
        # cup_dia = 0.05      # 直径 5cm
        # cup_radius = cup_dia / 2.0
        # cup_height = 0.075   # 假设杯子高 10cm
        # gap = 0.03          # 间隔 2cm
        
        # # 平台参数
        # platform_z_top = 0.12
        # platform_x = -0.57

        # # 计算Y轴起始位置 (居中排列)
        # # 两个杯子中心的距离 = 直径 + 间隔
        # step_y = cup_dia + gap 
        # # 总跨度 (圆心到圆心) = (N-1) * step
        # total_span = (cup_count - 1) * step_y
        # start_y = -total_span / 2.0

        # for i in range(cup_count):
        #     # 创建杯子几何体
        #     cup_prim = SolidPrimitive()
        #     cup_prim.type = SolidPrimitive.CYLINDER
        #     # Cylinder dimensions: [height, radius]
        #     cup_prim.dimensions = [cup_height, cup_radius]

        #     # 创建杯子位姿
        #     cup_pose = Pose()
        #     cup_pose.position.x = platform_x
            
        #     # Y轴位置：起始点 + i * 步长
        #     cup_pose.position.y = start_y + (i * step_y)
            
        #     # Z轴位置：平台表面 + 杯子高度的一半 (几何中心)
        #     cup_pose.position.z = platform_z_top + (cup_height / 2.0)
            
        #     cup_pose.orientation.w = 1.0

        #     # 添加到集合中
        #     cups.primitives.append(cup_prim)
        #     cups.primitive_poses.append(cup_pose)

        # scene_msg.world.collision_objects.append(cups)
        # self.publisher_.publish(scene_msg)
        self.publisher_.publish(scene_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()