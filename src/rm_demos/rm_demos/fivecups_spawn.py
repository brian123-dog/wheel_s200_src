#!/usr/bin/env python3
'''
ros2 topic pub --rate 20 /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['Left_1_Joint'], position: [0.60], velocity: [], effort: []}"
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from std_msgs.msg import Int32 

class RealScenePublisher(Node):
    def __init__(self):
        super().__init__('real_scene_publisher')
        
        # 1. 发布场景
        self.publisher_ = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.timer = self.create_timer(1.0, self.publish_scene)
        
        # 2. [原有] 订阅删除指令
        # 话题名: /rm_group/cup_delete (建议统一加上前缀，或者保持你原来的 /delete_cup_id)
        self.sub_delete = self.create_subscription(
            Int32,
            '/rm_group/cup_delete', # 请确保 Client 端也是发到这个话题
            self.delete_callback,
            10)
        
        # ================= [新增] 3. 订阅添加指令 =================
        # 话题名: /rm_group/cup_add
        self.sub_add = self.create_subscription(
            Int32,
            '/rm_group/cup_add',    # 对应 Client 端 add_cup_obstacle 发的话题
            self.add_callback,      # 新的回调函数
            10)
        # ========================================================
        
        # 使用一个集合记录哪些杯子被“移除”了
        self.removed_cup_indices = set()

        self.get_logger().info(">>> 场景节点启动。等待 Add/Delete 指令...")

    # --- [原有] 删除回调 ---
    def delete_callback(self, msg):
        cup_id = msg.data
        if cup_id == -1:
            self.removed_cup_indices.clear()
            self.get_logger().info("↺ 重置所有杯子")
        else:
            self.removed_cup_indices.add(cup_id) # 加入黑名单 -> 消失
            self.get_logger().info(f"❌ 收到移除指令: cup_{cup_id}")
        self.publish_scene()

    # ================= [新增] 添加回调 =================
    def add_callback(self, msg):
        """
        这里处理加回杯子的逻辑
        其实就是把 ID 从 'removed_cup_indices' 集合里删掉
        """
        cup_id = msg.data
        
        # discard 的好处是：如果集合里本来就没有这个ID，它不会报错，remove会报错
        if cup_id in self.removed_cup_indices:
            self.removed_cup_indices.discard(cup_id) 
            self.get_logger().info(f"✅ 收到添加指令: cup_{cup_id} (已恢复显示)")
        else:
            self.get_logger().info(f"⚠️ 指令 cup_{cup_id} 无效 (该杯子本就在场景中)")
            
        self.publish_scene()
    # ==================================================

    def publish_scene(self):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True 

        # ... (这里保留你原本的 back_wall 代码) ...
        # 简易演示用的平台
        back_wall = CollisionObject()
        back_wall.header.frame_id = "base_link"
        back_wall.id = "back_wall_neg_x"
        back_wall.operation = CollisionObject.ADD
        wall_prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.2, 0.9, 0.13])
        wall_pose = Pose(); wall_pose.position.x = -0.59; wall_pose.position.z = 0.13/2.0; wall_pose.orientation.w = 1.0
        back_wall.primitives.append(wall_prim); back_wall.primitive_poses.append(wall_pose)
        scene_msg.world.collision_objects.append(back_wall)

        # --- 2. 动态发布的杯子 ---
        cup_count = 5
        cup_dia = 0.06
        cup_height = 0.07
        step_y = 0.18 
        start_y = -((cup_count - 1) * step_y) / 2.0
        platform_x = -0.59
        platform_z_top = 0.13

        for i in range(cup_count):
            single_cup = CollisionObject()
            single_cup.header.frame_id = "base_link"
            single_cup.id = f"cup_{i}"

            # === [核心逻辑] ===
            # 如果 i 在“移除列表”里，就是 REMOVE
            # 如果 i 不在列表里，就是 ADD (这就实现了加回效果)
            if i in self.removed_cup_indices:
                single_cup.operation = CollisionObject.REMOVE
            else:
                single_cup.operation = CollisionObject.ADD
                
                prim = SolidPrimitive()
                prim.type = SolidPrimitive.CYLINDER
                prim.dimensions = [cup_height, cup_dia / 2.0]

                pose = Pose()
                pose.position.x = platform_x
                pose.position.y = start_y + (i * step_y)
                pose.position.z = platform_z_top + (cup_height / 2.0)
                pose.orientation.w = 1.0

                single_cup.primitives.append(prim)
                single_cup.primitive_poses.append(pose)

            scene_msg.world.collision_objects.append(single_cup)

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


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped, Pose
# from shape_msgs.msg import SolidPrimitive
# from moveit_msgs.msg import CollisionObject, PlanningScene
# from std_msgs.msg import Int32  # 引入整数消息类型

# class RealScenePublisher(Node):
#     def __init__(self):
#         super().__init__('real_scene_publisher')
        
#         # 1. 发布场景
#         self.publisher_ = self.create_publisher(PlanningScene, '/planning_scene', 10)
#         self.timer = self.create_timer(1.0, self.publish_scene)
        
#         # 2. [新增开关] 订阅一个话题，用于接收指令
#         # 话题名: /delete_cup_id
#         # 类型: Int32
#         self.subscription = self.create_subscription(
#             Int32,
#             '/delete_cup_id',
#             self.listener_callback,
#             10)
        
#         # 3. 使用一个集合(Set)来记录哪些杯子已经被“移除”了
#         # 开始时是空的，表示所有杯子都在
#         self.removed_cup_indices = set()

#         self.get_logger().info(">>> 节点已启动！所有杯子默认显示。")
#         self.get_logger().info(">>> 请在终端输入指令控制移除: ros2 topic pub --once /delete_cup_id std_msgs/msg/Int32 \"{data: 2}\"")

#     def listener_callback(self, msg):
#         """
#         这个回调函数会在你收到指令时触发
#         """
#         cup_id = msg.data
        
#         if cup_id == -1:
#             # 设定：如果输入 -1，则重置所有杯子（全部变回来）
#             self.removed_cup_indices.clear()
#             self.get_logger().info(f"收到指令: {cup_id} -> 重置！所有杯子已恢复。")
#         else:
#             # 记录要删除的 ID
#             self.removed_cup_indices.add(cup_id)
#             self.get_logger().info(f"收到指令: {cup_id} -> 杯子 cup_{cup_id} 将被移除！")
            
#         # 收到指令后立即触发一次发布，不用等定时器
#         self.publish_scene()

#     def publish_scene(self):
#         scene_msg = PlanningScene()
#         scene_msg.is_diff = True 

#         # --- 1. 地面和平台 (这些保持不变) ---
#         # (为了代码简洁，我把之前那堆复杂的静态物体简化为保留平台，
#         #  你在实际用的时候请把之前的货架、柱子代码都留着)
        
#         # 简易演示用的平台
#         back_wall = CollisionObject()
#         back_wall.header.frame_id = "base_link"
#         back_wall.id = "back_wall_neg_x"
#         back_wall.operation = CollisionObject.ADD
#         wall_prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.2, 0.9, 0.13])
#         wall_pose = Pose(); wall_pose.position.x = -0.59; wall_pose.position.z = 0.13/2.0; wall_pose.orientation.w = 1.0
#         back_wall.primitives.append(wall_prim); back_wall.primitive_poses.append(wall_pose)
#         scene_msg.world.collision_objects.append(back_wall)

#         # --- 2. 动态发布的杯子 ---
#         cup_count = 5
#         cup_dia = 0.06
#         cup_height = 0.07
#         gap = 0.02
#         step_y = cup_dia + gap 
#         start_y = -((cup_count - 1) * step_y) / 2.0
#         platform_x = -0.59
#         platform_z_top = 0.13

#         for i in range(cup_count):
#             single_cup = CollisionObject()
#             single_cup.header.frame_id = "base_link"
#             single_cup.id = f"cup_{i}" # ID: cup_0, cup_1 ...

#             # === [关键逻辑] 检查这个ID是否在移除列表中 ===
#             if i in self.removed_cup_indices:
#                 # 如果在“黑名单”里，就执行 REMOVE
#                 single_cup.operation = CollisionObject.REMOVE
#             else:
#                 # 否则执行 ADD
#                 single_cup.operation = CollisionObject.ADD
                
#                 prim = SolidPrimitive()
#                 prim.type = SolidPrimitive.CYLINDER
#                 prim.dimensions = [cup_height, cup_dia / 2.0]

#                 pose = Pose()
#                 pose.position.x = platform_x
#                 pose.position.y = start_y + (i * step_y)
#                 pose.position.z = platform_z_top + (cup_height / 2.0)
#                 pose.orientation.w = 1.0

#                 single_cup.primitives.append(prim)
#                 single_cup.primitive_poses.append(pose)

#             scene_msg.world.collision_objects.append(single_cup)

#         self.publisher_.publish(scene_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = RealScenePublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# ros2 topic pub --once /delete_cup_id std_msgs/msg/Int32 "{data: 2}"


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped, Pose
# from shape_msgs.msg import SolidPrimitive
# from moveit_msgs.msg import CollisionObject, PlanningScene

# class RealScenePublisher(Node):
#     def __init__(self):
#         super().__init__('real_scene_publisher')
        
#         self.publisher_ = self.create_publisher(PlanningScene, '/planning_scene', 10)
#         self.timer = self.create_timer(1.0, self.publish_scene)
        
#         # ==========================================
#         # 模拟：假设我们要移除索引为 3 的杯子 (即第4个)
#         # 在实际业务逻辑中，当你抓起杯子后，就把它的索引改成这个值
#         # ==========================================
#         self.removed_cup_index = 3 
        
#         self.get_logger().info(f">>> 场景发布中... (已模拟移除杯子 ID: cup_{self.removed_cup_index})")

#     def publish_scene(self):
#         scene_msg = PlanningScene()
#         scene_msg.is_diff = True 

#         # ---------------------------------------------------
#         # 1-4. 静态环境 (地面、货架、后方柱子、后方平台) - 保持不变
#         # ---------------------------------------------------
        
#         # 1. 地面
#         floor = CollisionObject()
#         floor.header.frame_id = "base_link"
#         floor.id = "floor"
#         floor.operation = CollisionObject.ADD
#         floor.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[3.0, 3.0, 0.1]))
#         pose_f = Pose(); pose_f.position.z = -0.051; pose_f.orientation.w = 1.0
#         floor.primitive_poses.append(pose_f)
#         scene_msg.world.collision_objects.append(floor)

#         # 2. 货架 (简化代码显示，逻辑同前)
#         shelf = CollisionObject()
#         shelf.header.frame_id = "base_link"
#         shelf.id = "shelf_rack_right"
#         shelf.operation = CollisionObject.ADD
#         # ... 这里省略重复的货架构建代码，与上一次完全一致，为了节省篇幅 ...
#         # (请确保你的实际代码里保留了完整的货架构建部分)
#         # 为了演示杯子，这里我先只发布杯子和平台，如果你需要完整的代码，我可以把货架补全。
#         # 这里我仅用一行示意货架还在：
#         # scene_msg.world.collision_objects.append(shelf) 
#         # *注：为了运行不出错，建议你直接把这部分保留，这里主要关注下面的杯子逻辑*

#         # 4. 后方平台 (杯子放这里)
#         back_wall = CollisionObject()
#         back_wall.header.frame_id = "base_link"
#         back_wall.id = "back_wall_neg_x"
#         back_wall.operation = CollisionObject.ADD
#         wall_prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.2, 0.9, 0.12])
#         wall_pose = Pose()
#         wall_pose.position.x = -0.57
#         wall_pose.position.y = 0.0
#         wall_pose.position.z = 0.12 / 2.0 
#         wall_pose.orientation.w = 1.0
#         back_wall.primitives.append(wall_prim)
#         back_wall.primitive_poses.append(wall_pose)
#         scene_msg.world.collision_objects.append(back_wall)

#         # ==========================================
#         # 5. [重点修改] 独立发布的 10 个杯子
#         # ==========================================
        
#         # 参数
#         cup_count = 5
#         cup_dia = 0.06
#         cup_height = 0.10
#         gap = 0.02
#         platform_z_top = 0.12
#         platform_x = -0.57
        
#         # 计算Y轴步长
#         step_y = cup_dia + gap 
#         start_y = -((cup_count - 1) * step_y) / 2.0

#         for i in range(cup_count):
#             # 1. 为每个杯子创建一个独立的 CollisionObject
#             single_cup = CollisionObject()
#             single_cup.header.frame_id = "base_link"
            
#             # 2. 关键：给每个杯子唯一的 ID
#             cup_id = f"cup_{i}"  # 例如: cup_0, cup_1 ... cup_9
#             single_cup.id = cup_id

#             # 3. 判断状态：如果是被抓走的杯子，设置为 REMOVE，否则 ADD
#             if i == self.removed_cup_index:
#                 # 告诉 MoveIt 把这个 ID 的物体删掉
#                 single_cup.operation = CollisionObject.REMOVE
#             else:
#                 # 其他杯子正常添加
#                 single_cup.operation = CollisionObject.ADD

#                 # 创建几何体
#                 prim = SolidPrimitive()
#                 prim.type = SolidPrimitive.CYLINDER
#                 prim.dimensions = [cup_height, cup_dia / 2.0]

#                 # 创建位姿
#                 pose = Pose()
#                 pose.position.x = platform_x
#                 pose.position.y = start_y + (i * step_y)
#                 pose.position.z = platform_z_top + (cup_height / 2.0)
#                 pose.orientation.w = 1.0

#                 single_cup.primitives.append(prim)
#                 single_cup.primitive_poses.append(pose)

#             # 4. 将这个独立的杯子对象加入列表
#             scene_msg.world.collision_objects.append(single_cup)

#         self.publisher_.publish(scene_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = RealScenePublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()