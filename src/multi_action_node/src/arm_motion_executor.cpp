#include "multi_action_node/arm_motion_executor.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <moveit_msgs/msg/bounding_volume.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{
constexpr const char * kPlanningGroup = "rm_group";
constexpr const char * kBaseLink = "base_link";
constexpr const char * kEndEffector = "link6";

constexpr const char * kGripperIp = "192.168.1.18";
constexpr int kGripperPort = 8080;
constexpr int kDeviceId = 1;
constexpr int kPosOpen = 3300;
constexpr int kPosClose = 4700;

constexpr double kChassisHeight = 0.0;
constexpr double kJointGap = 0.0;
constexpr double kTubeHeight = 0.0;
constexpr double kStepHeight = 0.130;
constexpr double kCupHeight = 0.07;
constexpr double kCupRadius = 0.03;
constexpr double kCupGap = 0.12;
constexpr int kCupCount = 5;
constexpr double kBaseLen = 0.4843 - 0.013;
constexpr double kTopLen = 0.20;
constexpr double kThickness = 0.005;
constexpr double kGripperLength = 0.186;
constexpr double kPlatformOffset = 0.0;

std::vector<geometry_msgs::msg::Pose> interpolate_segment(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & end,
  int count)
{
  std::vector<geometry_msgs::msg::Pose> points;
  points.reserve(static_cast<size_t>(count));
  for (int i = 1; i <= count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(count);
    geometry_msgs::msg::Pose p;
    p.position.x = start.position.x + (end.position.x - start.position.x) * t;
    p.position.y = start.position.y + (end.position.y - start.position.y) * t;
    p.position.z = start.position.z + (end.position.z - start.position.z) * t;
    p.orientation.x = start.orientation.x + (end.orientation.x - start.orientation.x) * t;
    p.orientation.y = start.orientation.y + (end.orientation.y - start.orientation.y) * t;
    p.orientation.z = start.orientation.z + (end.orientation.z - start.orientation.z) * t;
    p.orientation.w = start.orientation.w + (end.orientation.w - start.orientation.w) * t;
    points.push_back(p);
  }
  return points;
}

struct Vec3
{
  double x;
  double y;
  double z;
};

Vec3 operator+(const Vec3 & a, const Vec3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3 & a, const Vec3 & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3 & a, double s)
{
  return {a.x * s, a.y * s, a.z * s};
}

double dot(const Vec3 & a, const Vec3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x};
}

double norm(const Vec3 & v)
{
  return std::sqrt(dot(v, v));
}

Vec3 normalized(const Vec3 & v)
{
  const double n = norm(v);
  if (n < 1e-9) {
    return {0.0, 0.0, 0.0};
  }
  return v * (1.0 / n);
}

Vec3 pose_to_vec(const geometry_msgs::msg::Pose & p)
{
  return {p.position.x, p.position.y, p.position.z};
}

geometry_msgs::msg::Quaternion slerp(
  const geometry_msgs::msg::Quaternion & q1,
  const geometry_msgs::msg::Quaternion & q2,
  double t)
{
  double x1 = q1.x;
  double y1 = q1.y;
  double z1 = q1.z;
  double w1 = q1.w;
  double x2 = q2.x;
  double y2 = q2.y;
  double z2 = q2.z;
  double w2 = q2.w;

  double cos_theta = x1 * x2 + y1 * y2 + z1 * z2 + w1 * w2;
  if (cos_theta < 0.0) {
    x2 = -x2;
    y2 = -y2;
    z2 = -z2;
    w2 = -w2;
    cos_theta = -cos_theta;
  }

  geometry_msgs::msg::Quaternion out;
  if (cos_theta > 0.9995) {
    out.x = (1.0 - t) * x1 + t * x2;
    out.y = (1.0 - t) * y1 + t * y2;
    out.z = (1.0 - t) * z1 + t * z2;
    out.w = (1.0 - t) * w1 + t * w2;
    const double n = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z + out.w * out.w);
    if (n > 1e-9) {
      out.x /= n;
      out.y /= n;
      out.z /= n;
      out.w /= n;
    }
    return out;
  }

  const double theta = std::acos(cos_theta);
  const double sin_theta = std::sin(theta);
  const double a = std::sin((1.0 - t) * theta) / sin_theta;
  const double b = std::sin(t * theta) / sin_theta;

  out.x = a * x1 + b * x2;
  out.y = a * y1 + b * y2;
  out.z = a * z1 + b * z2;
  out.w = a * w1 + b * w2;
  return out;
}

void scale_trajectory(
  moveit_msgs::msg::RobotTrajectory & traj,
  double velocity_scaling,
  double acceleration_scaling)
{
  if (std::abs(velocity_scaling - 1.0) < 1e-4 && std::abs(acceleration_scaling - 1.0) < 1e-4) {
    return;
  }

  const double scale_factor = 1.0 / std::max(velocity_scaling, 1e-3);
  for (auto & point : traj.joint_trajectory.points) {
    const double total_nanos =
      (static_cast<double>(point.time_from_start.sec) * 1e9 +
      static_cast<double>(point.time_from_start.nanosec)) * scale_factor;
    point.time_from_start.sec = static_cast<int32_t>(total_nanos / 1e9);
    point.time_from_start.nanosec = static_cast<uint32_t>(std::fmod(total_nanos, 1e9));

    for (auto & v : point.velocities) {
      v *= velocity_scaling;
    }
    for (auto & a : point.accelerations) {
      a *= acceleration_scaling;
    }
  }
}

std::vector<geometry_msgs::msg::Pose> get_arc_waypoints_continuous(
  const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::Pose & mid_pose,
  const geometry_msgs::msg::Pose & end_pose,
  int num_points)
{
  const Vec3 p1 = pose_to_vec(start_pose);
  const Vec3 p2 = pose_to_vec(mid_pose);
  const Vec3 p3 = pose_to_vec(end_pose);

  const Vec3 v1 = p2 - p1;
  const Vec3 v2 = p3 - p2;
  Vec3 n = cross(v1, v2);
  if (norm(n) < 1e-6) {
    return {};
  }
  n = normalized(n);

  const Vec3 m1 = (p1 + p2) * 0.5;
  const Vec3 m2 = (p2 + p3) * 0.5;
  const Vec3 dir1 = cross(v1, n);
  const Vec3 dir2 = cross(v2, n);

  // Solve m1 + t1*dir1 = m2 + t2*dir2 via least squares on 3x2 system.
  const Vec3 b = m2 - m1;
  const double a11 = dot(dir1, dir1);
  const double a12 = -dot(dir1, dir2);
  const double a22 = dot(dir2, dir2);
  const double det = a11 * a22 - a12 * a12;
  if (std::abs(det) < 1e-9) {
    return {};
  }
  const double rhs1 = dot(dir1, b);
  const double rhs2 = -dot(dir2, b);
  const double t1 = (rhs1 * a22 - rhs2 * a12) / det;
  const Vec3 center = m1 + dir1 * t1;

  const Vec3 vec_s = p1 - center;
  const Vec3 vec_m = p2 - center;
  const Vec3 vec_e = p3 - center;

  auto raw_angle = [&n](const Vec3 & a, const Vec3 & bvec) {
    const Vec3 c = cross(a, bvec);
    double ang = std::atan2(norm(c), dot(a, bvec));
    if (dot(c, n) < 0.0) {
      ang = 2.0 * M_PI - ang;
    }
    return ang;
  };

  const double total_angle = raw_angle(vec_s, vec_m) + raw_angle(vec_m, vec_e);
  std::vector<geometry_msgs::msg::Pose> out;
  out.reserve(static_cast<size_t>(num_points));

  for (int i = 0; i < num_points; ++i) {
    const double fraction = static_cast<double>(i + 1) / static_cast<double>(num_points);
    const double current_angle = fraction * total_angle;
    const Vec3 v_curr = vec_s * std::cos(current_angle) + cross(n, vec_s) * std::sin(current_angle);
    const Vec3 pt = center + v_curr;

    geometry_msgs::msg::Pose p;
    p.position.x = pt.x;
    p.position.y = pt.y;
    p.position.z = pt.z;
    p.orientation = slerp(start_pose.orientation, end_pose.orientation, fraction);
    out.push_back(p);
  }

  return out;
}

}  // namespace

ArmMotionExecutor::ArmMotionExecutor(rclcpp::Node * node)
: node_(node)
{
}

bool ArmMotionExecutor::initialize()
{
  if (node_ == nullptr) {
    return false;
  }

  move_group_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
    node_, "move_action");
  execute_client_ = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
    node_, "execute_trajectory");
  cartesian_client_ = node_->create_client<moveit_msgs::srv::GetCartesianPath>(
    "compute_cartesian_path");
  planning_scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
    "get_planning_scene");

  scene_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
  cup_delete_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/rm_group/cup_delete", 10);
  cup_add_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/rm_group/cup_add", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  move_group_client_->wait_for_action_server(std::chrono::seconds(3));
  execute_client_->wait_for_action_server(std::chrono::seconds(3));
  cartesian_client_->wait_for_service(std::chrono::seconds(3));
  planning_scene_client_->wait_for_service(std::chrono::seconds(3));

  add_scene_objects();
  init_gripper();

  return true;
}

void ArmMotionExecutor::add_scene_objects()
{
  if (!scene_pub_) {
    return;
  }

  moveit_msgs::msg::PlanningScene scene_msg;
  scene_msg.is_diff = true;
  scene_pub_->publish(scene_msg);
}

void ArmMotionExecutor::delete_cup_obstacle(int cup_id)
{
  if (!cup_delete_pub_) {
    return;
  }
  std_msgs::msg::Int32 msg;
  msg.data = cup_id;
  cup_delete_pub_->publish(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void ArmMotionExecutor::add_cup_obstacle(int cup_id)
{
  if (!cup_add_pub_) {
    return;
  }
  std_msgs::msg::Int32 msg;
  msg.data = cup_id;
  cup_add_pub_->publish(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

bool ArmMotionExecutor::control_gripper_open()
{
  const int target = kPosOpen;
  const int low = target & 0xFF;
  const int high = (target >> 8) & 0xFF;
  std::ostringstream ss;
  ss << "{\"command\":\"write_registers\",\"port\":1,\"address\":258,\"num\":2,\"data\":[0,0,"
     << high << "," << low << "],\"device\":" << kDeviceId << "}\r\n";
  if (!send_gripper_json(ss.str())) {
    return false;
  }
  return send_gripper_json("{\"command\":\"write_single_register\",\"port\":1,\"address\":264,\"data\":1,\"device\":1}\r\n");
}

bool ArmMotionExecutor::control_gripper_close()
{
  const int target = kPosClose;
  const int low = target & 0xFF;
  const int high = (target >> 8) & 0xFF;
  std::ostringstream ss;
  ss << "{\"command\":\"write_registers\",\"port\":1,\"address\":258,\"num\":2,\"data\":[0,0,"
     << high << "," << low << "],\"device\":" << kDeviceId << "}\r\n";
  if (!send_gripper_json(ss.str())) {
    return false;
  }
  return send_gripper_json("{\"command\":\"write_single_register\",\"port\":1,\"address\":264,\"data\":1,\"device\":1}\r\n");
}

bool ArmMotionExecutor::move_to_pose(const geometry_msgs::msg::Pose & pose)
{
  moveit_msgs::action::MoveGroup::Goal goal_msg;
  goal_msg.request.group_name = kPlanningGroup;
  goal_msg.request.num_planning_attempts = 20;
  goal_msg.request.allowed_planning_time = 10.0;
  goal_msg.request.max_velocity_scaling_factor = 0.9;
  goal_msg.request.max_acceleration_scaling_factor = 0.9;

  moveit_msgs::msg::PositionConstraint pos_con;
  pos_con.header.frame_id = kBaseLink;
  pos_con.link_name = kEndEffector;
  moveit_msgs::msg::BoundingVolume region;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.001, 0.001, 0.001};
  region.primitives.push_back(box);
  region.primitive_poses.push_back(pose);
  pos_con.constraint_region = region;
  pos_con.weight = 1.0;

  moveit_msgs::msg::OrientationConstraint ori_con;
  ori_con.header.frame_id = kBaseLink;
  ori_con.link_name = kEndEffector;
  ori_con.orientation = pose.orientation;
  ori_con.absolute_x_axis_tolerance = 0.001;
  ori_con.absolute_y_axis_tolerance = 0.001;
  ori_con.absolute_z_axis_tolerance = 0.001;
  ori_con.weight = 1.0;

  moveit_msgs::msg::Constraints goal_constraints;
  goal_constraints.position_constraints.push_back(pos_con);
  goal_constraints.orientation_constraints.push_back(ori_con);
  goal_msg.request.goal_constraints.push_back(goal_constraints);

  moveit_msgs::msg::Constraints path_constraints;
  moveit_msgs::msg::JointConstraint joint_con;
  joint_con.joint_name = "joint1";
  joint_con.position = 0.0;
  joint_con.tolerance_above = 1.57;
  joint_con.tolerance_below = 1.57;
  joint_con.weight = 1.0;
  path_constraints.joint_constraints.push_back(joint_con);
  goal_msg.request.path_constraints = path_constraints;

  return send_goal_and_wait(goal_msg);
}

bool ArmMotionExecutor::move_linear(
  const geometry_msgs::msg::Pose & target_pose,
  double vel_scale,
  double acc_scale)
{
  (void)vel_scale;
  (void)acc_scale;

  if (!cartesian_client_ || !execute_client_ || node_ == nullptr) {
    return false;
  }

  auto req = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>();
  req->header.frame_id = kBaseLink;
  req->header.stamp = node_->get_clock()->now();
  req->group_name = kPlanningGroup;
  req->waypoints = {target_pose};
  req->max_step = 0.02;
  req->jump_threshold = 0.0;
  req->avoid_collisions = true;

  auto future = cartesian_client_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto response = future.get();
  if (!response) {
    return false;
  }
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    return false;
  }
  if (response->fraction < 0.90) {
    return false;
  }

  auto scaled = response->solution;
  scale_trajectory(scaled, vel_scale, acc_scale);
  return execute_trajectory(scaled);
}

bool ArmMotionExecutor::move_arc(
  const geometry_msgs::msg::Pose & mid_pose,
  const geometry_msgs::msg::Pose & end_pose,
  int steps,
  bool continuous,
  double vel_scale,
  double acc_scale)
{
  (void)vel_scale;
  (void)acc_scale;

  geometry_msgs::msg::Pose start_pose;
  if (!get_current_pose(start_pose)) {
    return false;
  }

  auto req = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>();
  req->header.frame_id = kBaseLink;
  req->header.stamp = node_->get_clock()->now();
  req->group_name = kPlanningGroup;
  req->max_step = 0.02;
  req->jump_threshold = 0.0;
  req->avoid_collisions = true;

  if (continuous) {
    auto waypoints = get_arc_waypoints_continuous(start_pose, mid_pose, end_pose, std::max(10, steps));
    if (waypoints.empty()) {
      return false;
    }
    req->waypoints = std::move(waypoints);
  } else {
    const int half = std::max(2, steps / 2);
    auto seg1 = interpolate_segment(start_pose, mid_pose, half);
    auto seg2 = interpolate_segment(mid_pose, end_pose, std::max(2, steps - half));
    req->waypoints.insert(req->waypoints.end(), seg1.begin(), seg1.end());
    req->waypoints.insert(req->waypoints.end(), seg2.begin(), seg2.end());
  }

  auto future = cartesian_client_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto response = future.get();
  if (!response) {
    return false;
  }
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    return false;
  }
  if (response->fraction < 0.90) {
    return false;
  }

  auto scaled = response->solution;
  scale_trajectory(scaled, vel_scale, acc_scale);
  return execute_trajectory(scaled);
}

bool ArmMotionExecutor::rotate_end_effector()
{
  // 原始实现：翻转180度（兼容旧接口，内部调用参数化版本）
  return rotate_end_effector_by_angle(180.0);
}

bool ArmMotionExecutor::rotate_end_effector_by_angle(double degrees)
{
  if (!planning_scene_client_ || node_ == nullptr) {
    return false;
  }

  // 1. 获取当前机器人关节状态
  auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  req->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE;
  auto future = planning_scene_client_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto res = future.get();
  if (!res) {
    return false;
  }

  const auto & names = res->scene.robot_state.joint_state.name;
  const auto & positions = res->scene.robot_state.joint_state.position;

  // 2. 定位末端关节名（兼容 joint6 或 arm_joint6 命名）
  std::string target_joint = "joint6";
  for (const auto & n : names) {
    if (n == "arm_joint6") {
      target_joint = "arm_joint6";
      break;
    }
  }

  // 3. 构建 MoveGroup Goal：只修改 target_joint，其他关节固定当前位置
  moveit_msgs::action::MoveGroup::Goal goal_msg;
  goal_msg.request.group_name = kPlanningGroup;

  moveit_msgs::msg::Constraints constraints;
  bool found = false;
  for (size_t i = 0; i < names.size() && i < positions.size(); ++i) {
    const std::string & name = names[i];
    // 跳过非关节或末端夹爪
    if (name.find("joint") == std::string::npos) {
      continue;
    }
    if (name.find("gripper") != std::string::npos || name.find("lift") != std::string::npos) {
      continue;
    }

    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = name;
    jc.weight = 1.0;
    jc.tolerance_above = 0.001;
    jc.tolerance_below = 0.001;
    if (name == target_joint) {
      // 关键：在当前角度基础上加上指定的角度偏移（度→弧度）
      const double angle_offset_rad = degrees * M_PI / 180.0;
      jc.position = positions[i] + angle_offset_rad;
      found = true;
    } else {
      jc.position = positions[i];
    }
    constraints.joint_constraints.push_back(jc);
  }

  if (!found) {
    return false;
  }

  goal_msg.request.goal_constraints.push_back(constraints);
  return send_goal_and_wait(goal_msg);
}

bool ArmMotionExecutor::get_cup_pose(
  int cup_id,
  geometry_msgs::msg::Pose & pre_grasp,
  geometry_msgs::msg::Pose & grasp,
  geometry_msgs::msg::Pose & up_grasp) const
{
  const double table_z = kChassisHeight + kJointGap + kTubeHeight + kStepHeight + kPlatformOffset;
  const double x = -(kBaseLen + kThickness + (kTopLen / 2.0));
  const double total_width = (kCupCount * kCupRadius * 2.0) + ((kCupCount - 1) * kCupGap);
  const double start_y = -(total_width / 2.0) + kCupRadius;
  const double cup_y = start_y + static_cast<double>(cup_id) * (kCupRadius * 2.0 + kCupGap);

  const double wrist_x = x + kGripperLength;
  const double wrist_y = cup_y;
  const double wrist_z = table_z + (kCupHeight / 2.0) - 0.01;

  grasp.position.x = wrist_x;
  grasp.position.y = wrist_y;
  grasp.position.z = wrist_z;
  grasp.orientation = rpy_to_quaternion(M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0);

  pre_grasp = grasp;
  pre_grasp.position.x = wrist_x + 0.09;

  up_grasp = grasp;
  up_grasp.position.z = wrist_z + 0.130;

  return true;
}

bool ArmMotionExecutor::send_goal_and_wait(const moveit_msgs::action::MoveGroup::Goal & goal_msg)
{
  if (!move_group_client_ || node_ == nullptr) {
    return false;
  }

  if (!move_group_client_->wait_for_action_server(std::chrono::seconds(3))) {
    return false;
  }

  auto goal_future = move_group_client_->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), goal_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    return false;
  }

  auto result_future = move_group_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto wrapped_result = result_future.get();
  return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED;
}

bool ArmMotionExecutor::execute_trajectory(const moveit_msgs::msg::RobotTrajectory & traj)
{
  if (!execute_client_ || node_ == nullptr) {
    return false;
  }

  if (!execute_client_->wait_for_action_server(std::chrono::seconds(3))) {
    return false;
  }

  moveit_msgs::action::ExecuteTrajectory::Goal goal_msg;
  goal_msg.trajectory = traj;

  auto goal_future = execute_client_->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), goal_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    return false;
  }

  auto result_future = execute_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto wrapped_result = result_future.get();
  return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED;
}

bool ArmMotionExecutor::get_current_pose(geometry_msgs::msg::Pose & pose_out) const
{
  if (!tf_buffer_) {
    return false;
  }

  try {
    auto t = tf_buffer_->lookupTransform(kBaseLink, kEndEffector, rclcpp::Time(0));
    pose_out.position.x = t.transform.translation.x;
    pose_out.position.y = t.transform.translation.y;
    pose_out.position.z = t.transform.translation.z;
    pose_out.orientation = t.transform.rotation;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool ArmMotionExecutor::init_gripper()
{
  gripper_socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (gripper_socket_fd_ < 0) {
    return false;
  }

  sockaddr_in server_addr {};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(kGripperPort);
  if (::inet_pton(AF_INET, kGripperIp, &server_addr.sin_addr) <= 0) {
    ::close(gripper_socket_fd_);
    gripper_socket_fd_ = -1;
    return false;
  }

  if (::connect(gripper_socket_fd_, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr)) < 0) {
    ::close(gripper_socket_fd_);
    gripper_socket_fd_ = -1;
    return false;
  }

  send_gripper_json("{\"command\":\"set_tool_voltage\",\"voltage_type\":3}\r\n");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  send_gripper_json("{\"command\":\"set_modbus_mode\",\"port\":1,\"baudrate\":115200,\"timeout\":2}\r\n");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  send_gripper_json("{\"command\":\"write_single_register\",\"port\":1,\"address\":257,\"data\":1,\"device\":1}\r\n");
  send_gripper_json("{\"command\":\"write_single_register\",\"port\":1,\"address\":256,\"data\":1,\"device\":1}\r\n");
  send_gripper_json("{\"command\":\"write_single_register\",\"port\":1,\"address\":260,\"data\":100,\"device\":1}\r\n");
  send_gripper_json("{\"command\":\"write_single_register\",\"port\":1,\"address\":261,\"data\":50,\"device\":1}\r\n");
  send_gripper_json("{\"command\":\"write_single_register\",\"port\":1,\"address\":262,\"data\":1000,\"device\":1}\r\n");
  return true;
}

bool ArmMotionExecutor::send_gripper_json(const std::string & payload) const
{
  if (gripper_socket_fd_ < 0) {
    return false;
  }
  const ssize_t n = ::send(gripper_socket_fd_, payload.c_str(), payload.size(), 0);
  if (n != static_cast<ssize_t>(payload.size())) {
    return false;
  }

  char buffer[256];
  const ssize_t recv_n = ::recv(gripper_socket_fd_, buffer, sizeof(buffer), MSG_DONTWAIT);
  if (recv_n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    return false;
  }
  return true;
}

geometry_msgs::msg::Quaternion ArmMotionExecutor::rpy_to_quaternion(double roll, double pitch, double yaw)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}
