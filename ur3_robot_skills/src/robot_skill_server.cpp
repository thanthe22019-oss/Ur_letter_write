#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/transport/Node.hh>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ur3_robot_skills/action/execute_skill.hpp"

using namespace std::chrono_literals;

namespace
{
using ExecuteSkill = ur3_robot_skills::action::ExecuteSkill;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteSkill>;

enum class Status : std::uint8_t
{
  SUCCESS = ExecuteSkill::Result::SUCCESS,
  FAILED = ExecuteSkill::Result::FAILED,
  INVALID_OBJECT = ExecuteSkill::Result::INVALID_OBJECT,
  INVALID_ZONE = ExecuteSkill::Result::INVALID_ZONE,
  PLANNING_FAILED = ExecuteSkill::Result::PLANNING_FAILED,
  EXECUTION_FAILED = ExecuteSkill::Result::EXECUTION_FAILED,
  INVALID_SKILL = ExecuteSkill::Result::INVALID_SKILL,
};

struct SkillResult
{
  Status status;
  std::string message;

  bool ok() const { return status == Status::SUCCESS; }
};

struct Zone
{
  geometry_msgs::msg::Pose pose;
  std::vector<double> size;
};

struct Settings
{
  std::string planning_group;
  std::string base_frame;
  std::string end_effector_link;
  std::string home_target;
  std::string controller_action;
  std::string world_name;
  std::vector<std::string> valid_objects;
  std::vector<std::string> valid_zones;
  std::map<std::string, Zone> zones;
  std::vector<double> tool_orientation;
  double controller_wait_seconds;
  double approach_height;
  double grasp_clearance;
  double place_clearance;
  double planning_time;
  int planning_attempts;
  int ik_attempts;
  double ik_timeout;
  double velocity_scaling;
  double acceleration_scaling;
  double cartesian_step;
  double minimum_path_fraction;
  double max_joint_step;
  double max_joint_travel;
  double scene_wait_seconds;
  int gazebo_service_timeout_ms;
};

template<typename ValueT>
ValueT parameter(
  const rclcpp::Node::SharedPtr& node, const std::string& name,
  const ValueT& default_value)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ValueT>(name, default_value);
  }
  return node->get_parameter(name).get_value<ValueT>();
}

geometry_msgs::msg::Pose poseFromArray(const std::vector<double>& values)
{
  if (values.size() != 6) {
    throw std::invalid_argument("zone pose must contain [x, y, z, roll, pitch, yaw]");
  }
  if (std::any_of(values.begin() + 3, values.end(), [](double value) {
      return std::abs(value) > 1.0e-9;
    })) {
    throw std::invalid_argument("Milestone 3 supports axis-aligned zones only");
  }
  geometry_msgs::msg::Pose pose;
  pose.position.x = values[0];
  pose.position.y = values[1];
  pose.position.z = values[2];
  pose.orientation.w = 1.0;
  return pose;
}

Settings loadSettings(const rclcpp::Node::SharedPtr& node)
{
  Settings settings;
  settings.planning_group = parameter<std::string>(node, "planning_group", "ur_manipulator");
  settings.base_frame = parameter<std::string>(node, "base_frame", "base_link");
  settings.end_effector_link = parameter<std::string>(node, "end_effector_link", "tool0");
  settings.home_target = parameter<std::string>(node, "home_target", "home");
  settings.controller_action = parameter<std::string>(
    node, "controller_action", "/joint_trajectory_controller/follow_joint_trajectory");
  settings.world_name = parameter<std::string>(node, "world_name", "empty");
  settings.valid_objects = parameter<std::vector<std::string>>(node, "valid_objects", {});
  settings.valid_zones = parameter<std::vector<std::string>>(node, "valid_zones", {});
  settings.tool_orientation = parameter<std::vector<double>>(
    node, "tool_orientation", {1.0, 0.0, 0.0, 0.0});
  settings.controller_wait_seconds = parameter<double>(node, "controller_wait_seconds", 90.0);
  settings.approach_height = parameter<double>(node, "approach_height", 0.12);
  settings.grasp_clearance = parameter<double>(node, "grasp_clearance", 0.015);
  settings.place_clearance = parameter<double>(node, "place_clearance", 0.003);
  settings.planning_time = parameter<double>(node, "planning_time", 10.0);
  settings.planning_attempts = parameter<int>(node, "planning_attempts", 6);
  settings.ik_attempts = parameter<int>(node, "ik_attempts", 20);
  settings.ik_timeout = parameter<double>(node, "ik_timeout", 0.10);
  settings.velocity_scaling = parameter<double>(node, "velocity_scaling", 0.15);
  settings.acceleration_scaling = parameter<double>(node, "acceleration_scaling", 0.15);
  settings.cartesian_step = parameter<double>(node, "cartesian_step", 0.005);
  settings.minimum_path_fraction = parameter<double>(node, "minimum_path_fraction", 0.995);
  settings.max_joint_step = parameter<double>(node, "max_joint_step", 0.35);
  settings.max_joint_travel = parameter<double>(node, "max_joint_travel", 5.5);
  settings.scene_wait_seconds = parameter<double>(node, "scene_wait_seconds", 30.0);
  settings.gazebo_service_timeout_ms = parameter<int>(node, "gazebo_service_timeout_ms", 3000);

  if (settings.valid_objects.empty() || settings.valid_zones.empty()) {
    throw std::invalid_argument("valid_objects and valid_zones must come from scene.yaml");
  }
  if (settings.tool_orientation.size() != 4) {
    throw std::invalid_argument("tool_orientation must contain [x, y, z, w]");
  }
  double quaternion_norm = 0.0;
  for (const double value : settings.tool_orientation) {
    quaternion_norm += value * value;
  }
  quaternion_norm = std::sqrt(quaternion_norm);
  if (quaternion_norm < 1.0e-9) {
    throw std::invalid_argument("tool_orientation must not be zero");
  }
  for (double& value : settings.tool_orientation) {
    value /= quaternion_norm;
  }

  if (settings.controller_wait_seconds <= 0.0 || settings.approach_height <= 0.0 ||
      settings.grasp_clearance < 0.0 || settings.place_clearance < 0.0 ||
      settings.planning_time <= 0.0 || settings.planning_attempts < 1 ||
      settings.ik_attempts < 1 || settings.ik_timeout <= 0.0 ||
      settings.cartesian_step <= 0.0 || settings.minimum_path_fraction <= 0.0 ||
      settings.minimum_path_fraction > 1.0 || settings.max_joint_step <= 0.0 ||
      settings.max_joint_travel <= 0.0 || settings.scene_wait_seconds <= 0.0 ||
      settings.gazebo_service_timeout_ms <= 0) {
    throw std::invalid_argument("invalid motion setting");
  }
  if (settings.velocity_scaling <= 0.0 || settings.velocity_scaling > 1.0 ||
      settings.acceleration_scaling <= 0.0 || settings.acceleration_scaling > 1.0) {
    throw std::invalid_argument("velocity and acceleration scaling must be in (0, 1]");
  }

  for (const auto& zone_name : settings.valid_zones) {
    const auto pose_values = parameter<std::vector<double>>(
      node, "zones." + zone_name + ".pose", {});
    const auto size = parameter<std::vector<double>>(
      node, "zones." + zone_name + ".size", {});
    if (size.size() != 3 || std::any_of(size.begin(), size.end(), [](double value) {
        return value <= 0.0;
      })) {
      throw std::invalid_argument("zone size must contain three positive values");
    }
    settings.zones.emplace(zone_name, Zone{poseFromArray(pose_values), size});
  }
  return settings;
}

const char* statusName(Status status)
{
  switch (status) {
    case Status::SUCCESS: return "SUCCESS";
    case Status::FAILED: return "FAILED";
    case Status::INVALID_OBJECT: return "INVALID_OBJECT";
    case Status::INVALID_ZONE: return "INVALID_ZONE";
    case Status::PLANNING_FAILED: return "PLANNING_FAILED";
    case Status::EXECUTION_FAILED: return "EXECUTION_FAILED";
    case Status::INVALID_SKILL: return "INVALID_SKILL";
  }
  return "FAILED";
}

bool contains(const std::vector<std::string>& values, const std::string& value)
{
  return std::find(values.begin(), values.end(), value) != values.end();
}

geometry_msgs::msg::Pose composePoses(
  const geometry_msgs::msg::Pose& parent,
  const geometry_msgs::msg::Pose& child)
{
  const double x = parent.orientation.x;
  const double y = parent.orientation.y;
  const double z = parent.orientation.z;
  const double w = parent.orientation.w;
  const double tx = 2.0 * (y * child.position.z - z * child.position.y);
  const double ty = 2.0 * (z * child.position.x - x * child.position.z);
  const double tz = 2.0 * (x * child.position.y - y * child.position.x);

  geometry_msgs::msg::Pose result;
  result.position.x = parent.position.x + child.position.x + w * tx + y * tz - z * ty;
  result.position.y = parent.position.y + child.position.y + w * ty + z * tx - x * tz;
  result.position.z = parent.position.z + child.position.z + w * tz + x * ty - y * tx;
  result.orientation.x = w * child.orientation.x + x * child.orientation.w +
    y * child.orientation.z - z * child.orientation.y;
  result.orientation.y = w * child.orientation.y - x * child.orientation.z +
    y * child.orientation.w + z * child.orientation.x;
  result.orientation.z = w * child.orientation.z + x * child.orientation.y -
    y * child.orientation.x + z * child.orientation.w;
  result.orientation.w = w * child.orientation.w - x * child.orientation.x -
    y * child.orientation.y - z * child.orientation.z;
  return result;
}

struct ObjectInfo
{
  moveit_msgs::msg::CollisionObject collision;
  geometry_msgs::msg::Pose pose;
  std::vector<double> size;
};

class RobotSkills
{
public:
  using Feedback = std::function<void(const std::string&)>;

  explicit RobotSkills(rclcpp::Node::SharedPtr node)
  : node_(std::move(node)),
    settings_(loadSettings(node_)),
    move_group_(node_, settings_.planning_group)
  {
    compute_ik_client_ = node_->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
    move_group_.setPoseReferenceFrame(settings_.base_frame);
    if (!move_group_.setEndEffectorLink(settings_.end_effector_link)) {
      throw std::runtime_error(
              "unknown end-effector link '" + settings_.end_effector_link + "'");
    }
    move_group_.setPlanningTime(settings_.planning_time);
    move_group_.setNumPlanningAttempts(1);
    move_group_.setMaxVelocityScalingFactor(settings_.velocity_scaling);
    move_group_.setMaxAccelerationScalingFactor(settings_.acceleration_scaling);
  }

  bool initialize()
  {
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    auto controller = rclcpp_action::create_client<FollowJointTrajectory>(
      node_, settings_.controller_action);
    RCLCPP_INFO(
      node_->get_logger(), "Waiting for controller action %s...",
      settings_.controller_action.c_str());
    if (!controller->wait_for_action_server(
        std::chrono::duration<double>(settings_.controller_wait_seconds))) {
      RCLCPP_ERROR(node_->get_logger(), "Controller action was not available");
      return false;
    }
    if (!move_group_.startStateMonitor(30.0) || !move_group_.getCurrentState(5.0)) {
      RCLCPP_ERROR(node_->get_logger(), "Robot joint state was not available");
      return false;
    }
    if (!compute_ik_client_->wait_for_service(30s)) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt /compute_ik service was not available");
      return false;
    }

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(settings_.scene_wait_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      if (planning_scene_.getObjects(settings_.valid_objects).size() ==
          settings_.valid_objects.size()) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Robot skills ready: %zu objects and %zu zones loaded from scene.yaml",
          settings_.valid_objects.size(), settings_.valid_zones.size());
        return true;
      }
      std::this_thread::sleep_for(200ms);
    }
    RCLCPP_ERROR(node_->get_logger(), "Configured cubes were not available in MoveIt");
    return false;
  }

  SkillResult execute(const ExecuteSkill::Goal& goal, const Feedback& feedback)
  {
    if (goal.skill == "home") {
      return home(feedback);
    }
    if (goal.skill == "move_above") {
      return moveAbove(goal.object_name, feedback);
    }
    if (goal.skill == "pick") {
      return pick(goal.object_name, feedback);
    }
    if (goal.skill == "place") {
      return place(goal.object_name, goal.zone_name, feedback);
    }
    return {Status::INVALID_SKILL, "unsupported skill '" + goal.skill + "'"};
  }

private:
  SkillResult home(const Feedback& feedback)
  {
    feedback("planning home");
    auto current = move_group_.getCurrentState(5.0);
    if (!current) {
      return {Status::FAILED, "current robot state is unavailable"};
    }
    move_group_.setStartState(*current);
    if (!move_group_.setNamedTarget(settings_.home_target)) {
      move_group_.setStartStateToCurrentState();
      return {Status::PLANNING_FAILED, "MoveIt does not define the home target"};
    }
    auto result = planCurrentTarget("home", *current, feedback);
    move_group_.setStartStateToCurrentState();
    return result;
  }

  SkillResult moveAbove(const std::string& object_name, const Feedback& feedback)
  {
    if (!contains(settings_.valid_objects, object_name)) {
      return {Status::INVALID_OBJECT, "unknown object '" + object_name + "'"};
    }
    if (!held_object_.empty() && held_object_ == object_name && held_info_) {
      return moveToPose(
        abovePoseForObject(*held_info_), "move above held " + object_name, feedback);
    }
    auto object = findWorldObject(object_name);
    if (!object) {
      return {Status::FAILED, "object '" + object_name + "' is absent from MoveIt"};
    }
    return moveToPose(abovePoseForObject(*object), "move above " + object_name, feedback);
  }

  SkillResult pick(const std::string& object_name, const Feedback& feedback)
  {
    if (!contains(settings_.valid_objects, object_name)) {
      return {Status::INVALID_OBJECT, "unknown object '" + object_name + "'"};
    }
    if (!held_object_.empty()) {
      return {Status::FAILED, "already holding '" + held_object_ + "'"};
    }
    auto object = findWorldObject(object_name);
    if (!object) {
      return {Status::FAILED, "object '" + object_name + "' is absent from MoveIt"};
    }

    auto result = moveToPose(
      abovePoseForObject(*object), "pick approach for " + object_name, feedback);
    if (!result.ok()) {
      return result;
    }
    openGripper(feedback);

    feedback("lowering to " + object_name);
    result = executeCartesian(
      graspPoseForObject(*object), "lower to " + object_name, feedback);
    if (!result.ok()) {
      return result;
    }
    closeGripper(feedback);

    feedback("attaching " + object_name);
    const std::vector<std::string> touch_links{
      settings_.end_effector_link, "flange", "wrist_3_link"};
    if (!move_group_.attachObject(object_name, settings_.end_effector_link, touch_links) ||
        !waitForAttached(object_name, true, 3.0)) {
      openGripper(feedback);
      return {Status::FAILED, "MoveIt could not attach '" + object_name + "'"};
    }
    held_object_ = object_name;
    held_info_ = *object;

    feedback("retreating with " + object_name);
    result = executeCartesian(
      abovePoseForObject(*object), "retreat with " + object_name, feedback);
    if (!result.ok()) {
      return result;
    }
    return {Status::SUCCESS, "picked '" + object_name + "'"};
  }

  SkillResult place(
    const std::string& object_name, const std::string& zone_name,
    const Feedback& feedback)
  {
    if (!contains(settings_.valid_objects, object_name)) {
      return {Status::INVALID_OBJECT, "unknown object '" + object_name + "'"};
    }
    if (!contains(settings_.valid_zones, zone_name)) {
      return {Status::INVALID_ZONE, "unknown zone '" + zone_name + "'"};
    }
    if (held_object_ != object_name || !held_info_) {
      return {Status::FAILED, "robot is not holding '" + object_name + "'"};
    }
    const auto zone = settings_.zones.at(zone_name);
    const auto placed_object_pose = placedPose(*held_info_, zone);
    auto settled_object_pose = placed_object_pose;
    settled_object_pose.position.z -= settings_.place_clearance;
    const auto release_pose = releaseToolPose(*held_info_, placed_object_pose);
    auto above_release_pose = release_pose;
    above_release_pose.position.z += settings_.approach_height;

    auto result = moveToPose(
      above_release_pose, "move above " + zone_name, feedback);
    if (!result.ok()) {
      return result;
    }

    feedback("lowering into " + zone_name);
    result = executeCartesian(release_pose, "lower into " + zone_name, feedback);
    if (!result.ok()) {
      return result;
    }

    openGripper(feedback);
    feedback("detaching " + object_name);
    if (!move_group_.detachObject(object_name) || !waitForAttached(object_name, false, 3.0)) {
      return {Status::FAILED, "MoveIt could not detach '" + object_name + "'"};
    }

    auto placed_collision = held_info_->collision;
    if (placed_collision.primitive_poses.empty()) {
      return {Status::FAILED, "collision object has no pose"};
    }
    placed_collision.pose = settled_object_pose;
    placed_collision.primitive_poses[0] = geometry_msgs::msg::Pose();
    placed_collision.primitive_poses[0].orientation.w = 1.0;
    placed_collision.operation = moveit_msgs::msg::CollisionObject::ADD;
    if (!planning_scene_.applyCollisionObject(placed_collision)) {
      return {Status::FAILED, "could not update object pose in MoveIt"};
    }

    const bool gazebo_updated = setGazeboPose(object_name, settled_object_pose);
    held_object_.clear();
    held_info_.reset();

    feedback("retreating from " + zone_name);
    result = executeCartesian(above_release_pose, "retreat from " + zone_name, feedback);
    if (!result.ok()) {
      return result;
    }
    if (!gazebo_updated) {
      return {Status::FAILED, "placed in MoveIt but Gazebo pose synchronization failed"};
    }
    return {
      Status::SUCCESS,
      "placed '" + object_name + "' in '" + zone_name + "'"};
  }

  geometry_msgs::msg::Pose toolPose(double x, double y, double z) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = settings_.tool_orientation[0];
    pose.orientation.y = settings_.tool_orientation[1];
    pose.orientation.z = settings_.tool_orientation[2];
    pose.orientation.w = settings_.tool_orientation[3];
    return pose;
  }

  geometry_msgs::msg::Pose graspPoseForObject(const ObjectInfo& object) const
  {
    return toolPose(
      object.pose.position.x, object.pose.position.y,
      object.pose.position.z + object.size[2] / 2.0 + settings_.grasp_clearance);
  }

  geometry_msgs::msg::Pose abovePoseForObject(const ObjectInfo& object) const
  {
    auto pose = graspPoseForObject(object);
    pose.position.z += settings_.approach_height;
    return pose;
  }

  geometry_msgs::msg::Pose placedPose(const ObjectInfo& object, const Zone& zone) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = zone.pose.position.x;
    pose.position.y = zone.pose.position.y;
    pose.position.z = zone.pose.position.z + zone.size[2] / 2.0 +
      object.size[2] / 2.0 + settings_.place_clearance;
    pose.orientation.w = 1.0;
    return pose;
  }

  geometry_msgs::msg::Pose releaseToolPose(
    const ObjectInfo& object, const geometry_msgs::msg::Pose& placed_object_pose) const
  {
    return toolPose(
      placed_object_pose.position.x, placed_object_pose.position.y,
      placed_object_pose.position.z + object.size[2] / 2.0 + settings_.grasp_clearance);
  }

  std::optional<ObjectInfo> findWorldObject(const std::string& object_name)
  {
    const auto objects = planning_scene_.getObjects({object_name});
    const auto found = objects.find(object_name);
    if (found == objects.end() || found->second.primitives.empty() ||
        found->second.primitive_poses.empty() ||
        found->second.primitives.front().dimensions.size() != 3) {
      return std::nullopt;
    }
    const auto& dimensions = found->second.primitives.front().dimensions;
    return ObjectInfo{
      found->second,
      composePoses(found->second.pose, found->second.primitive_poses.front()),
      std::vector<double>(dimensions.begin(), dimensions.end())};
  }

  SkillResult moveToPose(
    const geometry_msgs::msg::Pose& target, const std::string& motion_name,
    const Feedback& feedback)
  {
    auto current = move_group_.getCurrentState(5.0);
    if (!current) {
      return {Status::FAILED, "current robot state is unavailable"};
    }
    feedback("planning " + motion_name);
    move_group_.setStartState(*current);
    // Resolve several IK candidates and keep the one closest to the measured
    // joints. A raw pose constraint can select an equivalent UR solution that
    // is represented almost 2*pi away on a wrist joint.
    if (!setNearestIKTarget(target, *current)) {
      move_group_.setStartStateToCurrentState();
      return {Status::PLANNING_FAILED, "MoveIt found no IK solution for " + motion_name};
    }
    auto result = planCurrentTarget(motion_name, *current, feedback);
    move_group_.setStartStateToCurrentState();
    return result;
  }

  bool setNearestIKTarget(
    const geometry_msgs::msg::Pose& target,
    const moveit::core::RobotState& current_state)
  {
    const auto* joint_group =
      move_group_.getRobotModel()->getJointModelGroup(settings_.planning_group);
    if (!joint_group) {
      return false;
    }

    moveit::core::RobotState best_state(current_state);
    double best_distance = std::numeric_limits<double>::infinity();
    bool found = false;
    for (int attempt = 0; attempt < settings_.ik_attempts; ++attempt) {
      moveit::core::RobotState candidate(current_state);
      auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
      request->ik_request.group_name = settings_.planning_group;
      request->ik_request.ik_link_name = settings_.end_effector_link;
      request->ik_request.pose_stamped.header.frame_id = settings_.base_frame;
      request->ik_request.pose_stamped.pose = target;
      request->ik_request.avoid_collisions = true;
      request->ik_request.timeout = rclcpp::Duration::from_seconds(settings_.ik_timeout);
      // An empty diff tells MoveGroup to seed IK from its monitored planning
      // scene. Without is_diff=true Humble treats the empty JointState as a
      // complete state and rejects an otherwise reachable pose.
      request->ik_request.robot_state.is_diff = true;

      if (attempt == 0) {
        RCLCPP_INFO(
          node_->get_logger(),
          "IK target in %s: [%.3f, %.3f, %.3f] q=[%.3f, %.3f, %.3f, %.3f]",
          settings_.base_frame.c_str(), target.position.x, target.position.y,
          target.position.z, target.orientation.x, target.orientation.y,
          target.orientation.z, target.orientation.w);
      }

      auto future = compute_ik_client_->async_send_request(request);
      if (future.wait_for(std::chrono::duration<double>(settings_.ik_timeout + 1.0)) !=
          std::future_status::ready) {
        continue;
      }
      const auto response = future.get();
      if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        if (attempt == 0) {
          RCLCPP_WARN(
            node_->get_logger(), "IK request failed with MoveIt error code %d",
            response->error_code.val);
        }
        continue;
      }
      if (!moveit::core::robotStateMsgToRobotState(response->solution, candidate)) {
        RCLCPP_WARN(node_->get_logger(), "MoveIt returned an invalid IK robot state");
        continue;
      }
      candidate.harmonizePositions(joint_group);
      if (!candidate.satisfiesBounds(joint_group)) {
        continue;
      }
      const double distance = current_state.distance(candidate, joint_group);
      if (distance < best_distance) {
        best_state = candidate;
        best_distance = distance;
        found = true;
      }
    }
    if (!found) {
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(), "Nearest IK candidate distance: %.3f rad", best_distance);
    return move_group_.setJointValueTarget(best_state);
  }

  SkillResult planCurrentTarget(
    const std::string& motion_name, const moveit::core::RobotState& start_state,
    const Feedback& feedback)
  {
    moveit::planning_interface::MoveGroupInterface::Plan best_plan;
    double best_travel = std::numeric_limits<double>::infinity();
    bool found = false;
    for (int attempt = 1; attempt <= settings_.planning_attempts; ++attempt) {
      moveit::planning_interface::MoveGroupInterface::Plan candidate;
      if (!static_cast<bool>(move_group_.plan(candidate))) {
        RCLCPP_WARN(
          node_->get_logger(), "%s planning attempt %d/%d failed",
          motion_name.c_str(), attempt, settings_.planning_attempts);
        continue;
      }
      double joint_travel = 0.0;
      if (!validateTrajectory(
          start_state, candidate.trajectory_, motion_name, false, &joint_travel)) {
        continue;
      }
      if (joint_travel < best_travel) {
        best_plan = std::move(candidate);
        best_travel = joint_travel;
        found = true;
      }
    }
    if (!found) {
      return {
        Status::PLANNING_FAILED,
        "no collision-free, joint-safe plan for " + motion_name};
    }

    RCLCPP_INFO(
      node_->get_logger(), "%s selected with %.3f rad total joint travel",
      motion_name.c_str(), best_travel);
    feedback("executing " + motion_name);
    if (!static_cast<bool>(move_group_.execute(best_plan))) {
      return {Status::EXECUTION_FAILED, "controller failed during " + motion_name};
    }
    return {Status::SUCCESS, motion_name + " completed"};
  }

  SkillResult executeCartesian(
    const geometry_msgs::msg::Pose& target, const std::string& motion_name,
    const Feedback& feedback)
  {
    auto start_state = move_group_.getCurrentState(5.0);
    if (!start_state) {
      return {Status::FAILED, "current robot state is unavailable"};
    }
    move_group_.setStartState(*start_state);
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group_.computeCartesianPath(
      {target}, settings_.cartesian_step, 0.0, trajectory, true);
    move_group_.setStartStateToCurrentState();
    RCLCPP_INFO(
      node_->get_logger(), "%s Cartesian path: %.1f%%",
      motion_name.c_str(), fraction * 100.0);
    if (fraction < settings_.minimum_path_fraction) {
      return {
        Status::PLANNING_FAILED,
        motion_name + " Cartesian path is incomplete"};
    }
    if (!validateTrajectory(*start_state, trajectory, motion_name, true, nullptr)) {
      return {
        Status::PLANNING_FAILED,
        motion_name + " violates joint safety limits"};
    }
    feedback("executing " + motion_name);
    if (!static_cast<bool>(move_group_.execute(trajectory))) {
      return {Status::EXECUTION_FAILED, "controller failed during " + motion_name};
    }
    return {Status::SUCCESS, motion_name + " completed"};
  }

  bool validateTrajectory(
    const moveit::core::RobotState& start_state,
    moveit_msgs::msg::RobotTrajectory& trajectory_message,
    const std::string& motion_name, bool parameterize_time,
    double* total_joint_travel)
  {
    robot_trajectory::RobotTrajectory trajectory(
      move_group_.getRobotModel(), settings_.planning_group);
    trajectory.setRobotTrajectoryMsg(start_state, trajectory_message);
    trajectory.unwind(start_state);
    const auto* joint_group =
      move_group_.getRobotModel()->getJointModelGroup(settings_.planning_group);
    for (std::size_t index = 0; index < trajectory.getWayPointCount(); ++index) {
      if (!trajectory.getWayPoint(index).satisfiesBounds(joint_group)) {
        RCLCPP_ERROR(
          node_->get_logger(), "%s violates a joint limit", motion_name.c_str());
        return false;
      }
    }
    trajectory.getRobotTrajectoryMsg(trajectory_message);

    const auto& joint_names = trajectory_message.joint_trajectory.joint_names;
    const auto& points = trajectory_message.joint_trajectory.points;
    if (joint_names.empty() || points.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "%s produced an empty trajectory", motion_name.c_str());
      return false;
    }
    std::vector<double> previous;
    std::vector<double> travel(joint_names.size(), 0.0);
    for (const auto& joint_name : joint_names) {
      previous.push_back(start_state.getVariablePosition(joint_name));
    }
    for (const auto& point : points) {
      if (point.positions.size() != previous.size()) {
        return false;
      }
      for (std::size_t index = 0; index < point.positions.size(); ++index) {
        const double step = std::abs(point.positions[index] - previous[index]);
        if (step > settings_.max_joint_step) {
          RCLCPP_WARN(
            node_->get_logger(),
            "%s rejected: joint %s step %.3f rad exceeds %.3f rad",
            motion_name.c_str(), joint_names[index].c_str(), step,
            settings_.max_joint_step);
          return false;
        }
        travel[index] += step;
        if (travel[index] > settings_.max_joint_travel) {
          RCLCPP_WARN(
            node_->get_logger(),
            "%s rejected: joint %s travel %.3f rad exceeds %.3f rad",
            motion_name.c_str(), joint_names[index].c_str(), travel[index],
            settings_.max_joint_travel);
          return false;
        }
      }
      previous = point.positions;
    }
    if (total_joint_travel) {
      *total_joint_travel = 0.0;
      for (const double value : travel) {
        *total_joint_travel += value;
      }
    }
    if (parameterize_time) {
      trajectory_processing::IterativeParabolicTimeParameterization timing;
      if (!timing.computeTimeStamps(
          trajectory, settings_.velocity_scaling, settings_.acceleration_scaling)) {
        return false;
      }
      trajectory.getRobotTrajectoryMsg(trajectory_message);
    }
    return true;
  }

  bool waitForAttached(const std::string& object_name, bool attached, double timeout_seconds)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      const bool currently_attached =
        !planning_scene_.getAttachedObjects({object_name}).empty();
      if (currently_attached == attached) {
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  bool setGazeboPose(
    const std::string& object_name, const geometry_msgs::msg::Pose& pose)
  {
    ignition::msgs::Pose request;
    request.set_name(object_name);
    request.mutable_position()->set_x(pose.position.x);
    request.mutable_position()->set_y(pose.position.y);
    request.mutable_position()->set_z(pose.position.z);
    request.mutable_orientation()->set_x(pose.orientation.x);
    request.mutable_orientation()->set_y(pose.orientation.y);
    request.mutable_orientation()->set_z(pose.orientation.z);
    request.mutable_orientation()->set_w(pose.orientation.w);

    ignition::msgs::Boolean response;
    bool service_result = false;
    const std::string service = "/world/" + settings_.world_name + "/set_pose";
    const bool executed = gazebo_node_.Request(
      service, request,
      static_cast<unsigned int>(settings_.gazebo_service_timeout_ms),
      response, service_result);
    if (!executed || !service_result || !response.data()) {
      RCLCPP_ERROR(
        node_->get_logger(), "Gazebo service %s could not move %s",
        service.c_str(), object_name.c_str());
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(), "Gazebo pose synchronized for %s", object_name.c_str());
    return true;
  }

  void openGripper(const Feedback& feedback)
  {
    feedback("opening logical gripper");
    RCLCPP_INFO(node_->get_logger(), "Logical gripper: OPEN");
  }

  void closeGripper(const Feedback& feedback)
  {
    feedback("closing logical gripper");
    RCLCPP_INFO(node_->get_logger(), "Logical gripper: CLOSED");
  }

  rclcpp::Node::SharedPtr node_;
  Settings settings_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;
  ignition::transport::Node gazebo_node_;
  std::string held_object_;
  std::optional<ObjectInfo> held_info_;
};

class SkillActionServer
{
public:
  SkillActionServer(rclcpp::Node::SharedPtr node, std::shared_ptr<RobotSkills> skills)
  : node_(std::move(node)), skills_(std::move(skills))
  {
    server_ = rclcpp_action::create_server<ExecuteSkill>(
      node_, "execute_skill",
      [this](const rclcpp_action::GoalUUID&,
        std::shared_ptr<const ExecuteSkill::Goal> goal) {
        if (busy_.exchange(true)) {
          RCLCPP_WARN(node_->get_logger(), "Rejecting '%s': server is busy", goal->skill.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread([this, goal_handle]() { run(goal_handle); }).detach();
      });
  }

private:
  void run(const std::shared_ptr<GoalHandle>& goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(
      node_->get_logger(), "Executing skill '%s' object='%s' zone='%s'",
      goal->skill.c_str(), goal->object_name.c_str(), goal->zone_name.c_str());
    const auto skill_result = skills_->execute(
      *goal, [goal_handle](const std::string& phase) {
        auto feedback = std::make_shared<ExecuteSkill::Feedback>();
        feedback->phase = phase;
        goal_handle->publish_feedback(feedback);
      });

    auto result = std::make_shared<ExecuteSkill::Result>();
    result->code = static_cast<std::uint8_t>(skill_result.status);
    result->status = statusName(skill_result.status);
    result->message = skill_result.message;
    goal_handle->succeed(result);
    busy_.store(false);
    RCLCPP_INFO(
      node_->get_logger(), "Skill '%s': %s - %s", goal->skill.c_str(),
      result->status.c_str(), result->message.c_str());
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<RobotSkills> skills_;
  rclcpp_action::Server<ExecuteSkill>::SharedPtr server_;
  std::atomic<bool> busy_{false};
};
}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "robot_skill_server",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  int exit_code = 1;
  try {
    auto skills = std::make_shared<RobotSkills>(node);
    if (skills->initialize()) {
      auto server = std::make_shared<SkillActionServer>(node, skills);
      RCLCPP_INFO(node->get_logger(), "Action /execute_skill is ready");
      spin_thread.join();
      exit_code = 0;
    } else {
      executor.cancel();
      spin_thread.join();
    }
  } catch (const std::exception& error) {
    RCLCPP_FATAL(node->get_logger(), "Robot skill server failed: %s", error.what());
    executor.cancel();
    spin_thread.join();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
