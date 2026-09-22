#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Settings
{
  std::string planning_group;
  std::string base_frame;
  std::string end_effector_link;
  std::string controller_action;
  double controller_wait_seconds;
  double plane_x;
  double center_y;
  double center_z;
  double letter_width;
  double letter_height;
  double pen_lift;
  double orientation_x;
  double orientation_y;
  double orientation_z;
  double orientation_w;
  int curve_points;
  double eef_step;
  double jump_threshold;
  double max_joint_step;
  double max_joint_travel;
  int approach_planning_attempts;
  double minimum_path_fraction;
  double velocity_scaling;
  double acceleration_scaling;
  double planning_time;
  double preview_seconds;
  double ground_z;
  double ground_thickness;
};

template<typename ValueT>
ValueT readParameter(
  const rclcpp::Node::SharedPtr& node, const std::string& name, const ValueT& default_value)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ValueT>(name, default_value);
  }
  return node->get_parameter(name).get_value<ValueT>();
}

Settings loadSettings(const rclcpp::Node::SharedPtr& node)
{
  Settings settings{
    readParameter<std::string>(node, "planning_group", "ur_manipulator"),
    readParameter<std::string>(node, "base_frame", "base_link"),
    readParameter<std::string>(node, "end_effector_link", "tool0"),
    readParameter<std::string>(
      node, "controller_action", "/joint_trajectory_controller/follow_joint_trajectory"),
    readParameter<double>(node, "controller_wait_seconds", 60.0),
    readParameter<double>(node, "plane_x", 0.30),
    readParameter<double>(node, "center_y", 0.0),
    readParameter<double>(node, "center_z", 0.24),
    readParameter<double>(node, "letter_width", 0.10),
    readParameter<double>(node, "letter_height", 0.14),
    readParameter<double>(node, "pen_lift", 0.03),
    readParameter<double>(node, "orientation_x", 0.0),
    readParameter<double>(node, "orientation_y", std::sqrt(0.5)),
    readParameter<double>(node, "orientation_z", 0.0),
    readParameter<double>(node, "orientation_w", std::sqrt(0.5)),
    readParameter<int>(node, "curve_points", 24),
    readParameter<double>(node, "eef_step", 0.005),
    readParameter<double>(node, "jump_threshold", 0.0),
    readParameter<double>(node, "max_joint_step", 0.20),
    readParameter<double>(node, "max_joint_travel", 5.0),
    readParameter<int>(node, "approach_planning_attempts", 5),
    readParameter<double>(node, "minimum_path_fraction", 0.995),
    readParameter<double>(node, "velocity_scaling", 0.15),
    readParameter<double>(node, "acceleration_scaling", 0.15),
    readParameter<double>(node, "planning_time", 10.0),
    readParameter<double>(node, "preview_seconds", 2.0),
    readParameter<double>(node, "ground_z", 0.0),
    readParameter<double>(node, "ground_thickness", 0.02),
  };

  if (settings.controller_wait_seconds <= 0.0 || settings.letter_width <= 0.0 ||
      settings.letter_height <= 0.0 || settings.pen_lift <= 0.0) {
    throw std::invalid_argument("letter_width, letter_height and pen_lift must be positive");
  }
  if (settings.curve_points < 4 || settings.approach_planning_attempts < 1) {
    throw std::invalid_argument(
            "curve_points must be at least 4 and approach_planning_attempts must be positive");
  }
  if (settings.eef_step <= 0.0 || settings.max_joint_step <= 0.0 ||
      settings.max_joint_travel <= 0.0 || settings.ground_thickness <= 0.0 ||
      settings.minimum_path_fraction <= 0.0 ||
      settings.minimum_path_fraction > 1.0) {
    throw std::invalid_argument("invalid Cartesian path settings");
  }
  if (settings.velocity_scaling <= 0.0 || settings.velocity_scaling > 1.0 ||
      settings.acceleration_scaling <= 0.0 || settings.acceleration_scaling > 1.0) {
    throw std::invalid_argument("velocity and acceleration scaling must be in (0, 1]");
  }

  const double quaternion_norm = std::sqrt(
    settings.orientation_x * settings.orientation_x +
    settings.orientation_y * settings.orientation_y +
    settings.orientation_z * settings.orientation_z +
    settings.orientation_w * settings.orientation_w);
  if (quaternion_norm < 1e-9) {
    throw std::invalid_argument("tool orientation quaternion must not be zero");
  }
  settings.orientation_x /= quaternion_norm;
  settings.orientation_y /= quaternion_norm;
  settings.orientation_z /= quaternion_norm;
  settings.orientation_w /= quaternion_norm;
  return settings;
}

geometry_msgs::msg::Pose makePose(
  const Settings& settings, double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = settings.orientation_x;
  pose.orientation.y = settings.orientation_y;
  pose.orientation.z = settings.orientation_z;
  pose.orientation.w = settings.orientation_w;
  return pose;
}

geometry_msgs::msg::Pose liftedPose(const geometry_msgs::msg::Pose& pose, double pen_lift)
{
  auto lifted = pose;
  lifted.position.x -= pen_lift;
  return lifted;
}

std::vector<std::vector<geometry_msgs::msg::Pose>> makeLetterD(const Settings& settings)
{
  const double left_y = settings.center_y - settings.letter_width / 2.0;
  const double top_z = settings.center_z + settings.letter_height / 2.0;
  const double bottom_z = settings.center_z - settings.letter_height / 2.0;

  std::vector<geometry_msgs::msg::Pose> stem;
  stem.push_back(makePose(settings, settings.plane_x, left_y, top_z));
  stem.push_back(makePose(settings, settings.plane_x, left_y, bottom_z));

  std::vector<geometry_msgs::msg::Pose> curve;
  curve.reserve(static_cast<std::size_t>(settings.curve_points + 1));
  for (int index = 0; index <= settings.curve_points; ++index) {
    const double ratio = static_cast<double>(index) / settings.curve_points;
    const double angle = kPi / 2.0 - kPi * ratio;
    const double y = left_y + settings.letter_width * std::cos(angle);
    const double z = settings.center_z + settings.letter_height / 2.0 * std::sin(angle);
    curve.push_back(makePose(settings, settings.plane_x, y, z));
  }
  return {stem, curve};
}

class PathVisualizer
{
public:
  PathVisualizer(
    rclcpp::Node::SharedPtr node, std::string base_frame, std::string end_effector_link)
  : node_(std::move(node)),
    base_frame_(std::move(base_frame)),
    end_effector_link_(std::move(end_effector_link)),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_, node_, false)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "letter_path_markers", qos);
    sample_timer_ = node_->create_wall_timer(40ms, [this]() { sampleActualPath(); });
  }

  void publishIntended(const std::vector<std::vector<geometry_msgs::msg::Pose>>& strokes)
  {
    for (std::size_t stroke_index = 0; stroke_index < strokes.size(); ++stroke_index) {
      auto marker = baseMarker("intended_letter_d", static_cast<int>(stroke_index));
      marker.scale.x = 0.006;
      marker.color.r = 0.10F;
      marker.color.g = 0.90F;
      marker.color.b = 0.20F;
      marker.color.a = 1.0F;
      for (const auto& pose : strokes[stroke_index]) {
        marker.points.push_back(pose.position);
      }
      marker_publisher_->publish(marker);
    }
  }

  void startActualStroke(int stroke_id)
  {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    active_marker_ = baseMarker("actual_tool_path", 100 + stroke_id);
    active_marker_.scale.x = 0.004;
    active_marker_.color.r = 1.0F;
    active_marker_.color.g = 0.25F;
    active_marker_.color.b = 0.05F;
    active_marker_.color.a = 1.0F;
    recording_.store(true);
    marker_publisher_->publish(active_marker_);
  }

  void stopActualStroke()
  {
    recording_.store(false);
    std::lock_guard<std::mutex> lock(trace_mutex_);
    active_marker_.header.stamp = node_->now();
    marker_publisher_->publish(active_marker_);
  }

private:
  visualization_msgs::msg::Marker baseMarker(const std::string& marker_namespace, int id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = node_->now();
    marker.ns = marker_namespace;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  void sampleActualPath()
  {
    if (!recording_.load()) {
      return;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, end_effector_link_, tf2::TimePointZero);
      geometry_msgs::msg::Point point;
      point.x = transform.transform.translation.x;
      point.y = transform.transform.translation.y;
      point.z = transform.transform.translation.z;

      std::lock_guard<std::mutex> lock(trace_mutex_);
      if (!active_marker_.points.empty()) {
        const auto& previous = active_marker_.points.back();
        const double distance = std::hypot(
          std::hypot(point.x - previous.x, point.y - previous.y), point.z - previous.z);
        if (distance < 0.0005) {
          return;
        }
      }
      active_marker_.header.stamp = node_->now();
      active_marker_.points.push_back(point);
      marker_publisher_->publish(active_marker_);
    } catch (const tf2::TransformException&) {
      // TF can be unavailable briefly while the simulator starts.
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string base_frame_;
  std::string end_effector_link_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::atomic<bool> recording_{false};
  std::mutex trace_mutex_;
  visualization_msgs::msg::Marker active_marker_;
};

void addGroundCollisionObject(const Settings& settings, const rclcpp::Logger& logger)
{
  moveit_msgs::msg::CollisionObject ground;
  ground.header.frame_id = settings.base_frame;
  ground.id = "letter_writer_ground";

  shape_msgs::msg::SolidPrimitive ground_shape;
  ground_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  ground_shape.dimensions = {2.0, 2.0, settings.ground_thickness};

  geometry_msgs::msg::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.position.z = settings.ground_z - settings.ground_thickness / 2.0;

  ground.primitives.push_back(ground_shape);
  ground.primitive_poses.push_back(ground_pose);
  ground.operation = moveit_msgs::msg::CollisionObject::ADD;

  moveit::planning_interface::PlanningSceneInterface planning_scene;
  if (!planning_scene.applyCollisionObject(ground)) {
    RCLCPP_WARN(logger, "Could not add the ground plane to the MoveIt planning scene");
  } else {
    RCLCPP_INFO(logger, "Added a collision floor at z=%.3f m", settings.ground_z);
  }
}

bool prepareAndValidateTrajectory(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::RobotState& start_state,
  moveit_msgs::msg::RobotTrajectory& trajectory_message,
  const Settings& settings,
  const rclcpp::Logger& logger,
  const std::string& motion_name,
  bool parameterize_time,
  double* total_joint_travel = nullptr)
{
  robot_trajectory::RobotTrajectory robot_trajectory(
    move_group.getRobotModel(), settings.planning_group);
  robot_trajectory.setRobotTrajectoryMsg(start_state, trajectory_message);

  // Keep continuous joints close to the measured start position. Equivalent
  // IK solutions may otherwise be represented 2*pi apart.
  robot_trajectory.unwind(start_state);
  const auto* joint_model_group =
    move_group.getRobotModel()->getJointModelGroup(settings.planning_group);
  for (std::size_t point_index = 0;
    point_index < robot_trajectory.getWayPointCount(); ++point_index)
  {
    if (!robot_trajectory.getWayPoint(point_index).satisfiesBounds(joint_model_group)) {
      RCLCPP_ERROR(
        logger, "%s violates a joint limit at point %zu", motion_name.c_str(), point_index);
      return false;
    }
  }
  robot_trajectory.getRobotTrajectoryMsg(trajectory_message);

  const auto& points = trajectory_message.joint_trajectory.points;
  const auto& joint_names = trajectory_message.joint_trajectory.joint_names;
  if (points.empty() || joint_names.empty()) {
    RCLCPP_ERROR(logger, "%s produced an empty trajectory", motion_name.c_str());
    return false;
  }

  std::vector<double> previous_positions;
  std::vector<double> accumulated_travel(joint_names.size(), 0.0);
  previous_positions.reserve(joint_names.size());
  for (const auto& joint_name : joint_names) {
    previous_positions.push_back(start_state.getVariablePosition(joint_name));
  }
  for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
    if (points[point_index].positions.size() != previous_positions.size()) {
      RCLCPP_ERROR(logger, "%s contains inconsistent joint data", motion_name.c_str());
      return false;
    }
    for (std::size_t joint_index = 0;
      joint_index < points[point_index].positions.size(); ++joint_index)
    {
      const double joint_step = std::abs(
        points[point_index].positions[joint_index] - previous_positions[joint_index]);
      if (joint_step > settings.max_joint_step) {
        RCLCPP_ERROR(
          logger, "%s has an unsafe %.3f rad jump at joint %s",
          motion_name.c_str(), joint_step, joint_names[joint_index].c_str());
        return false;
      }
      accumulated_travel[joint_index] += joint_step;
      if (accumulated_travel[joint_index] > settings.max_joint_travel) {
        RCLCPP_ERROR(
          logger,
          "%s would rotate joint %s by %.3f rad in total (limit %.3f rad); refusing to execute",
          motion_name.c_str(), joint_names[joint_index].c_str(),
          accumulated_travel[joint_index], settings.max_joint_travel);
        return false;
      }
    }
    previous_positions = points[point_index].positions;
  }

  if (total_joint_travel != nullptr) {
    *total_joint_travel = 0.0;
    for (const double joint_travel : accumulated_travel) {
      *total_joint_travel += joint_travel;
    }
  }

  if (parameterize_time) {
    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        robot_trajectory, settings.velocity_scaling, settings.acceleration_scaling)) {
      RCLCPP_ERROR(logger, "Could not time-parameterize %s", motion_name.c_str());
      return false;
    }
    robot_trajectory.getRobotTrajectoryMsg(trajectory_message);
  }
  return true;
}

bool planAndMoveToSafePose(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const geometry_msgs::msg::Pose& target,
  const Settings& settings,
  const rclcpp::Logger& logger)
{
  auto start_state = move_group.getCurrentState(5.0);
  if (!start_state) {
    RCLCPP_ERROR(logger, "No current robot state before the initial approach");
    return false;
  }

  move_group.setStartState(*start_state);
  if (!move_group.setPoseTarget(target, settings.end_effector_link)) {
    RCLCPP_ERROR(logger, "MoveIt rejected the initial approach pose");
    move_group.setStartStateToCurrentState();
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan best_plan;
  double best_travel = std::numeric_limits<double>::infinity();
  bool found_safe_plan = false;
  for (int attempt = 1; attempt <= settings.approach_planning_attempts; ++attempt) {
    moveit::planning_interface::MoveGroupInterface::Plan candidate;
    if (!static_cast<bool>(move_group.plan(candidate))) {
      RCLCPP_WARN(
        logger, "Initial approach planning attempt %d/%d failed",
        attempt, settings.approach_planning_attempts);
      continue;
    }

    double candidate_travel = 0.0;
    if (!prepareAndValidateTrajectory(
        move_group, *start_state, candidate.trajectory_, settings, logger,
        "initial approach candidate", false, &candidate_travel)) {
      RCLCPP_WARN(
        logger, "Rejected initial approach candidate %d/%d",
        attempt, settings.approach_planning_attempts);
      continue;
    }
    RCLCPP_INFO(
      logger, "Initial approach candidate %d/%d: total joint travel %.3f rad",
      attempt, settings.approach_planning_attempts, candidate_travel);
    if (candidate_travel < best_travel) {
      best_travel = candidate_travel;
      best_plan = std::move(candidate);
      found_safe_plan = true;
    }
  }

  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();
  if (!found_safe_plan) {
    RCLCPP_ERROR(
      logger,
      "No collision-free initial approach passed the joint-travel safety limits");
    return false;
  }
  RCLCPP_INFO(
    logger, "Executing initial approach with %.3f rad total joint travel", best_travel);
  if (!static_cast<bool>(move_group.execute(best_plan))) {
    RCLCPP_ERROR(logger, "Failed to execute the initial approach");
    return false;
  }
  return true;
}

bool planAndExecuteCartesian(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const std::vector<geometry_msgs::msg::Pose>& waypoints,
  const Settings& settings,
  const rclcpp::Logger& logger,
  const std::string& motion_name)
{
  auto start_state = move_group.getCurrentState(5.0);
  if (!start_state) {
    RCLCPP_ERROR(logger, "No current robot state before %s", motion_name.c_str());
    return false;
  }
  move_group.setStartState(*start_state);

  moveit_msgs::msg::RobotTrajectory trajectory_message;
  const double fraction = move_group.computeCartesianPath(
    waypoints, settings.eef_step, settings.jump_threshold, trajectory_message, true);
  move_group.setStartStateToCurrentState();

  RCLCPP_INFO(
    logger, "%s: Cartesian path %.1f%%", motion_name.c_str(), fraction * 100.0);
  if (fraction < settings.minimum_path_fraction) {
    RCLCPP_ERROR(
      logger, "%s is incomplete; refusing to execute it", motion_name.c_str());
    return false;
  }

  if (!prepareAndValidateTrajectory(
      move_group, *start_state, trajectory_message, settings, logger, motion_name, true)) {
    return false;
  }

  if (!static_cast<bool>(move_group.execute(trajectory_message))) {
    RCLCPP_ERROR(logger, "Execution failed for %s", motion_name.c_str());
    return false;
  }
  return true;
}

int runWriter(const rclcpp::Node::SharedPtr& node)
{
  const auto logger = node->get_logger();
  const Settings settings = loadSettings(node);
  const auto strokes = makeLetterD(settings);
  PathVisualizer visualizer(node, settings.base_frame, settings.end_effector_link);
  visualizer.publishIntended(strokes);

  RCLCPP_INFO(logger, "Waiting for MoveIt and the UR robot state...");
  moveit::planning_interface::MoveGroupInterface move_group(node, settings.planning_group);
  move_group.setPoseReferenceFrame(settings.base_frame);
  if (!move_group.setEndEffectorLink(settings.end_effector_link)) {
    RCLCPP_ERROR(
      logger, "Unknown end-effector link '%s'", settings.end_effector_link.c_str());
    return 1;
  }
  move_group.setPlanningTime(settings.planning_time);
  // The initial approach explicitly samples and compares multiple plans.
  // Keep each MoveIt request to one attempt so every candidate is independent.
  move_group.setNumPlanningAttempts(1);
  move_group.setMaxVelocityScalingFactor(settings.velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(settings.acceleration_scaling);

  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  auto controller_client = rclcpp_action::create_client<FollowJointTrajectory>(
    node, settings.controller_action);
  RCLCPP_INFO(
    logger, "Waiting for controller action %s...", settings.controller_action.c_str());
  if (!controller_client->wait_for_action_server(
      std::chrono::duration<double>(settings.controller_wait_seconds))) {
    RCLCPP_ERROR(
      logger, "Controller action %s did not become available",
      settings.controller_action.c_str());
    return 1;
  }

  if (!move_group.startStateMonitor(30.0) || !move_group.getCurrentState(5.0)) {
    RCLCPP_ERROR(logger, "Robot joint state did not become available");
    return 1;
  }

  addGroundCollisionObject(settings, logger);

  RCLCPP_INFO(
    logger,
    "Ready. Drawing D in the YZ plane at x=%.3f m (width %.3f m, height %.3f m)",
    settings.plane_x, settings.letter_width, settings.letter_height);
  if (settings.preview_seconds > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(settings.preview_seconds));
  }

  const auto first_pen_up_start = liftedPose(strokes.front().front(), settings.pen_lift);
  RCLCPP_INFO(logger, "Planning a collision-free approach above stroke 1");
  if (!planAndMoveToSafePose(move_group, first_pen_up_start, settings, logger)) {
    return 1;
  }

  for (std::size_t stroke_index = 0; stroke_index < strokes.size(); ++stroke_index) {
    const auto& stroke = strokes[stroke_index];
    const auto pen_up_start = liftedPose(stroke.front(), settings.pen_lift);
    const auto pen_up_end = liftedPose(stroke.back(), settings.pen_lift);
    const std::string stroke_name = "stroke " + std::to_string(stroke_index + 1);

    if (stroke_index > 0) {
      RCLCPP_INFO(logger, "Moving above %s with the tool lifted", stroke_name.c_str());
      if (!planAndExecuteCartesian(
          move_group, {pen_up_start}, settings, logger,
          "pen-up transition to " + stroke_name)) {
        return 1;
      }
    }
    if (!planAndExecuteCartesian(
        move_group, {stroke.front()}, settings, logger, "lowering tool for " + stroke_name)) {
      return 1;
    }

    RCLCPP_INFO(logger, "Drawing %s", stroke_name.c_str());
    visualizer.startActualStroke(static_cast<int>(stroke_index));
    const bool stroke_succeeded = planAndExecuteCartesian(
      move_group, stroke, settings, logger, stroke_name);
    visualizer.stopActualStroke();
    if (!stroke_succeeded) {
      return 1;
    }

    if (!planAndExecuteCartesian(
        move_group, {pen_up_end}, settings, logger, "lifting tool after " + stroke_name)) {
      return 1;
    }
  }

  RCLCPP_INFO(logger, "Finished drawing the letter D");
  return 0;
}
}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "letter_writer", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  int result = 1;
  try {
    result = runWriter(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(node->get_logger(), "Letter writer failed: %s", error.what());
  }

  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return result;
}
