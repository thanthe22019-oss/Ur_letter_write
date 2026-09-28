#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ur3_robot_skills/action/execute_skill.hpp"

using namespace std::chrono_literals;

namespace
{
using ExecuteSkill = ur3_robot_skills::action::ExecuteSkill;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteSkill>;

struct DemoStep
{
  std::string skill;
  std::string object_name;
  std::string zone_name;
};

bool runStep(
  const rclcpp::Node::SharedPtr& node,
  const rclcpp_action::Client<ExecuteSkill>::SharedPtr& client,
  const DemoStep& step)
{
  ExecuteSkill::Goal goal;
  goal.skill = step.skill;
  goal.object_name = step.object_name;
  goal.zone_name = step.zone_name;

  RCLCPP_INFO(
    node->get_logger(), "DEMO: %s(%s%s%s)", step.skill.c_str(),
    step.object_name.c_str(),
    step.object_name.empty() || step.zone_name.empty() ? "" : ", ",
    step.zone_name.c_str());

  rclcpp_action::Client<ExecuteSkill>::SendGoalOptions options;
  options.feedback_callback =
    [logger = node->get_logger()](GoalHandle::SharedPtr,
      const std::shared_ptr<const ExecuteSkill::Feedback> feedback) {
      RCLCPP_INFO(logger, "  phase: %s", feedback->phase.c_str());
    };

  auto goal_handle = client->async_send_goal(goal, options).get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Skill server rejected '%s'", step.skill.c_str());
    return false;
  }

  const auto wrapped_result = client->async_get_result(goal_handle).get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node->get_logger(), "Skill '%s' did not return a result", step.skill.c_str());
    return false;
  }

  const auto& result = wrapped_result.result;
  RCLCPP_INFO(
    node->get_logger(), "  result: %s - %s",
    result->status.c_str(), result->message.c_str());
  return result->code == ExecuteSkill::Result::SUCCESS;
}
}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "robot_skill_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const double wait_seconds = node->get_parameter_or("server_wait_seconds", 120.0);
  const double start_delay_seconds = node->get_parameter_or("start_delay_seconds", 3.0);
  const std::string object_name = node->get_parameter_or(
    "object_name", std::string("red_cube"));
  const std::string zone_name = node->get_parameter_or(
    "zone_name", std::string("zone_b"));

  auto client = rclcpp_action::create_client<ExecuteSkill>(node, "execute_skill");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  int exit_code = 1;
  if (!client->wait_for_action_server(std::chrono::duration<double>(wait_seconds))) {
    RCLCPP_ERROR(node->get_logger(), "Skill action server was not available");
  } else {
    RCLCPP_INFO(
      node->get_logger(), "Waiting %.1f s for MoveIt controller discovery...",
      start_delay_seconds);
    std::this_thread::sleep_for(std::chrono::duration<double>(start_delay_seconds));
    const std::vector<DemoStep> steps{
      {"home", "", ""},
      {"move_above", object_name, ""},
      {"pick", object_name, ""},
      {"place", object_name, zone_name},
    };

    bool success = true;
    for (const auto& step : steps) {
      if (!runStep(node, client, step)) {
        success = false;
        RCLCPP_ERROR(
          node->get_logger(), "Milestone 3 stopped after '%s' failed", step.skill.c_str());
        break;
      }
    }
    if (success) {
      RCLCPP_INFO(
        node->get_logger(),
        "MILESTONE 3 SUCCESS: %s was placed in %s",
        object_name.c_str(), zone_name.c_str());
      exit_code = 0;
    }
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
