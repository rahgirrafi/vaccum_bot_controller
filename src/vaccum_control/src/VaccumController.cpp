// Copyright 2025
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> node;
bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

// Track goals reached for each joint position set
unsigned int ct_goals_reached = 0;

std::vector<std::vector<double>> desired_goals = {
  {-1.5,  1.5,  -1.5,  1.5},
  {-2.1,  2.0,  -2.1,  2.0},
  { 0.0,  2.0,  -2.1,  2.0},
  { 0.0,  6.28, -2.1,  2.0},
  { 2.0,  6.28, -2.1,  2.0},
  { 2.0,  6.28,  0.0,  2.0},
  { 2.0,  6.28,  0.0,  6.28},
  { 0.0,  6.28,  0.0,  6.28}
};

void common_goal_response(
  rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::SharedPtr future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    common_goal_accepted = false;
    printf("Goal rejected\n");
  } else {
    common_goal_accepted = true;
    printf("Goal accepted\n");
  }
}

void common_result_response(
  const rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
{
  printf("Result received at time: %f\n", rclcpp::Clock(RCL_ROS_TIME).now().seconds());
  common_resultcode = result.code;
  common_action_result_code = result.result->error_code;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      printf("Trajectory execution succeeded ✅\n");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      printf("Trajectory execution aborted ❌\n");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      printf("Trajectory execution canceled ⚠️\n");
      return;
    default:
      printf("Unknown result code\n");
      return;
  }
}

void common_feedback(
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
  const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
  std::cout << "Feedback received: desired position[0] = " << feedback->desired.positions[0]
            << ", actual position[0] = " << feedback->actual.positions[0] << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("arm_trajectory_action_client");
  RCLCPP_INFO(node->get_logger(), "Node created");

  // Create action client
  auto action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node->get_node_base_interface(),
    node->get_node_graph_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "/arm_controller/follow_joint_trajectory");

  // Wait for server connection
  while (!action_client->wait_for_action_server(2s)) {
    RCLCPP_WARN(node->get_logger(), "Waiting for action server...");
  }
  RCLCPP_INFO(node->get_logger(), "Connected to action server");

  // Define joint names (must match your controller)
  std::vector<std::string> joint_names = {
    "left_arm_joint",
    "left_middle_arm_to_base_joint",
    "right_arm_joint",
    "right_middle_arm_to_base_joint"
  };

  // Create trajectory points
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
  double t = 0.0;
  for (const auto & goal : desired_goals) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = goal;
    point.velocities = std::vector<double>(joint_names.size(), 0.0);
    t += 2.0; // 2 seconds between waypoints
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    points.push_back(point);
  }

  // Define goal options
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
  opt.goal_response_callback = std::bind(common_goal_response, std::placeholders::_1);
  opt.result_callback = std::bind(common_result_response, std::placeholders::_1);
  opt.feedback_callback = std::bind(common_feedback, std::placeholders::_1, std::placeholders::_2);

  // Create and populate goal message
  control_msgs::action::FollowJointTrajectory_Goal goal_msg;
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
  goal_msg.trajectory.joint_names = joint_names;
  goal_msg.trajectory.points = points;

  // Send goal
  RCLCPP_INFO(node->get_logger(), "Sending trajectory goal...");
  auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Goal sent successfully");

  // Wait for execution result
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
    return 1;
  }

  auto result_future = action_client->async_get_result(goal_handle);
  RCLCPP_INFO(node->get_logger(), "Waiting for result...");

  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Get result call failed");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Trajectory execution complete ✅");
  rclcpp::shutdown();
  return 0;
}
