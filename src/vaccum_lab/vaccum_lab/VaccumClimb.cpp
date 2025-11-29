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
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> node;
bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
std::atomic<bool> cancel_requested(false);
rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr current_goal_handle;

#define LEFT_ARM_HOME_POSITION 0.45942 //radian
#define LEFT_ARM_HOME_POSITION_BODY 0.45942 //radian
#define RIGHT_ARM_HOME_POSITION 0.69182 //radian
#define RIGHT_ARM_HOME_POSITION_BODY 0.69182 //radian

// Calculate the home position difference for maintaining offset
#define HOME_POSITION_OFFSET (LEFT_ARM_HOME_POSITION - RIGHT_ARM_HOME_POSITION) // Should be -0.2324

// Track goals reached for each joint position set
unsigned int ct_goals_reached = 0;

// Desired goals with indices: [left_arm, left_middle_arm, right_arm, right_middle_arm]
// Goals at indices 0 and 2 will have offset maintained
std::vector<std::vector<double>> desired_goals = {
  {-1.5,  1.5,  -1.5,  1.5},  // Goal 0: maintain offset between joints 0 and 2
  {-2.1,  2.0,  -2.1,  2.0},  // Goal 1: maintain offset between joints 0 and 2
  { 0.0,  2.0,  -2.1,  2.0},  // Goal 2: maintain offset between joints 0 and 2
  { 0.0,  6.28, -2.1,  2.0},
  { 2.0,  6.28, -2.1,  2.0},
  { 2.0,  6.28,  0.0,  2.0},
  { 2.0,  6.28,  0.0,  6.28},
  { 0.0,  6.28,  0.0,  6.28}
};

// Indices of goals where offset should be maintained (0-indexed)
std::vector<int> maintain_offset_goals = {0, 2};  // 1st and 3rd goals

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
      printf("Trajectory execution succeeded\n");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      printf("Trajectory execution aborted\n");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      printf("Trajectory execution canceled\n");
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

// Function to set terminal to non-blocking mode
void set_nonblocking_input()
{
  termios term;
  tcgetattr(STDIN_FILENO, &term);
  term.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &term);
  
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

// Function to restore terminal to normal mode
void restore_terminal()
{
  termios term;
  tcgetattr(STDIN_FILENO, &term);
  term.c_lflag |= (ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &term);
  
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

// Function to apply offset constraint to maintain constant difference between joint pairs
void apply_offset_constraint(std::vector<double>& positions, int goal_index)
{
  // Check if this goal requires offset maintenance
  bool maintain_offset = false;
  for (int idx : maintain_offset_goals) {
    if (idx == goal_index) {
      maintain_offset = true;
      break;
    }
  }
  
  if (!maintain_offset) {
    return;  // No constraint for this goal
  }
  
  // Joint indices: 0=left_arm, 1=left_middle_arm, 2=right_arm, 3=right_middle_arm
  // Maintain constant offset between left_arm (index 0) and right_arm (index 2)
  
  // The offset should be: left_arm - right_arm = HOME_POSITION_OFFSET
  // So: right_arm = left_arm - HOME_POSITION_OFFSET
  
  // Use left_arm position as reference and adjust right_arm to maintain offset
  positions[2] = positions[0] - HOME_POSITION_OFFSET;
  
  RCLCPP_INFO(node->get_logger(), 
    "Goal %d: Applied offset constraint. Left arm: %.4f, Right arm: %.4f, Offset: %.4f",
    goal_index, positions[0], positions[2], positions[0] - positions[2]);
}

// Thread function to monitor keyboard input for cancel
void keyboard_monitor_thread()
{
  set_nonblocking_input();
  
  std::cout << "\n===========================================\n";
  std::cout << "Press 'c' or 'ESC' to cancel trajectory\n";
  std::cout << "===========================================\n\n";
  
  while (!cancel_requested && rclcpp::ok()) {
    char ch;
    if (read(STDIN_FILENO, &ch, 1) == 1) {
      if (ch == 'c' || ch == 'C' || ch == 27) {  // 'c', 'C', or ESC key
        std::cout << "\n*** CANCEL REQUESTED - Stopping robot immediately! ***\n" << std::endl;
        cancel_requested = true;
        break;
      }
    }
    std::this_thread::sleep_for(100ms);
  }
  
  restore_terminal();
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
  
  // Start keyboard monitoring thread for emergency stop
  std::thread keyboard_thread(keyboard_monitor_thread);
  keyboard_thread.detach();

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
  
  RCLCPP_INFO(node->get_logger(), "Generating trajectory with offset constraints...");
  RCLCPP_INFO(node->get_logger(), "Home position offset: %.4f rad (%.2f deg)", 
    HOME_POSITION_OFFSET, HOME_POSITION_OFFSET * 180.0 / M_PI);
  
  for (size_t i = 0; i < desired_goals.size(); ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = desired_goals[i];
    
    // Apply offset constraint for specific goals
    apply_offset_constraint(point.positions, i);
    
    point.velocities = std::vector<double>(joint_names.size(), 0.0);
    t += 2.0; // 2 seconds between waypoints
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    points.push_back(point);
  }
  
  RCLCPP_INFO(node->get_logger(), "Trajectory generated with %zu waypoints", points.size());

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
    cancel_requested = true;
    rclcpp::shutdown();
    return 1;
  }
  
  // Store the goal handle globally so it can be cancelled
  current_goal_handle = goal_handle;

  auto result_future = action_client->async_get_result(goal_handle);
  RCLCPP_INFO(node->get_logger(), "Waiting for result (press 'c' or ESC to cancel)...");

  // Spin with periodic checks for cancel request
  while (rclcpp::ok() && !cancel_requested) {
    auto spin_result = rclcpp::spin_until_future_complete(
      node, result_future, std::chrono::milliseconds(100));
    
    if (spin_result == rclcpp::FutureReturnCode::SUCCESS) {
      // Goal completed normally
      break;
    }
    
    if (cancel_requested) {
      // User requested cancellation
      RCLCPP_WARN(node->get_logger(), "Canceling trajectory...");
      
      // Cancel the goal using async_cancel
      auto cancel_result_future = action_client->async_cancel_goal(goal_handle);
      
      // Wait for cancel to complete
      if (rclcpp::spin_until_future_complete(node, cancel_result_future, 2s) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
        auto cancel_result = cancel_result_future.get();
        if (cancel_result->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
          RCLCPP_INFO(node->get_logger(), "Goal successfully canceled");
        } else {
          RCLCPP_WARN(node->get_logger(), "Goal cancellation returned code: %d", 
                      cancel_result->return_code);
        }
      } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to cancel goal");
      }
      break;
    }
  }

  if (cancel_requested) {
    RCLCPP_WARN(node->get_logger(), "Robot stopped by user request");
  } else {
    RCLCPP_INFO(node->get_logger(), "Trajectory execution complete");
  }
  
  cancel_requested = true;  // Signal keyboard thread to stop
  std::this_thread::sleep_for(200ms);  // Give keyboard thread time to cleanup
  
  rclcpp::shutdown();
  return cancel_requested ? 2 : 0;  // Return 2 if cancelled, 0 if completed
}
