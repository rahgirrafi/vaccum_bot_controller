#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class ArmTrajectoryClient : public rclcpp::Node
{
public:
  ArmTrajectoryClient()
  : Node("arm_trajectory_client")
  {
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/arm_controller/follow_joint_trajectory");

    joint_names_ = {
      "left_arm_joint",
      "left_middle_arm_to_base_joint",
      "right_arm_joint",
      "right_middle_arm_to_base_joint"
    };

    send_goal();
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  std::vector<std::string> joint_names_;

  void send_goal()
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    if (!action_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names_;

    // Define trajectory points (8 waypoints)
    std::vector<std::vector<double>> points = {
      {-1.5,  1.5,  -1.5,  1.5},
      {-2.1,  2.0,  -2.1,  2.0},
      { 0.0,  2.0,  -2.1,  2.0},
      { 0.0,  6.28, -2.1,  2.0},
      { 2.0,  6.28, -2.1,  2.0},
      { 2.0,  6.28,  0.0, 2.0},
      { 2.0,  6.28,  0.0, 6.28},
      { 0.0,  6.28,  0.0, 6.28}
    };

    double time_from_start = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = points[i];
      pt.velocities = std::vector<double>(4, 0.0);
      time_from_start += 2.0;  // 2 seconds between points
      pt.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
      goal_msg.trajectory.points.push_back(pt);
    }

    // Send goal
    RCLCPP_INFO(this->get_logger(), "Sending trajectory goal...");
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](std::shared_future<rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr> future)
      {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
      };

    send_goal_options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr,
             const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
      {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Currently executing trajectory point...");
      };

    send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Trajectory execution succeeded");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Trajectory execution aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Trajectory execution canceled");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
        rclcpp::shutdown();
      };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmTrajectoryClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
