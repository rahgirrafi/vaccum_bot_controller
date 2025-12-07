#ifndef VACCUM_SYSTEM_HPP_
#define VACCUM_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace vaccum_control
{
class VaccumSystem: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VaccumSystem)

  // Initialize the hardware
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // Configure the hardware
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  // Activate the hardware
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Deactivate the hardware
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Define state interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Define command interfaces
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read from hardware (unused for LED)
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Write to hardware
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  rclcpp::Logger get_logger() const { return *logger_; }
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
    void encoder_counts_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    rclcpp::Node::SharedPtr node_;
  
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr enc_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  double hw_start_sec_;
  double hw_stop_sec_;

  double rear_left_wheel_position_;
  double rear_right_wheel_position_;
  double front_left_wheel_position_;
  double front_right_wheel_position_;
  double rear_left_wheel_velocity_;
  double rear_right_wheel_velocity_;
  double front_left_wheel_velocity_;
  double front_right_wheel_velocity_;
  
  double rear_left_wheel_command_;
  double rear_right_wheel_command_;
  double front_left_wheel_command_;
  double front_right_wheel_command_;

  double left_middle_arm_position_;
  double right_middle_arm_position_;
  double left_arm_position_;
  double right_arm_position_;
  double left_middle_arm_velocity_;
  double right_middle_arm_velocity_;
  double left_arm_velocity_;
  double right_arm_velocity_;

  double left_middle_arm_command_;
  double right_middle_arm_command_;
  double left_arm_command_;
  double right_arm_command_;

std::shared_ptr<rclcpp::Logger> logger_;
rclcpp::Clock::SharedPtr clock_;


};

}  // namespace ros2_control_demo_example_2

#endif 