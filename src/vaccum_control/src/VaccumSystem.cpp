#include "vaccum_control/VaccumSystem.hpp"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define ENCODER_SAMPLE_MS 100
#define TICKS_PER_REV 1440.0f // placeholder value
#define WHEEL_RADIUS 0.022f // actual value in meters

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
        
namespace vaccum_control
{

hardware_interface::CallbackReturn VaccumSystem::on_init(const hardware_interface::HardwareInfo &info){
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  VaccumSystem::node_ = rclcpp::Node::make_shared("vaccum_system");
  VaccumSystem::enc_sub_ = VaccumSystem::node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/encoder_counts", rclcpp::QoS(10),
    std::bind(&VaccumSystem::encoder_counts_callback, this, std::placeholders::_1));
  
  VaccumSystem::left_arm_angle_sub_ = VaccumSystem::node_->create_subscription<std_msgs::msg::Float32>(
    "/left_arm_angle_rad", rclcpp::QoS(10),
    std::bind(&VaccumSystem::left_arm_angle_callback, this, std::placeholders::_1));
  
  VaccumSystem::right_arm_angle_sub_ = VaccumSystem::node_->create_subscription<std_msgs::msg::Float32>(
    "/right_arm_angle_rad", rclcpp::QoS(10),
    std::bind(&VaccumSystem::right_arm_angle_callback, this, std::placeholders::_1));

  VaccumSystem::cmd_pub_ = VaccumSystem::node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    // Initialize member variables to prevent null access
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("VaccumSystem"));
  clock_ = std::make_shared<rclcpp::Clock>();

  return hardware_interface::CallbackReturn::SUCCESS;
}

double last_left1 = 0;
double last_right1 = 0;
double last_left2 = 0;
double last_right2 = 0;

double left_pos_left1 = 0;
double left_pos_right1 = 0;
double left_pos_left2 = 0;
double left_pos_right2 = 0;

void VaccumSystem::encoder_counts_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{

  // [7] = right_arm RPM (AS5600 sensor 1)
  
  if (msg->data.size() >= 8) {
    // Extract wheel encoder data (RPS - revolutions per second)
    float rear_left_rps = static_cast<double>(msg->data[0]);
    float rear_right_rps = static_cast<double>(msg->data[1]);
    float front_left_rps = static_cast<double>(msg->data[2]);
    float front_right_rps = static_cast<double>(msg->data[3]);

    // Convert RPS to linear velocity (m/s): v = rps * 2π * radius
    rear_left_wheel_velocity_ = rear_left_rps * 2.0 * M_PI * WHEEL_RADIUS;
    rear_right_wheel_velocity_ = rear_right_rps * 2.0 * M_PI * WHEEL_RADIUS;
    front_left_wheel_velocity_ = front_left_rps * 2.0 * M_PI * WHEEL_RADIUS;
    front_right_wheel_velocity_ = front_right_rps * 2.0 * M_PI * WHEEL_RADIUS;
    
    // Update wheel positions by integrating velocity over time
    float dt = (float)ENCODER_SAMPLE_MS / 1000.0f;
    
    // Position in radians = velocity * dt / radius
    // Or simply: radians = rps * 2π * dt
    left_pos_left1 += rear_left_rps * 2.0 * M_PI * dt;
    left_pos_right1 += rear_right_rps * 2.0 * M_PI * dt;
    left_pos_left2 += front_left_rps * 2.0 * M_PI * dt;
    left_pos_right2 += front_right_rps * 2.0 * M_PI * dt;

    rear_left_wheel_position_ = left_pos_left1;
    rear_right_wheel_position_ = left_pos_right1;
    front_left_wheel_position_ = left_pos_left2;
    front_right_wheel_position_ = left_pos_right2;

    // Extract AS5600 arm joint sensor data
    float left_middle_arm_radians = static_cast<double>(msg->data[4]);
    float left_middle_arm_rpm = static_cast<double>(msg->data[5]);
    float right_middle_arm_radians = static_cast<double>(msg->data[6]);
    float right_middle_arm_rpm = static_cast<double>(msg->data[7]);

    // Convert degrees to radians for ROS control
    left_middle_arm_position_ = left_middle_arm_radians;
    right_middle_arm_position_ = right_middle_arm_radians;

    // Convert RPM to radians per second: rad/s = rpm * 2π / 60
    left_middle_arm_velocity_ = left_middle_arm_rpm * 2.0 * M_PI / 60.0;
    right_middle_arm_velocity_ = right_middle_arm_rpm * 2.0 * M_PI / 60.0;

    // Note: middle arm positions/velocities would need additional sensors
    // For now, these remain at their default values or can be derived from arm positions
    
  } else if (msg->data.size() >= 4) {
    // Fallback for older firmware that only publishes 4 elements
    RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 5000, 
      "Received only %zu encoder elements, expected 8 (wheels + AS5600 data)", msg->data.size());
    
    float rear_left_rps = static_cast<double>(msg->data[0]);
    float rear_right_rps = static_cast<double>(msg->data[1]);
    float front_left_rps = static_cast<double>(msg->data[2]);
    float front_right_rps = static_cast<double>(msg->data[3]);

    rear_left_wheel_velocity_ = rear_left_rps * 2.0 * M_PI * WHEEL_RADIUS;
    rear_right_wheel_velocity_ = rear_right_rps * 2.0 * M_PI * WHEEL_RADIUS;
    front_left_wheel_velocity_ = front_left_rps * 2.0 * M_PI * WHEEL_RADIUS;
    front_right_wheel_velocity_ = front_right_rps * 2.0 * M_PI * WHEEL_RADIUS;
    
    float dt = (float)ENCODER_SAMPLE_MS / 1000.0f;
    left_pos_left1 += rear_left_rps * 2.0 * M_PI * dt;
    left_pos_right1 += rear_right_rps * 2.0 * M_PI * dt;
    left_pos_left2 += front_left_rps * 2.0 * M_PI * dt;
    left_pos_right2 += front_right_rps * 2.0 * M_PI * dt;

    rear_left_wheel_position_ = left_pos_left1;
    rear_right_wheel_position_ = left_pos_right1;
    front_left_wheel_position_ = left_pos_left2;
    front_right_wheel_position_ = left_pos_right2;
  }
}

void VaccumSystem::left_arm_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // Update left arm position from the angle topic (already in radians)
  left_arm_position_ = static_cast<double>(msg->data);
  // Log periodically for debugging (every ~100 messages)
  static int log_counter_left = 0;
  if (++log_counter_left % 100 == 0) {
    RCLCPP_DEBUG(get_logger(), "Left arm angle received: %.4f rad", left_arm_position_);
  }
}

void VaccumSystem::right_arm_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // Update right arm position from the angle topic (already in radians)
  right_arm_position_ = static_cast<double>(msg->data);
  // Log periodically for debugging (every ~100 messages)
  static int log_counter_right = 0;
  if (++log_counter_right % 100 == 0) {
    RCLCPP_DEBUG(get_logger(), "Right arm angle received: %.4f rad", right_arm_position_);
  }
}

hardware_interface::CallbackReturn VaccumSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring hardware");
 
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VaccumSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating hardware");

 
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VaccumSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating hardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VaccumSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back("rear_left_wheel_joint", hardware_interface::HW_IF_POSITION, &rear_left_wheel_position_);
  state_interfaces.emplace_back("rear_right_wheel_joint", hardware_interface::HW_IF_POSITION, &rear_right_wheel_position_);
  state_interfaces.emplace_back("front_left_wheel_joint", hardware_interface::HW_IF_POSITION, &front_left_wheel_position_);
  state_interfaces.emplace_back("front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &front_right_wheel_position_);
  state_interfaces.emplace_back("left_middle_arm_to_base_joint", hardware_interface::HW_IF_POSITION, &left_middle_arm_position_);
  state_interfaces.emplace_back("right_middle_arm_to_base_joint", hardware_interface::HW_IF_POSITION, &right_middle_arm_position_);
  state_interfaces.emplace_back("left_arm_joint", hardware_interface::HW_IF_POSITION, &left_arm_position_);
  state_interfaces.emplace_back("right_arm_joint", hardware_interface::HW_IF_POSITION, &right_arm_position_);

  state_interfaces.emplace_back("rear_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &rear_left_wheel_velocity_);
  state_interfaces.emplace_back("rear_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &rear_right_wheel_velocity_);
  state_interfaces.emplace_back("front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &front_left_wheel_velocity_);
  state_interfaces.emplace_back("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &front_right_wheel_velocity_);
  state_interfaces.emplace_back("left_middle_arm_to_base_joint", hardware_interface::HW_IF_VELOCITY, &left_middle_arm_velocity_);
  state_interfaces.emplace_back("right_middle_arm_to_base_joint", hardware_interface::HW_IF_VELOCITY, &right_middle_arm_velocity_);
  state_interfaces.emplace_back("left_arm_joint", hardware_interface::HW_IF_VELOCITY, &left_arm_velocity_);
  state_interfaces.emplace_back("right_arm_joint", hardware_interface::HW_IF_VELOCITY, &right_arm_velocity_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VaccumSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back("rear_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &rear_left_wheel_command_);
  command_interfaces.emplace_back("rear_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &rear_right_wheel_command_);
  command_interfaces.emplace_back("front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &front_left_wheel_command_);
  command_interfaces.emplace_back("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &front_right_wheel_command_);
  command_interfaces.emplace_back("left_middle_arm_to_base_joint", hardware_interface::HW_IF_POSITION, &left_middle_arm_command_);
  command_interfaces.emplace_back("right_middle_arm_to_base_joint", hardware_interface::HW_IF_POSITION, &right_middle_arm_command_);
  command_interfaces.emplace_back("left_arm_joint", hardware_interface::HW_IF_POSITION, &left_arm_command_);
  command_interfaces.emplace_back("right_arm_joint", hardware_interface::HW_IF_POSITION, &right_arm_command_);
  return command_interfaces;
}

hardware_interface::return_type VaccumSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{ 
    RCLCPP_INFO(get_logger(), "Reading from hardware");
    rclcpp::spin_some(VaccumSystem::node_);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type VaccumSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    RCLCPP_INFO(get_logger(), "Writing to hardware");
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = (rear_left_wheel_command_ + rear_right_wheel_command_ + front_left_wheel_command_ + front_right_wheel_command_) / 4.0;
    cmd_msg.angular.z = ((-rear_left_wheel_command_ + rear_right_wheel_command_ - front_left_wheel_command_ + front_right_wheel_command_) / 4.0) / 0.18; // 0.18 is half the distance between wheels
    cmd_pub_->publish(cmd_msg);
  return hardware_interface::return_type::OK;
}

}  // namespace arduino_led_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  vaccum_control::VaccumSystem, hardware_interface::SystemInterface
)