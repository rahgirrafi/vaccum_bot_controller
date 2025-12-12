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
  if (msg->data.size() >= 4) {

    float left_mps1 = static_cast<double>(msg->data[0]);
    float right_mps1 = static_cast<double>(msg->data[1]);
    float left_mps2 = static_cast<double>(msg->data[2]);
    float right_mps2 = static_cast<double>(msg->data[3]);

    rear_left_wheel_velocity_ = left_mps1;
    rear_right_wheel_velocity_ = right_mps1;
    front_left_wheel_velocity_ = left_mps2;
    front_right_wheel_velocity_ = right_mps2;
    float dt = (float)ENCODER_SAMPLE_MS / 1000.0f;
    
    left_pos_left1 +=  left_mps1 * dt / WHEEL_RADIUS;
    left_pos_right1 += right_mps1 *dt  / WHEEL_RADIUS;
    left_pos_left2 +=  left_mps2 *dt / WHEEL_RADIUS;
    left_pos_right2 += right_mps2 *dt /  WHEEL_RADIUS;

    rear_left_wheel_position_ = left_pos_left1;
    rear_right_wheel_position_ = left_pos_right1;
    front_left_wheel_position_ = left_pos_left2;
    front_right_wheel_position_ = left_pos_right2;

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