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

hardware_interface::CallbackReturn VaccumSystem::on_init(
  const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  VaccumSystem::node_ = rclcpp::Node::make_shared("vaccum_system");
  VaccumSystem::enc_sub_ = VaccumSystem::node_->create_subscription<std_msgs::msg::Int64MultiArray>(
    "/encoder_counts", rclcpp::QoS(10),
    std::bind(&VaccumSystem::encoder_counts_callback, this, std::placeholders::_1));
  VaccumSystem::cmd_pub_ = VaccumSystem::node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));

  return hardware_interface::CallbackReturn::SUCCESS;
}

double last_left1 = 0;
double last_right1 = 0;
double last_left2 = 0;
double last_right2 = 0;

double laft_pos_left1 = 0;
double laft_pos_right1 = 0;
double laft_pos_left2 = 0;
double laft_pos_right2 = 0;

void VaccumSystem::encoder_counts_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 4) {
    double left1 = static_cast<double>(msg->data[0]);
    double right1 = static_cast<double>(msg->data[1]);
    double left2 = static_cast<double>(msg->data[2]);
    double right2 = static_cast<double>(msg->data[3]);
    int64_t dl1 = left1 - last_left1;
    int64_t dr1 = right1 - last_right1;
    int64_t dl2 = left2 - last_left2;
    int64_t dr2 = right2 - last_right2;
    last_left1 = left1;
    last_right1 = right1;
    last_left2 = left2;
    last_right2 = right2;
    
    float dt = (float)ENCODER_SAMPLE_MS / 1000.0f;
    
    float left_rps1 = (dl1 / TICKS_PER_REV) / dt;
    float right_rps1 = (dr1 / TICKS_PER_REV) / dt;
    float left_rps2 = (dl2 / TICKS_PER_REV) / dt;
    float right_rps2 = (dr2 / TICKS_PER_REV) / dt;
    float left_mps1 = left_rps1 * (2.0f * M_PI * WHEEL_RADIUS);
    float right_mps1 = right_rps1 * (2.0f * M_PI * WHEEL_RADIUS);
    float left_mps2 = left_rps2 * (2.0f * M_PI * WHEEL_RADIUS);
    float right_mps2 = right_rps2 * (2.0f * M_PI * WHEEL_RADIUS);

    rear_left_wheel_velocity_ = left_mps1;
    rear_right_wheel_velocity_ = right_mps1;
    front_left_wheel_velocity_ = left_mps2;
    front_right_wheel_velocity_ = right_mps2;

    laft_pos_left1 += (dl1 / TICKS_PER_REV) * (2.0f * M_PI * WHEEL_RADIUS);
    laft_pos_right1 += (dr1 / TICKS_PER_REV) * (2.0f * M_PI * WHEEL_RADIUS);
    laft_pos_left2 += (dl2 / TICKS_PER_REV) * (2.0f * M_PI * WHEEL_RADIUS);
    laft_pos_right2 += (dr2 / TICKS_PER_REV) * (2.0f * M_PI * WHEEL_RADIUS);

    rear_left_wheel_position_ = laft_pos_left1;
    rear_right_wheel_position_ = laft_pos_right1;
    front_left_wheel_position_ = laft_pos_left2;
    front_right_wheel_position_ = laft_pos_right2;

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
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VaccumSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back("rear_left_wheel_joint", hardware_interface::HW_IF_POSITION, &rear_left_wheel_position_);
  state_interfaces.emplace_back("rear_right_wheel_joint", hardware_interface::HW_IF_POSITION, &rear_right_wheel_position_);
  state_interfaces.emplace_back("front_left_wheel_joint", hardware_interface::HW_IF_POSITION, &front_left_wheel_position_);
  state_interfaces.emplace_back("front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &front_right_wheel_position_);
  state_interfaces.emplace_back("left_middle_arm_joint", hardware_interface::HW_IF_POSITION, &left_middle_arm_position_);
  state_interfaces.emplace_back("right_middle_arm_joint", hardware_interface::HW_IF_POSITION, &right_middle_arm_position_);
  state_interfaces.emplace_back("left_arm_joint", hardware_interface::HW_IF_POSITION, &left_arm_position_);
  state_interfaces.emplace_back("right_arm_joint", hardware_interface::HW_IF_POSITION, &right_arm_position_);

  state_interfaces.emplace_back("rear_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &rear_left_wheel_velocity_);
  state_interfaces.emplace_back("rear_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &rear_right_wheel_velocity_);
  state_interfaces.emplace_back("front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &front_left_wheel_velocity_);
  state_interfaces.emplace_back("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &front_right_wheel_velocity_);
  state_interfaces.emplace_back("left_middle_arm_joint", hardware_interface::HW_IF_VELOCITY, &left_middle_arm_velocity_);
  state_interfaces.emplace_back("right_middle_arm_joint", hardware_interface::HW_IF_VELOCITY, &right_middle_arm_velocity_);
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
  command_interfaces.emplace_back("left_middle_arm_joint", hardware_interface::HW_IF_VELOCITY, &left_middle_arm_command_);
  command_interfaces.emplace_back("right_middle_arm_joint", hardware_interface::HW_IF_VELOCITY, &right_middle_arm_command_);
  command_interfaces.emplace_back("left_arm_joint", hardware_interface::HW_IF_VELOCITY, &left_arm_command_);
  command_interfaces.emplace_back("right_arm_joint", hardware_interface::HW_IF_VELOCITY, &right_arm_command_);
  return command_interfaces;
}

hardware_interface::return_type VaccumSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{

  rclcpp::spin_some(node_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VaccumSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  

  return hardware_interface::return_type::OK;
}

}  // namespace arduino_led_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  vaccum_control::VaccumSystem, hardware_interface::SystemInterface
)