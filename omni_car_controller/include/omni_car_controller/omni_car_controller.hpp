#ifndef OMNI_CAR_CONTROLLER_HPP_
#define OMNI_CAR_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.h"

namespace omni_car {
class RobotController : public controller_interface::ControllerInterface {
public:
  CONTROLLER_INTERFACE_PUBLIC
  RobotController();

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_configure( const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  bool get_parameters();

  double wheel_radius_, wheel_base_, wheel_track_;  // parameters
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> cmd_vel_ptr_;
  bool new_msg_ = false;
  rclcpp::Time start_time_;
  std::shared_ptr<geometry_msgs::msg::Twist> cmd_vel_msg_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_velocity_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;

  std::unordered_map< std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
    command_interface_map_ = { {"velocity", &joint_velocity_command_interface_} };

  std::unordered_map< std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
    state_interface_map_ = { {"velocity", &joint_velocity_state_interface_} };
};

}  // namespace omni_car

#endif  // OMNI_CAR_CONTROLLER_HPP_