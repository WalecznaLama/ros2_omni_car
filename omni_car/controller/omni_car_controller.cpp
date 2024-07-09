#include "omni_car/omni_car_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace omni_car {
RobotController::RobotController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RobotController::on_init() {
  // should have error handling TODO
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  wheel_radius_ = auto_declare<double>("wheel_radius", 0.0);
  wheel_base_ = auto_declare<double>("wheel_base", 0.0);
  wheel_track_ = auto_declare<double>("wheel_track", 0.0);

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : command_interface_types_) conf.names.push_back(joint_name + "/" + interface_type);
  }
  return conf;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : state_interface_types_) conf.names.push_back(joint_name + "/" + interface_type);
  }
  return conf;
}

controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &) {
  auto callback = [this](const std::shared_ptr<geometry_msgs::msg::Twist> cmd_vel_msg) -> void {
    cmd_vel_ptr_.writeFromNonRT(cmd_vel_msg);
    new_msg_ = true;
  };

  cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>("~/cmd_vel", rclcpp::SystemDefaultsQoS(), callback);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &) {
  // clear out vectors in case of restart
  joint_velocity_command_interface_.clear();
  joint_velocity_state_interface_.clear();

  // assign command interfaces
  for (auto & interface : command_interfaces_) command_interface_map_[interface.get_interface_name()]->push_back(interface);
  
  // assign state interfaces
  for (auto & interface : state_interfaces_) state_interface_map_[interface.get_interface_name()]->push_back(interface);
  
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
    if (new_msg_) {
        cmd_vel_msg_ = *cmd_vel_ptr_.readFromRT();
        start_time_ = time;
        new_msg_ = false;
    }

    if (cmd_vel_msg_ != nullptr) {
        double vx = cmd_vel_msg_->linear.x;
        double vy = cmd_vel_msg_->linear.y;
        double omega = cmd_vel_msg_->angular.z;

        // Calculate wheel velocities for mecanum drive
        double wheel_radius_inverse = 1.0 / wheel_radius_;
        double front_left_velocity  = (vx - vy - (wheel_base_ + wheel_track_) * omega) * wheel_radius_inverse;
        double front_right_velocity = (vx + vy + (wheel_base_ + wheel_track_) * omega) * wheel_radius_inverse;
        double rear_left_velocity   = (vx + vy - (wheel_base_ + wheel_track_) * omega) * wheel_radius_inverse;
        double rear_right_velocity  = (vx - vy + (wheel_base_ + wheel_track_) * omega) * wheel_radius_inverse;

        // Joints are ordered as [front_left, front_right, rear_left, rear_right]
        joint_velocity_command_interface_[0].get().set_value(front_left_velocity);
        joint_velocity_command_interface_[1].get().set_value(front_right_velocity);
        joint_velocity_command_interface_[2].get().set_value(rear_left_velocity);
        joint_velocity_command_interface_[3].get().set_value(rear_right_velocity);
    }

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &) {
  release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_cleanup(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }

controller_interface::CallbackReturn RobotController::on_error(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }

controller_interface::CallbackReturn RobotController::on_shutdown(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }

}  // namespace omni_car

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(omni_car::RobotController, controller_interface::ControllerInterface)