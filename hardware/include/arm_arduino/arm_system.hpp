// Copyright 2021 ros2_control Development Team
//
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

#ifndef ARM_ARDUINO__DIFFBOT_SYSTEM_HPP_
#define ARM_ARDUINO__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "arm_arduino/visibility_control.h"

#include "arm_arduino/arduino_comms_network.hpp"

namespace arm_arduino
{
class ArmArduinoHardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::string ip_addr;
  int32_t network_port_number;
};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmArduinoHardware);

  ARM_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ARM_ARDUINO_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ARM_ARDUINO_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ARM_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ARM_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  ARM_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ARM_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ARM_ARDUINO_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ARM_ARDUINO_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;


private:
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }
  rclcpp::Clock::SharedPtr clock_;
  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_prev_positions_;
  std::vector<double> hw_velocities_;

  ArduinoComms comms_;
  Config cfg_;
};

}  // namespace arm_arduino

#endif  // ARM_ARDUINO__DIFFBOT_SYSTEM_HPP_
