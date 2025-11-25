// Copyright 2025 Daiki Tomimoka
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

#ifndef __ADDRESS_VALUE_HPP__
#define __ADDRESS_VALUE_HPP__

#include "rclcpp/rclcpp.hpp"

// #include "std_msgs/msg/int32.hpp"
#include "address_value_msg/msg/address_value.hpp"

#include <cstdint>

namespace address_value {
class AddressValue : public rclcpp::Node {
 public:
  explicit AddressValue(const rclcpp::NodeOptions &node_options);

 private:
  void on_timer();
  void on_message(const address_value_msg::msg::AddressValue::SharedPtr msg);

  rclcpp::Publisher<address_value_msg::msg::AddressValue>::SharedPtr    publisher_;
  rclcpp::Subscription<address_value_msg::msg::AddressValue>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr                                          timer_;

  int32_t address_{0};
  int32_t value_{0};
};
}  // namespace address_value

#endif  // __ADDRESS_VALUE_HPP__