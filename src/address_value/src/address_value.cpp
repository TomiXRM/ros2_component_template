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

#include "address_value/address_value.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <chrono>
#include <functional>

namespace address_value {

AddressValue::AddressValue(const rclcpp::NodeOptions &node_options) : Node("address_value", node_options) {
  address_ = this->declare_parameter<int32_t>("address", 0);
  value_   = this->declare_parameter<int32_t>("value", 0);

  publisher_ = this->create_publisher<address_value_msg::msg::AddressValue>("example_int", 10);

  subscription_ = this->create_subscription<address_value_msg::msg::AddressValue>("example_int", 10, std::bind(&AddressValue::on_message, this, std::placeholders::_1));

  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(1s, std::bind(&AddressValue::on_timer, this));

  RCLCPP_INFO(this->get_logger(), "address_value node has been started.");
}

void AddressValue::on_timer() {
  address_value_msg::msg::AddressValue msg;
  msg.address = address_;
  msg.value   = value_++;
  RCLCPP_INFO(this->get_logger(), "Publishing address: %d value:%d", msg.address, msg.value);
  publisher_->publish(msg);
}

void AddressValue::on_message(const address_value_msg::msg::AddressValue::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received address: %d value: %d", msg->address, msg->value);
}

}  // namespace address_value

RCLCPP_COMPONENTS_REGISTER_NODE(address_value::AddressValue)
