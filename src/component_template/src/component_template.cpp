#include "component_template/component_template.hpp"

#include <chrono>
#include <functional>

namespace component_template {

ComponentTemplate::ComponentTemplate(const rclcpp::NodeOptions &node_options) : Node("component_template", node_options) {
  // Declare parameters
  counter_    = this->declare_parameter<int32_t>("counter", 0);
  topic_name_ = this->declare_parameter<std::string>("topic_name", "example_int");

  // Create publisher and subscription
  publisher_    = this->create_publisher<std_msgs::msg::Int32>(topic_name_, 10);
  subscription_ = this->create_subscription<std_msgs::msg::Int32>(topic_name_, 10, std::bind(&ComponentTemplate::on_message, this, std::placeholders::_1));

  // Create timer
  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(1s, std::bind(&ComponentTemplate::on_timer, this));

  RCLCPP_INFO(this->get_logger(), "component_template node has been started.");
}

void ComponentTemplate::on_timer() {
  std_msgs::msg::Int32 msg;
  msg.data = counter_++;
  RCLCPP_INFO(this->get_logger(), "Publishing int: %d", msg.data);
  publisher_->publish(msg);
}

void ComponentTemplate::on_message(const std_msgs::msg::Int32::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received int: %d", msg->data);
}

}  // namespace component_template

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(component_template::ComponentTemplate)
