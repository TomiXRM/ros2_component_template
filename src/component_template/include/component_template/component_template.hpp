#ifndef __COMPONENT_TEMPLATE_HPP__
#define __COMPONENT_TEMPLATE_HPP__

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int32.hpp"

#include <cstdint>

namespace component_template {
class ComponentTemplate : public rclcpp::Node {
 public:
  explicit ComponentTemplate(const rclcpp::NodeOptions &node_options);

 private:
  void on_timer();
  void on_message(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr    publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr                          timer_;

  int32_t     counter_{0};
  std::string topic_name_{"example_int"};
};
}  // namespace component_template

#endif  // __COMPONENT_TEMPLATE_HPP__
