#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

namespace rj_training
// Namespace should match the package name
{
class ParametersDemoNode : public rclcpp::Node
{
public:
  explicit ParametersDemoNode(const rclcpp::NodeOptions& options) : rclcpp::Node("parameters_demo_node", options)
  {
    declare_parameter<std::string>("message_data", "Hello!");
    // Calling a function declare parameter to declare a parameter named "message_data" and an initial value "Hello!"
    publisher_ = create_publisher<std_msgs::msg::String>("/demo_topic", rclcpp::SystemDefaultsQoS());
    const auto timer_duration = declare_parameter<int>("timer_duration", 1);
    // declare_parameter also returns the value of parameter
    timer_ = create_wall_timer(std::chrono::seconds(timer_duration), std::bind(&ParametersDemoNode::TimerCallback, this));
    // This timer run every "timer_duration" value second and is calling the "TimerCallback" function on "this" object
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void TimerCallback()
  {
    std_msgs::msg::String msg;
    msg.data = get_parameter("message_data").as_string();
    // get_parameter function is used to get the parameter data
    // Using .as_string because the return type of get_parameter is varient as it can hold value of different types
    publisher_->publish(msg);
  }
};

}  // namespace rj_training

RCLCPP_COMPONENTS_REGISTER_NODE(rj_training::ParametersDemoNode)