#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
// Included String msg defination

namespace rj_training
{
class TopicsDemoNode : public rclcpp::Node
{
public:
  explicit TopicsDemoNode(const rclcpp::NodeOptions& options) : rclcpp::Node("topics_demo_node", options)
  {
    publisher_ = create_publisher<std_msgs::msg::String>("/greeting", rclcpp::SystemDefaultsQoS());
    // create_publisher method takes in name of the topic and a default Quality of service settings
    subscription_ = create_subscription<std_msgs::msg::String>(
        "/name",
        // name of the topic
        rclcpp::SystemDefaultsQoS(),
        // QoS setting we want to use
        std::bind(&TopicsDemoNode::Callback, this, std::placeholders::_1)
        // A callback we want to use when ever a new message is received
        // Using std::bind to pass it a member function
        // in this case we can to call a function names callback
        // "this" means on the current object
        // placeholder will let it pass through the message parameter

        );
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // Added mumber variable publisher to our class
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void Callback(const std_msgs::msg::String::SharedPtr msg)
  // Defination of the call back when a new message is received
  {
    std_msgs::msg::String greeting_msg;
    // A new message object "greeting_msg"
    greeting_msg.data = "Hello, " + msg->data + "!";
    // populating the data field using the datafield of incomming message
    publisher_->publish(greeting_msg);
    // Publishing a new message using publisher
  } // Logic to take the name message add greeting and publish it
};

}  // namespace rj_training

RCLCPP_COMPONENTS_REGISTER_NODE(rj_training::TopicsDemoNode)