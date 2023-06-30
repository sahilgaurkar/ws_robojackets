#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

namespace rj_training
{
class ServiceServerDemoNode : public rclcpp::Node
{
public:
  explicit ServiceServerDemoNode(const rclcpp::NodeOptions& option) : rclcpp::Node("service_server_demo_node", option)
  {
    service_ = create_service<example_interfaces::srv::AddTwoInts>(
        "/add_ints", std::bind(&ServiceServerDemoNode::Callback, this, std::placeholders::_1, std::placeholders::_2));
    // create_service is used to initilize the service members
    // template to pass in type of the service
    // give name and callback when service request is received
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
  // Make a service member variable
  void Callback(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
  {
    response->sum = request->a + request->b;
  }
  // ROS handles getting the response back to the client for us
};
}  // namespace rj_training

RCLCPP_COMPONENTS_REGISTER_NODE(rj_training::ServiceServerDemoNode)