#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <turtlesim/srv/spawn.hpp>
// To use the service we have to include it from the header file

namespace rj_training
{
class ServiceClientDemoNode : public rclcpp::Node
{
public:
  explicit ServiceClientDemoNode(const rclcpp::NodeOptions& options) : rclcpp::Node("service_client_demo_node", options)
  {
    client_ = create_client<turtlesim::srv::Spawn>("/spawn");
    // initialise the client variable with create_client function
    // with type of the service and name of the service
    std::thread(std::bind(&ServiceClientDemoNode::sendRequest, this)).detach();
    // As service need to wait for response
    // We will create a new thread to call the service
    // Create a thread and bind it to send request function
    // .datach() is used to automatically clear itself up after completion

  }

private:
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
  // To make this node act as service client we need to add a member variable


  void sendRequest()
  {
    RCLCPP_INFO(get_logger(), "Waiting for service...");
    if (!client_->wait_for_service(std::chrono::seconds(10)))
    // To check if the spawn service actually exists
    // Using wait_for_service with a timeout
    {
      RCLCPP_ERROR(get_logger(), "Couldn't find service");
      return; //Will return and the request will not be sent
    }
    RCLCPP_INFO(get_logger(), "Service ready!");
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    // Construct the request message
    request->x = 2.5;
    request->y = 2.5;
    // fill out the fields of our request
    auto result = client_->async_send_request(request);
    // async is an asynchronous function
    // Its return a value in future which will be stored in a variable called result
    RCLCPP_INFO(get_logger(), "Request sent");
    if (result.wait_for(std::chrono::seconds(10)) == std::future_status::timeout)
    // Waiting for the future to be ready with a timeout
    {
      RCLCPP_ERROR(get_logger(), "Service took too long to complete!");
      return;
    }
    RCLCPP_INFO(get_logger(), "Turtle spawned with name %s", result.get()->name.c_str());
    // If the service returns a response .get() is used to access the data
  }
};

}  // namespace rj_training

RCLCPP_COMPONENTS_REGISTER_NODE(rj_training::ServiceClientDemoNode)