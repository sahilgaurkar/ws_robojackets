#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>

namespace rj_training
{
class ActionClientDemoNode : public rclcpp::Node
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using FibonacciGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;
  // The above as just for convenience, shortcuts! that is type alias.

  explicit ActionClientDemoNode(const rclcpp::NodeOptions& options) : rclcpp::Node("action_client_demo_node", options)
  {
    client_ = rclcpp_action::create_client<Fibonacci>(this, "/fibonacci");
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&ActionClientDemoNode::TimerCallback, this));
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void TimerCallback()
  {
    timer_->cancel();  // one-shot timer
    // That is it will wait for the duration and trigger callback exactly once
    // This will cancle the timer used to wait for 1 sec before calling the TimerCallback

    rclcpp_action::Client<Fibonacci>::SendGoalOptions goal_options;
    //auto goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    goal_options.goal_response_callback =
        std::bind(&ActionClientDemoNode::GoalResponseCallback, this, std::placeholders::_1);
    // Lets us know if the server accepted or rejected the goal
    goal_options.feedback_callback =
        std::bind(&ActionClientDemoNode::FeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    // This is called when we have feedback from the server
    goal_options.result_callback = 
        std::bind(&ActionClientDemoNode::ResultCallback, this, std::placeholders::_1);
    // This gets called when we have the final result for our goal

    // This Section initilizes the goal_options object
    // Just defines different Callbacks that we will be intrested in for different status updates

    Fibonacci::Goal goal;
    goal.order = 4;

    client_->async_send_goal(goal, goal_options);
  }

  void GoalResponseCallback(const FibonacciGoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "Action server rejected goal!");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Action server accepted goal (%s)",
                  rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
    }
  }
  // We get a parameter future as goal_handle
  // Wait for that future to be ready

  void FeedbackCallback(FibonacciGoalHandle::SharedPtr goal_handle,
                        const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Feedback for goal (%s): %ld numbers available",
                rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(), feedback->sequence.size());
  }
  //Takes a goal handle pointer and a feedback message

  void ResultCallback(const FibonacciGoalHandle::WrappedResult &result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal (%s) succedded! final Fibonacci number: %d", rclcpp_action::to_string(result.goal_id).c_str(), result.result->sequence.back());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal (%s) aborted!", rclcpp_action::to_string(result.goal_id).c_str());
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal (%s) canceled!", rclcpp_action::to_string(result.goal_id).c_str());
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknone result code for goal (%s)", rclcpp_action::to_string(result.goal_id).c_str());
        break;
    }
    rclcpp::shutdown();
  }
  // Takes in wrapped result - result message from action wrapped with metadata such as status of goal and its ID.
  // 
};

}  // namespace rj_training

RCLCPP_COMPONENTS_REGISTER_NODE(rj_training::ActionClientDemoNode)