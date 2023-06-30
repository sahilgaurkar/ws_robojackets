#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>

namespace rj_training
{
class ActionServerDemoNode : public rclcpp::Node
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using FibonacciGoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
  // The above as just for convenience, shortcuts! that is type alias.


  explicit ActionServerDemoNode(const rclcpp::NodeOptions& options) 
  : rclcpp::Node("action_server_demo_node", options)
  {
    server_ = rclcpp_action::create_server<Fibonacci>(
      this, "/fibonacci",
      std::bind(&ActionServerDemoNode::NewGoalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ActionServerDemoNode::CancelCallback, this, std::placeholders::_1),
      std::bind(&ActionServerDemoNode::AcceptGoalCallback, this, std::placeholders::_1)
    );
    // Initilizing the server with rclcpp_action::create_server and type of our action
    // give it a handle to the node using 'this' and name of action '/fibonacci'
    // Next are the callback function that are bind to the member functions of this class


  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr server_;


  rclcpp_action::GoalResponse NewGoalCallback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "New goal (%s) with order %d", rclcpp_action::to_string(uuid).c_str(), goal->order);
    // Accept all goals
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  // NewGoalCallback job's is to take in a goal request and return an accept or reject code
  // the server_ object will handle sneding the acceptance or rejection back to client
  // In this case the node is going to accept and immediatly start executing all goals


  rclcpp_action::CancelResponse CancelCallback(
    const std::shared_ptr<FibonacciGoalHandle> goal_handle
  )
  {
    RCLCPP_INFO(get_logger(), "Cancelling goal %s",
    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
    // Accept all cancel requests
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  // Its job is to take in cancel request and either accept or reject those requestes
  // Here we are going to accept all cancellation request by returning the accept value

  void AcceptGoalCallback(const std::shared_ptr<FibonacciGoalHandle> goal_handle)
  {
    std::thread(
      [this, goal_handle]() {
        Execute(goal_handle);
      }).detach();
  }
  // Take a already accepted goal and start executing
  // Starts up a new thread using the execute function and detaches from that thread
  // Starting new thread so it will not block ROS event system
  // The node should stay responsive to other ROS events while executing this action
  // By keeping the execute funcion in detach thread it can run parallel to our ROS node

  void Execute(const std::shared_ptr<FibonacciGoalHandle> goal_handle)
  {
    RCLCPP_INFO(
      get_logger(), "Executing goal %s",
      rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
      const auto goal = goal_handle->get_goal();
      // getting goal message from goal_handle
      auto feedback = std::make_shared<Fibonacci::Feedback>();
      // setting up feedback message
      auto &sequence = feedback->sequence;
      sequence.push_back(0);
      sequence.push_back(1);
      // Initilizing the first 2 number of the sequence 0 and 1
      auto result = std::make_shared<Fibonacci::Result>();
      // setting up result message

      rclcpp::Rate rate(1 /*Hz*/);
      for (int i = 1; i < goal->order && rclcpp::ok(); ++i)
      {
        if(goal_handle->is_canceling())
        {
          result->sequence = sequence;
          // build result with whatever we go so far
          goal_handle->canceled(result);
          // return the result by calling cancel on goal handle
          RCLCPP_INFO(get_logger(), "Goal cancelled (%s)", rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
          return;
          //Log and return to finish execution function
        } // Check if the goal is canceled
        //
        sequence.push_back(sequence[i] + sequence[i-1]);
        goal_handle->publish_feedback(feedback);
        rate.sleep();
        // helper utility for creating rate limited loops using rate object defined earlier
      }//rclcpp::ok is used If the node gets shutdown during action this will finish the action

      if (rclcpp::ok()) // If the node is shutdown during execution it will not publish the results
      {
        result->sequence = sequence;
        goal_handle->succeed(result);
        // Report the completion of goal
        RCLCPP_INFO(
          get_logger(), "Goal succeeded (%s)",
          rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
      }
      // 3rd state is goal failed/aborted
      // 2nd Goal canceled
      // 1st Goal succeeded
  }
  // Its job is to take in the goal and execute the task
  // It will need to send any feedback messages and final result
  // It will also need to check if the goal has be cancelled and handle it
  // Here we are calculating Fibonacci numbers, which is structured atound the for loop. Everything else is just a boilerplate
};

}  // namespace rj_training

RCLCPP_COMPONENTS_REGISTER_NODE(rj_training::ActionServerDemoNode)