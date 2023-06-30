#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/QuadWord.h>

namespace rj_training
{
class TfBroadcasterDemoNode : public rclcpp::Node
{
public:
  explicit TfBroadcasterDemoNode(const rclcpp::NodeOptions& options) 
  : rclcpp::Node("tf_broadcaster_demo_node", options), 
  tf_broadcaster_(*this)
  // Inatilize the tf_broadcaster_ member with the current node in constructor
  {
    timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TfBroadcasterDemoNode::TimerCallback, this)
    );
  }

private:
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    void TimerCallback()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "parent";
        transform.header.stamp = now();
        transform.child_frame_id = "child";
        transform.transform.translation.x = 1.0;
        transform.transform.translation.y = 2.0;
        transform.transform.translation.z = 3.0;
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, M_PI_2);
        transform.transform.rotation = tf2::toMsg(orientation);
        tf_broadcaster_.sendTransform(transform);
        // tf_broadcaster_ handles setting up topics and publishing messages for us
        // This is a static transform broadcaster which send the same fix transform at constant rate
        // Normally its dynamic from other data source
    }
};

}  // namespace rj_training

RCLCPP_COMPONENTS_REGISTER_NODE(rj_training::TfBroadcasterDemoNode)