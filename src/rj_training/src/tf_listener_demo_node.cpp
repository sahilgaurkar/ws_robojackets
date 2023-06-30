#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace rj_training
{
class TfListenerDemoNode : public rclcpp::Node
{
public:
  explicit TfListenerDemoNode(const rclcpp::NodeOptions& options) 
  : rclcpp::Node("tf_listner_demo_node", options), 
  tf_buffer_(get_clock()), //To initilize the buffer
  tf_listener_(tf_buffer_) //To initilize the listener
  {
    timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TfListenerDemoNode::TimerCallback, this)
    );
  }

private:
    tf2_ros::Buffer tf_buffer_;
    // tf_buffer is the object that actually tracks the tf tree
    // keeps tracks of what frames are available, what transformas are available and how it changes with time
    // It will maintain some short history of the tree and that becomes the range of time we will be able to query it
    tf2_ros::TransformListener tf_listener_;
    // tf_listner_ is an object that starts its own thread and runs in the background
    // this is the object that subscribes the ros topics and updating the buffer with messages that comes across those topics

    rclcpp::TimerBase::SharedPtr timer_;

    void TimerCallback()
    {
        if(!tf_buffer_.canTransform("child", "parent", tf2::TimePointZero))
        // Check if we can actually lookup the transform that we are intrested in
        // Here we are looking for "parent" and "child" relationship
        // .canTransform is used to check if all the parts of the tree is available to connect parent and child
        // In addition to source and target frame we also need to give it a time point.
        // In this case the time point is 0, that means we want the latest info available
        // This is an IMPORTANT step to avoid exceptions
        {
            RCLCPP_WARN_ONCE(get_logger(), "Waiting for TF data...");
            return;
        } // Will run when the relationship does not exists
        // With this check the node will wait until the TF data is available
        PrintTransform();
        TransformPoint();
        // These are the 2 ways to use the transform data
    }

    void PrintTransform()
    // Look up the transform offsets between our two frames
    // print those out to the console
    {
        const auto &transform = tf_buffer_.lookupTransform("child", "parent", tf2::TimePointZero);
        std::stringstream output;
        output << "\nTransform parent --> child:\n";
        output << "\tTranslation:\n";
        output << "\t\tX = " << transform.transform.translation.x << "\n";
        output << "\t\tY = " << transform.transform.translation.y << "\n";
        output << "\t\tZ = " << transform.transform.translation.z << "\n";
        output << "\tRotation:\n";
        tf2::Quaternion rotation;
        tf2::fromMsg(transform.transform.rotation, rotation);
        tf2::Matrix3x3 rotation_matrix(rotation);
        double roll;
        double pitch;
        double yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);
        output << "\t\tR = " << roll << "\n";
        output << "\t\tP = " << pitch << "\n";
        output << "\t\tY = " << yaw << "\n";
        RCLCPP_INFO(get_logger(), "%s", output.str().c_str());
    }

    void TransformPoint()
    // Used to convert a point wrt one frame to another
    {
        geometry_msgs::msg::PointStamped point;
        point.header.frame_id = "child";
        point.point.x = 1.0;
        point.point.y = 0.0;
        point.point.z = 0.0;
        const auto transformed_point = tf_buffer_.transform(point, "parent");
        // transform fuction takes in the point we want to convert and the target frame we want to convert it to.
        // As we are using stamped point, it already has the source frame for the transform function
        RCLCPP_INFO(
            get_logger(), "Transformed point: (%f, %f, %f)", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
    }

};

}  // namespace rj_training

RCLCPP_COMPONENTS_REGISTER_NODE(rj_training::TfListenerDemoNode)