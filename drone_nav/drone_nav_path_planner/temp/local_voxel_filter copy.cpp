#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

class TFListenerNode : public rclcpp::Node
{
public:
    TFListenerNode()
    : Node("tf_listener_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        timer_ = this->create_wall_timer(500ms, std::bind(&TFListenerNode::on_timer, this));
    }

private:
    void on_timer()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped = 
                tf_buffer_.lookupTransform("target_frame", "source_frame", tf2::TimePointZero);
            
            RCLCPP_INFO(this->get_logger(), 
                "Transform: Translation (%.2f, %.2f, %.2f)", 
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFListenerNode>());
    rclcpp::shutdown();
    return 0;
}
