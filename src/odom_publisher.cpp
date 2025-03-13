#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class VOFramePublisher : public rclcpp::Node
{
public:
        VOFramePublisher()
        : Node("vo_frame_publisher")
        {
                tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

                auto publish_pose = 
                [this]() -> void {
                        geometry_msgs::msg::TransformStamped t;
                        
                        t.header.stamp = this->get_clock()->now();
                        t.header.frame_id = "odom";
                        t.child_frame_id = "base_link";

                        t.transform.translation.x = 1.0;
                        t.transform.translation.y = 2.0;
                        t.transform.translation.z = 3.0;
                        
                        tf2::Quaternion q;
                        q.setRPY(0, 0, 0);
                        t.transform.rotation.x = q.x();
                        t.transform.rotation.y = q.y();
                        t.transform.rotation.z = q.z();
                        t.transform.rotation.w = q.w();
                        
                        tf_broadcaster_->sendTransform(t);
                };

                timer_ = this->create_wall_timer(500ms, publish_pose);
        }
private:
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char * argv[]) 
{
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<VOFramePublisher>());
        rclcpp::shutdown();
        return 0;
}