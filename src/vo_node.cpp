#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include "frame_handler.hpp"

class ImageProcessorNode : public rclcpp::Node {
public:
    ImageProcessorNode() : Node("image_processor") {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10, 
            std::bind(&ImageProcessorNode::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat image = cv_bridge::toCvCopy(msg, "mono8")->image;
            auto [R,t] = visual_odometry_.run(image); 
            std::cout << "Rotation Matrix (R):\n" << R << std::endl;
            std::cout << "Translation Vector (t):\n" << t << std::endl; 
            // Convert rotation matrix to quaternion
            tf2::Matrix3x3 tf2_R(
                R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2)
            );
            tf2::Quaternion q;
            tf2_R.getRotation(q);
            // Create TransformStamped message
            geometry_msgs::msg::TransformStamped transform_msg;
            transform_msg.header.stamp = this->get_clock()->now();
            transform_msg.header.frame_id = "odom";  // Parent frame
            transform_msg.child_frame_id = "base_link";  // Child frame

            transform_msg.transform.translation.x = t.at<double>(2, 0); // z
            transform_msg.transform.translation.y = t.at<double>(0, 0); // x
            transform_msg.transform.translation.z = 0;
            transform_msg.transform.rotation.x = q.x();
            transform_msg.transform.rotation.y = q.y();
            transform_msg.transform.rotation.z = q.z();
            transform_msg.transform.rotation.w = q.w();

            // Broadcast transform
            tf_broadcaster_->sendTransform(transform_msg);

        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge Error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    VisualOdometry visual_odometry_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
