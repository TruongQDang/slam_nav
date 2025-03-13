#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <iostream>

using namespace std;

int main(int argc, char ** argv) {
        rclcpp::init(argc, argv);
        rclcpp::NodeOptions options;
        rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
        image_transport::ImageTransport it(node);
        image_transport::Publisher pub = it.advertise("camera/image", 1);

        char imageset_path[500];
        string pre_path = "/home/truongdang/Documents/visual_slam/kitti_dataset/data_odometry_gray/dataset/sequences/00/image_0/";
        int num_frame = 0;

        rclcpp::WallRate loop_rate(30);
        while (rclcpp::ok()) {
                if (num_frame == 2000) {
                        break;
                }
                sprintf(imageset_path, (pre_path + "%06d.png").c_str(), num_frame++);
                cout << "Imageset path: " << imageset_path << endl;
                
                cv::Mat image_c = cv::imread(imageset_path);
                cv::Mat image;
                cv::cvtColor(image_c, image, cv::COLOR_BGR2GRAY);
                std_msgs::msg::Header hdr;
                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(hdr, "mono8", image).toImageMsg();

                pub.publish(msg);
                rclcpp::spin_some(node);
                loop_rate.sleep();
        }
        rclcpp::shutdown();
        return 0;
}