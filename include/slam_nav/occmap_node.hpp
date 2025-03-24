#ifndef OCCUPANCYGRIDMAPNODE_H
#define OCCUPANCYGRIDMAPNODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <chrono>
#include <vector>

#include <Eigen/Dense>

#include "slam_nav/occmap.hpp"

class OccupancyGridMapNode : public rclcpp::Node  
{
public:
        OccupancyGridMapNode();
private:
        std::unique_ptr<OccupancyGridMap> occupancy_grid_map_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void map_publisher_callback();
};

#endif