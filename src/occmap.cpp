#include "slam_nav/occmap.hpp"
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"


OccupancyGridMap::OccupancyGridMap(unsigned int grid_size, double cell_size)
    : grid_size_{grid_size}, cell_size_{cell_size}
{
        num_cells_ = grid_size_ / cell_size_;

        int x_origin = (-static_cast<int>(num_cells_) / 2 * cell_size_);
        int y_origin = (-static_cast<int>(num_cells_) / 2 * cell_size_);
        grid_origin_ << x_origin, y_origin;

        grid_map_.resize(num_cells_, num_cells_);
        grid_map_.setConstant(p_prior_);
}

void OccupancyGridMap::generate_map_msg(
        nav_msgs::msg::OccupancyGrid &occupancy_grid_msg)
{
        occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
        occupancy_grid_msg.info.width = num_cells_;
        occupancy_grid_msg.info.height = num_cells_;
        occupancy_grid_msg.info.resolution = cell_size_;
        occupancy_grid_msg.info.origin.position.x = grid_origin_.x();
        occupancy_grid_msg.info.origin.position.y = grid_origin_.y();
        occupancy_grid_msg.header.frame_id = "map";

        occupancy_grid_msg.data.resize(num_cells_ * num_cells_);
        for (unsigned int row = 0; row < num_cells_; row++)
        {
                for (unsigned int col = 0; col < num_cells_; col++)
                {
                        int index = row * num_cells_ + col; // row-major index
                        occupancy_grid_msg.data[index] = grid_map_(row,col);
                }
        }
}

void OccupancyGridMap::convert_gridmap_prob2logodds()
{
        for (int i = 0; i < grid_map_.size(); i++)
                grid_map_(i) = log(grid_map_(i) / (1 - grid_map_(i)));
}

void OccupancyGridMap::convert_gridmap_prob2occupancy()
{
        for (int i = 0; i < grid_map_.size(); i++)
        {
                double prob = grid_map_(i);
                if (prob == 0.5)
                {
                        grid_map_(i) = -1; // unknown cell
                        continue;
                }
                grid_map_(i) = std::round(prob) * 100;
        }
}

void OccupancyGridMap::convert_gridmap_logodds2prob()
{
        for (int i = 0; i < grid_map_.size(); i++)
                grid_map_(i) = 1 - (1 / (1 + exp(grid_map_(i))));
        std::cout << grid_map_ << std::endl;
}

double OccupancyGridMap::convert_cell_prob2logodds(double cell_prob)
{
        return log(cell_prob / (1 - cell_prob));
}
void OccupancyGridMap::update_cells(
        Eigen::Vector2i &p_grid_pose,
        Eigen::Vector2i &p_grid_point)
{
        std::vector<Eigen::Vector2i> cells_to_update = bresenham_line(p_grid_pose, p_grid_point);

        // update free points
        for (unsigned int i = 0; i < (cells_to_update.size()-1); i++) {
                int x_cell = cells_to_update[i].x();
                int y_cell = cells_to_update[i].y();
                grid_map_(x_cell, y_cell) = 
                        convert_cell_prob2logodds(p_free_) + 
                        grid_map_(x_cell, y_cell) +
                        - convert_cell_prob2logodds(p_prior_); 
        }

        // update endpoint as occupied
        int x_cell = cells_to_update[cells_to_update.size()-1].x();
        int y_cell = cells_to_update[cells_to_update.size()-1].y();
        grid_map_(x_cell, y_cell) =
                convert_cell_prob2logodds(p_occ_) +
                grid_map_(x_cell, y_cell) +
                -convert_cell_prob2logodds(p_prior_);
}

std::vector<Eigen::Vector2i> OccupancyGridMap::bresenham_line(
    Eigen::Vector2i start_cell, Eigen::Vector2i end_cell)
{
        std::vector<Eigen::Vector2i> points;

        int dx = end_cell.x() - start_cell.x();
        int dy = end_cell.y() - start_cell.y();

        int xsign = (dx > 0) ? 1 : -1;
        int ysign = (dy > 0) ? 1 : -1;

        dx = abs(dx);
        dy = abs(dy);

        int xx, xy, yx, yy;
        if (dx > dy) {
                xx = xsign;
                xy = 0;
                yx = 0;
                yy = ysign;
        } else {
                std::swap(dx, dy);
                xx = 0;
                xy = ysign;
                yx = xsign;
                yy = 0;
        }

        int D = 2 * dy - dx;
        int y = 0;

        for (int x = 0; x <= dx; x++) {
                points.push_back(Eigen::Vector2i(start_cell.x() + x * xx + y * yx, start_cell.y() + x * xy + y * yy));
                if (D >= 0) {
                        y++;
                        D -= 2 * dx;
                }
                D += 2 * dy;
        }

        return points;
}

void OccupancyGridMap::update_map(
        const std::vector<Eigen::Matrix3d> &T_w_r_array, 
        const std::vector<sensor_msgs::msg::LaserScan> &laser_scan_array)
{
        convert_gridmap_prob2logodds();

        // grid coordinate of robot
        std::vector<Eigen::Vector2i> p_grid_pose_array = transform_pose2grid(T_w_r_array);  

        // iterate over all poses + laser scans
        for (unsigned int i = 0; i < p_grid_pose_array.size(); i++) {
                std::vector<Eigen::Vector2i> p_grid_points = transform_scan2grid(laser_scan_array[i], T_w_r_array[i]);
                // iterate over all endpoints of a laser scan and update cells on line of sight
                for (unsigned int j = 0; j < p_grid_points.size(); j++)
                        update_cells(p_grid_pose_array[i], p_grid_points[j]);
        }

        convert_gridmap_logodds2prob();
        convert_gridmap_prob2occupancy();
}

std::vector<Eigen::Vector3d> OccupancyGridMap::transform_scan2robot(const sensor_msgs::msg::LaserScan &laser_scan)
{
        std::vector<Eigen::Vector3d> scan_cartesian;
        scan_cartesian.reserve(laser_scan.ranges.size());
        float angle = laser_scan.angle_min;

        for (float range : laser_scan.ranges)
        {
                if (std::isnan(range) || std::isinf(range) || range <= laser_scan.range_min || range > laser_scan.range_max)
                {
                        angle += laser_scan.angle_increment;
                        continue;
                }
                // homogeneous coordinate
                Eigen::Vector3d cartesian_point(range * cos(angle), range * sin(angle), 1);
                scan_cartesian.push_back(cartesian_point);
                angle += laser_scan.angle_increment;
        }

        return scan_cartesian;
}

std::vector<Eigen::Vector3d> OccupancyGridMap::transform_robot2world(
        const std::vector<Eigen::Vector3d> &p_r_points, const 
        Eigen::Matrix3d &T_w_r)
{
        std::vector<Eigen::Vector3d> p_w_points;
        p_w_points.reserve(p_r_points.size());
        for (auto p_r_point : p_r_points)
        {
                Eigen::Vector3d p_w_point = T_w_r * p_r_point;
                p_w_points.push_back(p_w_point);
        }

        return p_w_points;
}

std::vector<Eigen::Vector2i> OccupancyGridMap::transform_world2grid(
        const std::vector<Eigen::Vector3d> &p_w_points)
{
        std::vector<Eigen::Vector2i> p_grid_points;
        p_grid_points.reserve(p_w_points.size());
        for (auto p_w_point : p_w_points) {
                int x_grid_point = (p_w_point.x() - grid_origin_.x()) / cell_size_;
                int y_grid_point = (p_w_point.y() - grid_origin_.y()) / cell_size_;
                Eigen::Vector2i p_grid_point(x_grid_point, y_grid_point);
                p_grid_points.push_back(p_grid_point);
        }

        return p_grid_points;
}

std::vector<Eigen::Vector2i> OccupancyGridMap::transform_scan2grid(
        const sensor_msgs::msg::LaserScan &laser_scan, 
        const Eigen::Matrix3d &T_w_r)
{
        std::vector<Eigen::Vector3d> p_r_points = transform_scan2robot(laser_scan);
        std::vector<Eigen::Vector3d> p_w_points = transform_robot2world(p_r_points, T_w_r);
        std::vector<Eigen::Vector2i> p_grid_points = transform_world2grid(p_w_points);
        return p_grid_points;
}

std::vector<Eigen::Vector2i> OccupancyGridMap::transform_pose2grid(const std::vector<Eigen::Matrix3d> &pose_array)
{
        std::vector<Eigen::Vector2i> p_grid_pose_array;
        p_grid_pose_array.reserve(pose_array.size());
        for (auto pose : pose_array) {
                // extract position of robot pose
                Eigen::Vector3d p_pose = pose.col(2);
                int x_grid_pose = (p_pose.x() - grid_origin_.x()) / cell_size_;
                int y_grid_pose = (p_pose.y() - grid_origin_.y()) / cell_size_;
                Eigen::Vector2i p_grid_pose(x_grid_pose, y_grid_pose);
                p_grid_pose_array.push_back(p_grid_pose);
        }

        return p_grid_pose_array;
}
