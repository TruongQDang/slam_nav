#ifndef OCCUPANCYGRIDMAP_H
#define OCCUPANCYGRIDMAP_H

#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

template <typename T>
struct Point2d
{
        T x;
        T y;
};

enum class CellState : unsigned int
{
        FREE,
        OCCUPIED
};

class OccupancyGridMap {
public:
        OccupancyGridMap(unsigned int map_size, double cell_size);

        void update_map(
                const std::vector<Eigen::Matrix3d> &T_w_r_array,
                const std::vector <sensor_msgs::msg::LaserScan> &laser_scan_array);

        void generate_map_msg(
            nav_msgs::msg::OccupancyGrid &occupancy_grid_msg);

private:
        Eigen::MatrixXd grid_map_;
        unsigned int grid_size_;
        double cell_size_;
        unsigned int num_cells_;
        Eigen::Vector2i grid_origin_;
        const double p_free_{0.35};
        const double p_occ_{0.9};
        const double p_prior_{0.5};

        std::vector<Eigen::Vector3d> transform_scan2robot(
                const sensor_msgs::msg::LaserScan &laser_scan);

        std::vector<Eigen::Vector3d> transform_robot2world(
                const std::vector<Eigen::Vector3d> &p_r_points,
                const Eigen::Matrix3d &T_w_r);

        std::vector<Eigen::Vector2i> transform_world2grid(
                const std::vector<Eigen::Vector3d> &p_w_points);

        std::vector<Eigen::Vector2i> transform_scan2grid(
                const sensor_msgs::msg::LaserScan &laser_scan,
                const Eigen::Matrix3d &T_w_r);

        std::vector<Eigen::Vector2i> transform_pose2grid(
                const std::vector<Eigen::Matrix3d> &T_w_r_array);

        void convert_gridmap_prob2logodds();
        void convert_gridmap_logodds2prob();
        void convert_gridmap_prob2occupancy();

        double convert_cell_prob2logodds(double cell_prob);

        void update_cells(Eigen::Vector2i &p_grid_pose, Eigen::Vector2i &p_grid_point);

        std::vector<Eigen::Vector2i> bresenham_line(Eigen::Vector2i start_cell, Eigen::Vector2i end_cell);

};

#endif