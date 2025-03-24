cmake_minimum_required(VERSION 3.8)
project(slam_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(occupancy_grid_map src/occmap_node.cpp
                                src/occmap.cpp)
ament_target_dependencies(
        occupancy_grid_map
        rclcpp
        nav_msgs
        sensor_msgs
        geometry_msgs)
install(TARGETS occupancy_grid_map 
        DESTINATION lib/${PROJECT_NAME}) 
target_link_libraries(occupancy_grid_map Eigen3::Eigen)      

add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(
        image_publisher 
        rclcpp 
        sensor_msgs 
        cv_bridge 
        image_transport 
        OpenCV)
install(TARGETS image_publisher 
        DESTINATION lib/${PROJECT_NAME})

add_executable(vo_node src/vo_node.cpp)
ament_target_dependencies(
        vo_node 
        rclcpp 
        sensor_msgs 
        cv_bridge 
        image_transport 
        OpenCV 
        tf2 
        tf2_ros 
        geometry_msgs)
install(TARGETS vo_node 
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch description worlds config
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
