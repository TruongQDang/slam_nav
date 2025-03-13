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

add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(
        image_publisher 
        rclcpp 
        sensor_msgs 
        cv_bridge 
        image_transport 
        OpenCV)
install(TARGETS image_publisher DESTINATION lib/${PROJECT_NAME})

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

add_executable(odom_publisher src/odom_publisher.cpp)
ament_target_dependencies(
        odom_publisher
        geometry_msgs
        rclcpp
        tf2
        tf2_ros)

install(TARGETS odom_publisher
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch description worlds config
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
