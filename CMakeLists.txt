cmake_minimum_required(VERSION 3.8)
project(slam_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
        launch description worlds config
        DESTINATION share/${PROJECT_NAME}/)

# install(TARGETS
#         talker
#         DESTINATION lib/${PROJECT_NAME})

ament_package()
