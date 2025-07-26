cmake_minimum_required(VERSION 3.16)

project(erl_common_ros #
        LANGUAGES CXX #
        VERSION 0.1.0 #
        DESCRIPTION "erl_common_ros is a ROS package that provides ROS nodes and utilities")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if (NOT COMMAND erl_project_setup)
    find_package(erl_cmake_tools REQUIRED)
endif ()
erl_project_setup()
erl_setup_ros(
    CATKIN_COMPONENTS roscpp rosgraph_msgs geometry_msgs tf2_ros #
    CATKIN_DEPENDS roscpp rosgraph_msgs geometry_msgs tf2_ros #
    ROS2_COMPONENTS rclcpp rosgraph_msgs geometry_msgs tf2_ros)
erl_add_ros_src()
erl_install(EXECUTABLES ${${PROJECT_NAME}_COLLECTED_EXECUTABLES})
erl_mark_project_found()
