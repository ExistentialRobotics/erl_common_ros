message(STATUS "ROS2 activated, building ROS2 stuff")

# node: clock_node
add_executable(clock_node src/ros2/clock_node.cpp)
erl_target_dependencies(clock_node)
erl_collect_targets(EXECUTABLES clock_node)
