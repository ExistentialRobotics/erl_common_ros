message(STATUS "ROS1 activated, building ROS1 stuff")

# node: transform_to_tf_node
add_executable(transform_to_tf_node src/ros1/transform_to_tf_node.cpp)
erl_target_dependencies(transform_to_tf_node) # automatically adds ROS dependencies
erl_collect_targets(EXECUTABLES transform_to_tf_node) # add to the list of executables to install

# node: clock_node
add_executable(clock_node src/ros1/clock_node.cpp)
erl_target_dependencies(clock_node)
erl_collect_targets(EXECUTABLES clock_node)
