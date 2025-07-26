erl_common_ros
==============

Common ROS 1/2 utilities for ERL projects.

# Requirements

- ROS 1 Noetic or ROS 2 Humble or later
- Ubuntu 20.04 for ROS1
- Ubuntu 22.04 or later for ROS2
- dependencies:
    - [erl_cmake_tools](https://github.com/ExistentialRobotics/erl_cmake_tools)
    - [erl_common](https://github.com/ExistentialRobotics/erl_common)

# Setup

Clone this repository into your ROS 1/2 workspace:

```bash
cd <ros_ws>/src
git clone https://github.com/ExistentialRobotics/erl_cmake_tools.git
git clone https://github.com/ExistentialRobotics/erl_common.git
git clone https://github.com/ExistentialRobotics/erl_common_ros.git
cd <ros_ws>
# For ROS 1
catkin build
# For ROS 2
colcon build \
    --event-handlers console_cohesion+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON \
    --cmake-clean-cache --packages-up-to erl_common_ros
```

## Contributing

Contributions are welcome! Please open issues or submit pull requests.

## License

This project is licensed under the [MIT License](LICENSE).

## Contact

For questions or support, please contact the maintainers.
