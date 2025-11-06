#include "erl_common/yaml.hpp"

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

using namespace erl::common;

struct Options : public Yamlable<Options> {
    double clock_rate = 100.0;  // Hz
    double start_time = 0.0;    // seconds

    ERL_REFLECT_SCHEMA(
        Options,
        ERL_REFLECT_MEMBER(Options, clock_rate),
        ERL_REFLECT_MEMBER(Options, start_time));

    bool
    PostDeserialization() override {
        if (clock_rate <= 0.0) {
            ROS_ERROR("Invalid clock_rate: %f. Must be > 0.0", clock_rate);
            return false;
        }
        if (start_time < 0.0) {
            ROS_ERROR("Invalid start_time: %f. Must be >= 0.0", start_time);
            return false;
        }
        return true;
    }
};

int
main(int argc, char **argv) {
    ros::init(argc, argv, "clock_node");
    ros::NodeHandle nh("~");

    Options cfg;
    if (!cfg.LoadFromRos1(nh, "")) {
        ROS_FATAL("Failed to load parameters");
        return -1;
    }
    ROS_INFO("Loaded node parameters:\n%s", cfg.AsYamlString().c_str());

    // Publisher for the clock message
    ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);

    ros::Rate rate(cfg.clock_rate);
    ros::Duration dt(1.0 / cfg.clock_rate);
    ROS_INFO("Clock started with rate: %.2f Hz, start_time: %.2f", cfg.clock_rate, cfg.start_time);

    ros::Time current_time(cfg.start_time);
    ros::Time::setNow(current_time);
    while (ros::ok()) {
        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = current_time;
        clock_pub.publish(clock_msg);
        current_time += dt;
        rate.sleep();
    }

    return 0;
}
