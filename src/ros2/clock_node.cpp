#include "erl_common/yaml.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <chrono>
#include <memory>

static rclcpp::Node *g_curr_node = nullptr;

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
        auto logger = g_curr_node->get_logger();
        if (clock_rate <= 0.0) {
            RCLCPP_ERROR(logger, "Invalid clock_rate: %f. Must be > 0.0", clock_rate);
            return false;
        }
        if (start_time < 0.0) {
            RCLCPP_ERROR(logger, "Invalid start_time: %f. Must be >= 0.0", start_time);
            return false;
        }
        return true;
    }
};

class ClockNode : public rclcpp::Node {

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_clock_pub_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    rclcpp::Time m_current_time_;
    rclcpp::Duration m_dt_ = rclcpp::Duration::from_seconds(0.01);  // default 100Hz

public:
    ClockNode()
        : Node("clock_node") {
        g_curr_node = this;
        auto logger = this->get_logger();
        Options cfg;
        if (!cfg.LoadFromRos2(this, "")) {
            RCLCPP_FATAL(logger, "Failed to load parameters");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(logger, "Loaded node parameters:\n%s", cfg.AsYamlString().c_str());

        // Create publisher for clock messages
        m_clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Calculate timer period
        auto timer_period = std::chrono::duration<double>(1.0 / cfg.clock_rate);
        m_dt_ = rclcpp::Duration::from_seconds(1.0 / cfg.clock_rate);

        RCLCPP_INFO(
            this->get_logger(),
            "Clock node started with rate: %.2f Hz, start_time: %.2f",
            cfg.clock_rate,
            cfg.start_time);

        // Initialize current time
        m_current_time_ = rclcpp::Time(static_cast<int64_t>(cfg.start_time * 1e9));

        // Create timer
        m_timer_ =
            this->create_wall_timer(timer_period, std::bind(&ClockNode::CallbackTimer, this));
    }

private:
    void
    CallbackTimer() {
        auto clock_msg = rosgraph_msgs::msg::Clock();
        clock_msg.clock = m_current_time_;
        m_clock_pub_->publish(clock_msg);
        m_current_time_ = m_current_time_ + m_dt_;
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClockNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
