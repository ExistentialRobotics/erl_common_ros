#include "erl_common/yaml.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <memory>

static rclcpp::Node *g_curr_node = nullptr;

using namespace erl::common;

struct Options : public Yamlable<Options> {
    double clock_rate = 100.0;  // Hz
    double start_time = 0.0;    // seconds
    // if true, subscribe to /tf, read one message, use its header stamp as start_time,
    // then unsubscribe.
    bool start_time_from_tf = false;

    ERL_REFLECT_SCHEMA(
        Options,
        ERL_REFLECT_MEMBER(Options, clock_rate),
        ERL_REFLECT_MEMBER(Options, start_time),
        ERL_REFLECT_MEMBER(Options, start_time_from_tf));

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

    Options m_cfg_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_sub_tf_;

public:
    ClockNode()
        : Node("clock_node") {
        g_curr_node = this;
        auto logger = this->get_logger();
        if (!m_cfg_.LoadFromRos2(this, "")) {
            RCLCPP_FATAL(logger, "Failed to load parameters");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(logger, "Loaded node parameters:\n%s", m_cfg_.AsYamlString().c_str());

        // Create publisher for clock messages
        m_clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Calculate timer period
        m_dt_ = rclcpp::Duration::from_seconds(1.0 / m_cfg_.clock_rate);

        if (m_cfg_.start_time_from_tf) {
            RCLCPP_INFO(logger, "Waiting for one message on '/tf' to determine start_time ...");
            m_sub_tf_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
                "/tf",
                10,
                std::bind(&ClockNode::CallbackTf, this, std::placeholders::_1));
        } else {
            StartClock();
        }
    }

private:
    void
    CallbackTf(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        if (msg->transforms.empty()) { return; }
        const auto &stamp = msg->transforms.front().header.stamp;
        m_cfg_.start_time =
            static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
        RCLCPP_INFO(
            this->get_logger(),
            "Got start_time from /tf: %d.%09u (%.6f s)",
            stamp.sec,
            stamp.nanosec,
            m_cfg_.start_time);
        m_sub_tf_.reset();
        StartClock();
    }

    void
    StartClock() {
        RCLCPP_INFO(
            this->get_logger(),
            "Clock node started with rate: %.2f Hz, start_time: %.6f",
            m_cfg_.clock_rate,
            m_cfg_.start_time);
        m_current_time_ = rclcpp::Time(static_cast<int64_t>(m_cfg_.start_time * 1e9));
        auto timer_period = std::chrono::duration<double>(1.0 / m_cfg_.clock_rate);
        m_timer_ =
            this->create_wall_timer(timer_period, std::bind(&ClockNode::CallbackTimer, this));
    }

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
