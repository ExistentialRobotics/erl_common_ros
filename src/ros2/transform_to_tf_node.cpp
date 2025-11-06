#include "erl_common/ros2_topic_params.hpp"
#include "erl_common/yaml.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

using namespace erl::common;
using namespace erl::common::ros_params;

static rclcpp::Node *g_curr_node = nullptr;

struct Options : public Yamlable<Options> {
    Ros2TopicParams transform_topic{"transform"};

    ERL_REFLECT_SCHEMA(Options, ERL_REFLECT_MEMBER(Options, transform_topic));

    bool
    PostDeserialization() override {
        if (transform_topic.path.empty()) {
            RCLCPP_ERROR(g_curr_node->get_logger(), "transform_topic parameter is empty");
            return false;
        }
        return true;
    }
};

class TransformToTfNode : public rclcpp::Node {
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr m_sub_transform_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_broadcaster_;
    bool m_verbose_ = true;

public:
    TransformToTfNode()
        : Node("transform_to_tf_node") {

        g_curr_node = this;
        auto logger = this->get_logger();
        Options cfg;
        if (!cfg.LoadFromRos2(this, "")) {
            RCLCPP_FATAL(logger, "Failed to load parameters");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(logger, "Loaded node parameters:\n%s", cfg.AsYamlString().c_str());

        // Initialize the transform broadcaster
        m_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to the transform topic
        m_sub_transform_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            cfg.transform_topic.path,
            cfg.transform_topic.GetQoS(),
            std::bind(&TransformToTfNode::Callback, this, std::placeholders::_1));

        RCLCPP_INFO(
            logger,
            "TransformToTfNode initialized, listening to topic: %s",
            cfg.transform_topic.path.c_str());
    }

    void
    Callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
        // Broadcast the transform
        m_broadcaster_->sendTransform(*msg);
        if (m_verbose_) {
            RCLCPP_INFO(
                this->get_logger(),
                "Broadcasting transform from %s to %s",
                msg->header.frame_id.c_str(),
                msg->child_frame_id.c_str());
            m_verbose_ = false;  // only log once
        }
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformToTfNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
