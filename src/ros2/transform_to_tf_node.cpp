#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

class TransformToTfNode : public rclcpp::Node {
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr m_sub_transform_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_broadcaster_;
    bool m_verbose_ = true;

public:
    TransformToTfNode()
        : Node("transform_to_tf_node") {
        // Declare parameters
        this->declare_parameter("transform_topic", "transform");

        // Initialize the transform broadcaster
        m_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Get parameter value
        std::string transform_topic = this->get_parameter("transform_topic").as_string();

        // Subscribe to the transform topic
        m_sub_transform_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            transform_topic,
            1,
            std::bind(&TransformToTfNode::Callback, this, std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "TransformToTfNode initialized, listening to topic: %s",
            transform_topic.c_str());
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
main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformToTfNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
