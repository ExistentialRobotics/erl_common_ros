#include "erl_common/ros2_topic_params.hpp"
#include "erl_common/yaml.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <memory>

using namespace erl::common;
using namespace erl::common::ros_params;

static rclcpp::Node *g_curr_node = nullptr;

struct Options : public Yamlable<Options> {
    std::string parent_frame = "map";
    std::string child_frame = "base_link";
    double rate_hz = 10.0;
    double min_translation = 0.05;          // meters; <=0 disables gating
    double min_rotation = 0.05;             // radians; <=0 disables gating
    int64_t max_poses = 0;                  // 0 means unbounded
    double tf_lookup_timeout = 0.1;         // seconds
    bool publish_only_on_change = true;     // publish only when a new pose is appended
    Ros2TopicParams path_topic{"tf_path"};  // output nav_msgs/Path topic

    ERL_REFLECT_SCHEMA(
        Options,
        ERL_REFLECT_MEMBER(Options, parent_frame),
        ERL_REFLECT_MEMBER(Options, child_frame),
        ERL_REFLECT_MEMBER(Options, rate_hz),
        ERL_REFLECT_MEMBER(Options, min_translation),
        ERL_REFLECT_MEMBER(Options, min_rotation),
        ERL_REFLECT_MEMBER(Options, max_poses),
        ERL_REFLECT_MEMBER(Options, tf_lookup_timeout),
        ERL_REFLECT_MEMBER(Options, publish_only_on_change),
        ERL_REFLECT_MEMBER(Options, path_topic));

    bool
    PostDeserialization() override {
        auto logger = g_curr_node->get_logger();
        if (parent_frame.empty()) {
            RCLCPP_ERROR(logger, "parent_frame is empty");
            return false;
        }
        if (child_frame.empty()) {
            RCLCPP_ERROR(logger, "child_frame is empty");
            return false;
        }
        if (rate_hz <= 0.0) {
            RCLCPP_ERROR(logger, "Invalid rate_hz: %f. Must be > 0.0", rate_hz);
            return false;
        }
        if (path_topic.path.empty()) {
            RCLCPP_ERROR(logger, "path_topic.path is empty");
            return false;
        }
        return true;
    }
};

class TfToPathNode : public rclcpp::Node {
    Options m_cfg_;
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    nav_msgs::msg::Path m_path_;
    bool m_have_last_ = false;
    double m_last_x_ = 0.0;
    double m_last_y_ = 0.0;
    double m_last_z_ = 0.0;
    double m_last_qx_ = 0.0;
    double m_last_qy_ = 0.0;
    double m_last_qz_ = 0.0;
    double m_last_qw_ = 1.0;

public:
    TfToPathNode()
        : Node("tf_to_path_node") {
        g_curr_node = this;
        auto logger = this->get_logger();
        if (!m_cfg_.LoadFromRos2(this, "")) {
            RCLCPP_FATAL(logger, "Failed to load parameters");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(logger, "Loaded node parameters:\n%s", m_cfg_.AsYamlString().c_str());

        m_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_);

        m_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            m_cfg_.path_topic.path,
            m_cfg_.path_topic.GetQoS());

        m_path_.header.frame_id = m_cfg_.parent_frame;

        const auto period = std::chrono::duration<double>(1.0 / m_cfg_.rate_hz);
        m_timer_ = this->create_wall_timer(period, std::bind(&TfToPathNode::CallbackTimer, this));

        RCLCPP_INFO(
            logger,
            "TfToPathNode initialized: %s -> %s, publishing on '%s' at %.2f Hz",
            m_cfg_.parent_frame.c_str(),
            m_cfg_.child_frame.c_str(),
            m_cfg_.path_topic.path.c_str(),
            m_cfg_.rate_hz);
    }

private:
    void
    CallbackTimer() {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = m_tf_buffer_->lookupTransform(
                m_cfg_.parent_frame,
                m_cfg_.child_frame,
                tf2::TimePointZero,
                tf2::durationFromSec(m_cfg_.tf_lookup_timeout));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "TF lookup %s -> %s failed: %s",
                m_cfg_.parent_frame.c_str(),
                m_cfg_.child_frame.c_str(),
                ex.what());
            return;
        }

        const double x = tf.transform.translation.x;
        const double y = tf.transform.translation.y;
        const double z = tf.transform.translation.z;
        const double qx = tf.transform.rotation.x;
        const double qy = tf.transform.rotation.y;
        const double qz = tf.transform.rotation.z;
        const double qw = tf.transform.rotation.w;

        bool append = !m_have_last_;
        if (m_have_last_) {
            const double dx = x - m_last_x_;
            const double dy = y - m_last_y_;
            const double dz = z - m_last_z_;
            const double d2 = dx * dx + dy * dy + dz * dz;
            const double trans_thresh2 = m_cfg_.min_translation * m_cfg_.min_translation;
            const bool trans_ok = m_cfg_.min_translation <= 0.0 || d2 >= trans_thresh2;

            // Angle between quaternions: angle = 2 * acos(|dot|)
            double dot = m_last_qx_ * qx + m_last_qy_ * qy + m_last_qz_ * qz + m_last_qw_ * qw;
            if (dot > 1.0) { dot = 1.0; }
            if (dot < -1.0) { dot = -1.0; }
            const double angle = 2.0 * std::acos(std::abs(dot));
            const bool rot_ok = m_cfg_.min_rotation <= 0.0 || angle >= m_cfg_.min_rotation;

            append = trans_ok || rot_ok;
        }

        if (append) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = tf.header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation = tf.transform.rotation;
            m_path_.poses.push_back(pose);

            if (m_cfg_.max_poses > 0 &&
                static_cast<int64_t>(m_path_.poses.size()) > m_cfg_.max_poses) {
                const size_t drop = m_path_.poses.size() - static_cast<size_t>(m_cfg_.max_poses);
                m_path_.poses.erase(m_path_.poses.begin(), m_path_.poses.begin() + drop);
            }

            m_have_last_ = true;
            m_last_x_ = x;
            m_last_y_ = y;
            m_last_z_ = z;
            m_last_qx_ = qx;
            m_last_qy_ = qy;
            m_last_qz_ = qz;
            m_last_qw_ = qw;
        }

        if (append || !m_cfg_.publish_only_on_change) {
            m_path_.header.stamp = tf.header.stamp;
            m_path_pub_->publish(m_path_);
        }
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfToPathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
