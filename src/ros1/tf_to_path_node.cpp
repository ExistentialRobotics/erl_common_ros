#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>

class TfToPathNode {
    ros::NodeHandle m_nh_;
    ros::Publisher m_path_pub_;
    ros::Timer m_timer_;
    tf2_ros::Buffer m_tf_buffer_;
    tf2_ros::TransformListener m_tf_listener_;

    std::string m_parent_frame_;
    std::string m_child_frame_;
    double m_rate_hz_ = 10.0;
    double m_min_translation_ = 0.05;
    double m_min_rotation_ = 0.05;
    int m_max_poses_ = 0;
    double m_tf_lookup_timeout_ = 0.1;
    bool m_publish_only_on_change_ = true;
    std::string m_path_topic_ = "tf_path";
    int m_path_queue_size_ = 10;

    nav_msgs::Path m_path_;
    bool m_have_last_ = false;
    double m_last_x_ = 0.0;
    double m_last_y_ = 0.0;
    double m_last_z_ = 0.0;
    double m_last_qx_ = 0.0;
    double m_last_qy_ = 0.0;
    double m_last_qz_ = 0.0;
    double m_last_qw_ = 1.0;

public:
    explicit TfToPathNode(ros::NodeHandle &nh)
        : m_nh_(nh),
          m_tf_listener_(m_tf_buffer_) {

        nh.param<std::string>("parent_frame", m_parent_frame_, std::string("map"));
        nh.param<std::string>("child_frame", m_child_frame_, std::string("base_link"));
        nh.param("rate_hz", m_rate_hz_, m_rate_hz_);
        nh.param("min_translation", m_min_translation_, m_min_translation_);
        nh.param("min_rotation", m_min_rotation_, m_min_rotation_);
        nh.param("max_poses", m_max_poses_, m_max_poses_);
        nh.param("tf_lookup_timeout", m_tf_lookup_timeout_, m_tf_lookup_timeout_);
        nh.param("publish_only_on_change", m_publish_only_on_change_, m_publish_only_on_change_);
        nh.param<std::string>("path_topic", m_path_topic_, m_path_topic_);
        nh.param("path_queue_size", m_path_queue_size_, m_path_queue_size_);

        if (m_parent_frame_.empty()) {
            ROS_FATAL("parent_frame is empty");
            ros::shutdown();
            return;
        }
        if (m_child_frame_.empty()) {
            ROS_FATAL("child_frame is empty");
            ros::shutdown();
            return;
        }
        if (m_rate_hz_ <= 0.0) {
            ROS_FATAL("Invalid rate_hz: %f. Must be > 0.0", m_rate_hz_);
            ros::shutdown();
            return;
        }
        if (m_path_topic_.empty()) {
            ROS_FATAL("path_topic is empty");
            ros::shutdown();
            return;
        }

        m_path_.header.frame_id = m_parent_frame_;
        m_path_pub_ = m_nh_.advertise<nav_msgs::Path>(m_path_topic_, m_path_queue_size_);

        m_timer_ = m_nh_.createTimer(
            ros::Duration(1.0 / m_rate_hz_),
            &TfToPathNode::CallbackTimer,
            this);

        ROS_INFO(
            "TfToPathNode initialized: %s -> %s, publishing on '%s' at %.2f Hz",
            m_parent_frame_.c_str(),
            m_child_frame_.c_str(),
            m_path_topic_.c_str(),
            m_rate_hz_);
    }

private:
    void
    CallbackTimer(const ros::TimerEvent & /*event*/) {
        geometry_msgs::TransformStamped tf;
        try {
            tf = m_tf_buffer_.lookupTransform(
                m_parent_frame_,
                m_child_frame_,
                ros::Time(0),
                ros::Duration(m_tf_lookup_timeout_));
        } catch (const tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(
                2.0,
                "TF lookup %s -> %s failed: %s",
                m_parent_frame_.c_str(),
                m_child_frame_.c_str(),
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
            const double trans_thresh2 = m_min_translation_ * m_min_translation_;
            const bool trans_ok = m_min_translation_ <= 0.0 || d2 >= trans_thresh2;

            double dot = m_last_qx_ * qx + m_last_qy_ * qy + m_last_qz_ * qz + m_last_qw_ * qw;
            if (dot > 1.0) { dot = 1.0; }
            if (dot < -1.0) { dot = -1.0; }
            const double angle = 2.0 * std::acos(std::abs(dot));
            const bool rot_ok = m_min_rotation_ <= 0.0 || angle >= m_min_rotation_;

            append = trans_ok || rot_ok;
        }

        if (append) {
            geometry_msgs::PoseStamped pose;
            pose.header = tf.header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation = tf.transform.rotation;
            m_path_.poses.push_back(pose);

            if (m_max_poses_ > 0 &&
                static_cast<int>(m_path_.poses.size()) > m_max_poses_) {
                const size_t drop =
                    m_path_.poses.size() - static_cast<size_t>(m_max_poses_);
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

        if (append || !m_publish_only_on_change_) {
            m_path_.header.stamp = tf.header.stamp;
            m_path_pub_.publish(m_path_);
        }
    }
};

int
main(int argc, char **argv) {
    ros::init(argc, argv, "tf_to_path_node");
    ros::NodeHandle nh("~");
    TfToPathNode node(nh);
    ros::spin();
    return 0;
}
