#pragma once

// NoMagic: native ROS1 ("ROS One") side of the dual-stack driver.
//
// This header is only reachable from code guarded by #ifdef NOMAGIC_ROS1 and is
// only compiled when the CMake option NOMAGIC_ROS1 is ON. All ROS1 code lives
// in src/ros1/ros1_node.cpp so that rebases onto new upstream releases only
// have to carry the few hooks listed in docs/NOMAGIC_ROS1.md.
//
// IMPORTANT: this header must NOT include any ROS1 header (pimpl below). ROS1
// installs a flat include tree (/opt/ros/one/include) whose pluginlib, tf2_ros,
// class_loader, ... headers shadow the same-named ROS2 headers; only the single
// translation unit ros1_node.cpp gets the ROS1 include directories.
//
// Design constraints (see the NoMagic ROS1_AND_2_SUPPORT doc, section 4):
// - roscpp and rclcpp run in one process: ROS1 spins on a ros::AsyncSpinner,
//   ROS2 on the regular rclcpp executor.
// - The ROS1 side uses plain ros::Publisher / ros::ServiceServer only. ROS1
//   image_transport / camera_info_manager / tf2_ros cannot be linked because
//   their sonames clash with the ROS2 twins already linked by this driver.

#include <functional>
#include <memory>
#include <string>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_ros_driver {
namespace ros1 {

/// Publishes a ROS2 image + camera_info pair on ROS1 topics.
class Ros1CameraPublisher {
   public:
    /// Topic names must be fully resolved (global) names. Use
    /// Ros1Node::advertiseCamera() instead of constructing directly.
    Ros1CameraPublisher(const std::string& imageTopic, const std::string& infoTopic);
    ~Ros1CameraPublisher();
    void publish(const sensor_msgs::msg::Image& image, const sensor_msgs::msg::CameraInfo& info);
    /// True when the image or camera_info topic has at least one ROS1 subscriber.
    bool hasSubscribers() const;

   private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

/// Process-wide ROS1 context: lazy ros::init(), an AsyncSpinner, publisher and
/// std_srvs/Trigger service factories. All methods are safe to call when ROS1
/// is inactive (they become no-ops / return null).
class Ros1Node {
   public:
    /// Initialize ROS1 with the given node name (use the rclcpp node name so
    /// both graphs show the same node). Idempotent. Returns active().
    /// ROS1 is skipped (with a log line) when the environment variable
    /// NOMAGIC_ROS1_ENABLE is set to "0" or when no ROS master is reachable.
    static bool init(const std::string& nodeName);
    static bool active();
    /// Advertise image+info publishers. Topic names must be fully resolved
    /// (global) names, e.g. "/my_camera/rgb/image_raw". Null when inactive.
    static std::shared_ptr<Ros1CameraPublisher> advertiseCamera(const std::string& imageTopic, const std::string& infoTopic);
    /// Advertise a std_srvs/Trigger service in the node's private namespace
    /// (name relative to ~), mirroring the v2.x ROS1 driver services.
    static void advertiseTrigger(const std::string& name, std::function<bool(std::string& message)> handler);
    static void shutdown();
};

}  // namespace ros1
}  // namespace depthai_ros_driver
