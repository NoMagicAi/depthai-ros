// NoMagic: native ROS1 ("ROS One") side of the dual-stack driver.
// This is the ONLY translation unit that sees ROS1 headers; see ros1_node.hpp
// and docs/NOMAGIC_ROS1.md.

#include "depthai_ros_driver_v3/ros1/ros1_node.hpp"

#include <cstdlib>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros/master.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "std_srvs/Trigger.h"

namespace depthai_ros_driver {
namespace ros1 {

namespace {
struct Ros1State {
    std::mutex mutex;
    bool initialized = false;
    bool active = false;
    std::unique_ptr<ros::NodeHandle> nh;         // node namespace
    std::unique_ptr<ros::NodeHandle> pnh;        // private (~) namespace
    std::unique_ptr<ros::AsyncSpinner> spinner;  // services only; publishing needs no spin
    std::vector<ros::ServiceServer> services;
};
Ros1State& state() {
    static Ros1State s;
    return s;
}

sensor_msgs::Image toRos1(const sensor_msgs::msg::Image& in) {
    sensor_msgs::Image out;
    out.header.stamp.sec = in.header.stamp.sec;
    out.header.stamp.nsec = in.header.stamp.nanosec;
    out.header.frame_id = in.header.frame_id;
    out.height = in.height;
    out.width = in.width;
    out.encoding = in.encoding;
    out.is_bigendian = in.is_bigendian;
    out.step = in.step;
    out.data = in.data;
    return out;
}

sensor_msgs::CameraInfo toRos1(const sensor_msgs::msg::CameraInfo& in) {
    sensor_msgs::CameraInfo out;
    out.header.stamp.sec = in.header.stamp.sec;
    out.header.stamp.nsec = in.header.stamp.nanosec;
    out.header.frame_id = in.header.frame_id;
    out.height = in.height;
    out.width = in.width;
    out.distortion_model = in.distortion_model;
    out.D = in.d;
    std::copy(in.k.begin(), in.k.end(), out.K.begin());
    std::copy(in.r.begin(), in.r.end(), out.R.begin());
    std::copy(in.p.begin(), in.p.end(), out.P.begin());
    out.binning_x = in.binning_x;
    out.binning_y = in.binning_y;
    out.roi.x_offset = in.roi.x_offset;
    out.roi.y_offset = in.roi.y_offset;
    out.roi.height = in.roi.height;
    out.roi.width = in.roi.width;
    out.roi.do_rectify = in.roi.do_rectify;
    return out;
}
}  // namespace

struct Ros1CameraPublisher::Impl {
    ros::Publisher imgPub;
    ros::Publisher infoPub;
};

Ros1CameraPublisher::Ros1CameraPublisher(const std::string& imageTopic, const std::string& infoTopic) : impl(std::make_unique<Impl>()) {
    auto& s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    if(s.active) {
        impl->imgPub = s.nh->advertise<sensor_msgs::Image>(imageTopic, 10);
        impl->infoPub = s.nh->advertise<sensor_msgs::CameraInfo>(infoTopic, 10);
    }
}

Ros1CameraPublisher::~Ros1CameraPublisher() = default;

void Ros1CameraPublisher::publish(const sensor_msgs::msg::Image& image, const sensor_msgs::msg::CameraInfo& info) {
    impl->imgPub.publish(toRos1(image));
    impl->infoPub.publish(toRos1(info));
}

bool Ros1CameraPublisher::hasSubscribers() const {
    return impl->imgPub.getNumSubscribers() > 0 || impl->infoPub.getNumSubscribers() > 0;
}

bool Ros1Node::init(const std::string& nodeName) {
    auto& s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    if(s.initialized) {
        return s.active;
    }
    s.initialized = true;
    const char* enable = std::getenv("NOMAGIC_ROS1_ENABLE");
    if(enable != nullptr && std::string(enable) == "0") {
        RCLCPP_INFO(rclcpp::get_logger(nodeName), "NoMagic ROS1 side disabled via NOMAGIC_ROS1_ENABLE=0");
        return false;
    }
    if(!ros::isInitialized()) {
        // No remapping args: the ROS2 side owns the command line. NoSigintHandler:
        // lifetime is coupled to rclcpp below.
        ros::M_string remappings;
        ros::init(remappings, nodeName, ros::init_options::NoSigintHandler);
    }
    if(!ros::master::check()) {
        RCLCPP_WARN(rclcpp::get_logger(nodeName),
                    "NoMagic ROS1 side: no ROS master reachable at %s, ROS1 publishing disabled.",
                    ros::master::getURI().c_str());
        return false;
    }
    s.nh = std::make_unique<ros::NodeHandle>();
    s.pnh = std::make_unique<ros::NodeHandle>("~");
    s.spinner = std::make_unique<ros::AsyncSpinner>(1);
    s.spinner->start();
    s.active = true;
    // A ROS2 shutdown (SIGINT from ros2 launch/run) also stops the ROS1 side.
    rclcpp::on_shutdown([]() { Ros1Node::shutdown(); });
    RCLCPP_INFO(rclcpp::get_logger(nodeName),
                "NoMagic ROS1 side active: node %s, master %s",
                ros::this_node::getName().c_str(),
                ros::master::getURI().c_str());
    return true;
}

bool Ros1Node::active() {
    auto& s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    return s.active;
}

std::shared_ptr<Ros1CameraPublisher> Ros1Node::advertiseCamera(const std::string& imageTopic, const std::string& infoTopic) {
    if(!active()) {
        return nullptr;
    }
    return std::make_shared<Ros1CameraPublisher>(imageTopic, infoTopic);
}

void Ros1Node::advertiseTrigger(const std::string& name, std::function<bool(std::string&)> handler) {
    auto& s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    if(!s.active) {
        return;
    }
    boost::function<bool(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)> cb =
        [handler](std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res) {
            std::string message;
            res.success = handler(message);
            res.message = message;
            return true;
        };
    s.services.push_back(s.pnh->advertiseService(name, cb));
}

void Ros1Node::shutdown() {
    auto& s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    if(!s.active) {
        return;
    }
    s.services.clear();
    if(s.spinner) {
        s.spinner->stop();
    }
    s.spinner.reset();
    s.pnh.reset();
    s.nh.reset();
    s.active = false;
    ros::shutdown();
}

}  // namespace ros1
}  // namespace depthai_ros_driver
