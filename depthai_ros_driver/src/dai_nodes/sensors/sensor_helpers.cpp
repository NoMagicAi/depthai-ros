#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"

#include <cmath>
#include <tuple>

#include "camera_info_manager/camera_info_manager.h"
#include "depthai/device/Device.hpp"
#include "depthai-shared/common/CameraSensorType.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai_bridge/ImageConverter.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {
std::vector<ImageSensor> availableSensors = {{"IMX378", "1080P", {"12MP", "4K", "1080P"}, dai::CameraSensorType::COLOR},
                                             {"IMX462", "1080P", {"1080P"}, dai::CameraSensorType::COLOR},
                                             {"OV9282", "720P", {"800P", "720P", "400P"}, dai::CameraSensorType::MONO},
                                             {"OV9782", "720P", {"800P", "720P", "400P"}, dai::CameraSensorType::COLOR},
                                             {"OV9281", "720P", {"800P", "720P", "400P"}, dai::CameraSensorType::COLOR},
                                             {"IMX214", "1080P", {"13MP", "12MP", "4K", "1080P"}, dai::CameraSensorType::COLOR},
                                             {"IMX412", "1080P", {"13MP", "12MP", "4K", "1080P"}, dai::CameraSensorType::COLOR},
                                             {"OV7750", "480P", {"480P", "400P"}, dai::CameraSensorType::MONO},
                                             {"OV7251", "480P", {"480P", "400P"}, dai::CameraSensorType::MONO},
                                             {"IMX477", "1080P", {"12MP", "4K", "1080P"}, dai::CameraSensorType::COLOR},
                                             {"IMX577", "1080P", {"12MP", "4K", "1080P"}, dai::CameraSensorType::COLOR},
                                             {"AR0234", "1200P", {"1200P"}, dai::CameraSensorType::COLOR},
                                             {"IMX582", "4K", {"48MP", "12MP", "4K"}, dai::CameraSensorType::COLOR},
                                             {"LCM48", "4K", {"48MP", "12MP", "4K"}, dai::CameraSensorType::COLOR},
                                             {"TINY1C", "256", {"256"}, dai::CameraSensorType::THERMAL}};
const std::unordered_map<std::string, dai::MonoCameraProperties::SensorResolution> monoResolutionMap = {
    {"400P", dai::MonoCameraProperties::SensorResolution::THE_400_P},
    {"480P", dai::MonoCameraProperties::SensorResolution::THE_480_P},
    {"720P", dai::MonoCameraProperties::SensorResolution::THE_720_P},
    {"800P", dai::MonoCameraProperties::SensorResolution::THE_800_P},
    {"1200P", dai::MonoCameraProperties::SensorResolution::THE_1200_P},
};

const std::unordered_map<std::string, dai::ColorCameraProperties::SensorResolution> rgbResolutionMap = {
    {"720P", dai::ColorCameraProperties::SensorResolution::THE_720_P},
    {"1080P", dai::ColorCameraProperties::SensorResolution::THE_1080_P},
    {"4K", dai::ColorCameraProperties::SensorResolution::THE_4_K},
    {"12MP", dai::ColorCameraProperties::SensorResolution::THE_12_MP},
    {"13MP", dai::ColorCameraProperties::SensorResolution::THE_13_MP},
    {"800P", dai::ColorCameraProperties::SensorResolution::THE_800_P},
    {"1200P", dai::ColorCameraProperties::SensorResolution::THE_1200_P},
    {"5MP", dai::ColorCameraProperties::SensorResolution::THE_5_MP},
    {"4000x3000", dai::ColorCameraProperties::SensorResolution::THE_4000X3000},
    {"5312X6000", dai::ColorCameraProperties::SensorResolution::THE_5312X6000},
    {"48MP", dai::ColorCameraProperties::SensorResolution::THE_48_MP},
    {"1440X1080", dai::ColorCameraProperties::SensorResolution::THE_1440X1080}};

const std::unordered_map<std::string, dai::CameraControl::FrameSyncMode> fSyncModeMap = {
    {"OFF", dai::CameraControl::FrameSyncMode::OFF},
    {"OUTPUT", dai::CameraControl::FrameSyncMode::OUTPUT},
    {"INPUT", dai::CameraControl::FrameSyncMode::INPUT},
};
const std::unordered_map<std::string, dai::CameraImageOrientation> cameraImageOrientationMap = {
    {"NORMAL", dai::CameraImageOrientation::NORMAL},
    {"ROTATE_180_DEG", dai::CameraImageOrientation::ROTATE_180_DEG},
    {"AUTO", dai::CameraImageOrientation::AUTO},
    {"HORIZONTAL_MIRROR", dai::CameraImageOrientation::HORIZONTAL_MIRROR},
    {"VERTICAL_FLIP", dai::CameraImageOrientation::VERTICAL_FLIP},
};

const std::unordered_map<dai::CameraBoardSocket, std::string> socketNameMap = {
    {dai::CameraBoardSocket::AUTO, "rgb"},
    {dai::CameraBoardSocket::CAM_A, "rgb"},
    {dai::CameraBoardSocket::CAM_B, "left"},
    {dai::CameraBoardSocket::CAM_C, "right"},
    {dai::CameraBoardSocket::CAM_D, "cam_d"},
    {dai::CameraBoardSocket::CAM_E, "cam_e"},
};
const std::unordered_map<dai::CameraBoardSocket, std::string> rsSocketNameMap = {
    {dai::CameraBoardSocket::AUTO, "color"},
    {dai::CameraBoardSocket::CAM_A, "color"},
    {dai::CameraBoardSocket::CAM_B, "infra2"},
    {dai::CameraBoardSocket::CAM_C, "infra1"},
    {dai::CameraBoardSocket::CAM_D, "infra4"},
    {dai::CameraBoardSocket::CAM_E, "infra3"},
};
const std::unordered_map<NodeNameEnum, std::string> rsNodeNameMap = {
    {NodeNameEnum::RGB, "color"},
    {NodeNameEnum::Left, "infra2"},
    {NodeNameEnum::Right, "infra1"},
    {NodeNameEnum::Stereo, "depth"},
    {NodeNameEnum::IMU, "imu"},
    {NodeNameEnum::NN, "nn"},
};

const std::unordered_map<NodeNameEnum, std::string> NodeNameMap = {
    {NodeNameEnum::RGB, "rgb"},
    {NodeNameEnum::Left, "left"},
    {NodeNameEnum::Right, "right"},
    {NodeNameEnum::Stereo, "stereo"},
    {NodeNameEnum::IMU, "imu"},
    {NodeNameEnum::NN, "nn"},
};

const std::unordered_map<std::string, dai::ColorCameraProperties::ColorOrder> colorOrderMap = {{"BGR", dai::ColorCameraProperties::ColorOrder::BGR},
                                                                                               {"RGB", dai::ColorCameraProperties::ColorOrder::RGB}};

bool rsCompabilityMode(ros::NodeHandle node) {
    bool compat = false;
    node.getParam("camera_i_rs_compat", compat);
    return compat;
}
std::string tfPrefix(ros::NodeHandle node) {
    bool pubTF = false;
    std::string tfPrefix = node.getNamespace();
    node.getParam("camera_i_publish_tf_from_calibration", pubTF);
    if(pubTF) {
        node.getParam("camera_i_tf_base_frame", tfPrefix);
    }
    return tfPrefix;
}
std::string getNodeName(ros::NodeHandle node, NodeNameEnum name) {
    if(rsCompabilityMode(node)) {
        return rsNodeNameMap.at(name);
    }
    return NodeNameMap.at(name);
}

std::string getSocketName(ros::NodeHandle node, dai::CameraBoardSocket socket) {
    if(rsCompabilityMode(node)) {
        return rsSocketNameMap.at(socket);
    }
    return socketNameMap.at(socket);
}

void basicCameraPub(const std::string& /*name*/,
                    const std::shared_ptr<dai::ADatatype>& data,
                    dai::ros::ImageConverter& converter,
                    image_transport::CameraPublisher& pub,
                    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager) {
    if(ros::ok() && (pub.getNumSubscribers() > 0)) {
        auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        auto info = infoManager->getCameraInfo();
        auto rawMsg = converter.toRosMsgRawPtr(img);
        info.header = rawMsg.header;
        pub.publish(rawMsg, info);
    }
}

void cameraPub(const std::string& /*name*/,
               const std::shared_ptr<dai::ADatatype>& data,
               dai::ros::ImageConverter& converter,
               image_transport::CameraPublisher& pub,
               std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
               bool lazyPub) {
    if(ros::ok() && (!lazyPub || pub.getNumSubscribers() > 0)) {
        auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        auto info = infoManager->getCameraInfo();
        auto rawMsg = converter.toRosMsgRawPtr(img, info);
        info.header = rawMsg.header;
        pub.publish(rawMsg, info);
    }
}
sensor_msgs::CameraInfo getCalibInfo(
    std::shared_ptr<dai::ros::ImageConverter> converter, std::shared_ptr<dai::Device> device, dai::CameraBoardSocket socket, int width, int height) {
    sensor_msgs::CameraInfo info;
    auto calibHandler = device->readCalibration();

    try {
        info = converter->calibrationToCameraInfo(calibHandler, socket, width, height);
    } catch(std::runtime_error& e) {
        ROS_ERROR("No calibration for socket %d! Publishing empty camera_info.", static_cast<int>(socket));
    }
    return info;
}

namespace {
using Intrinsics = std::vector<std::vector<float>>;

Intrinsics scaleIntrinsics(Intrinsics k, float sx, float sy) {
    k[0][0] *= sx;
    k[0][2] *= sx;
    k[1][1] *= sy;
    k[1][2] *= sy;
    return k;
}

// Intrinsics of a width x height window whose top-left corner sits at (x0, y0) in the source frame.
Intrinsics cropIntrinsics(Intrinsics k, float x0, float y0) {
    k[0][2] -= x0;
    k[1][2] -= y0;
    return k;
}

bool sameAspectRatio(int w1, int h1, int w2, int h2) {
    // Sizes are integers, so a 1% tolerance separates 4:3 from 16:9 comfortably while absorbing rounding
    // such as 1352x1012 vs 4056x3040.
    return std::abs(static_cast<float>(w1) * h2 - static_cast<float>(h1) * w2) < 0.01f * w1 * h2;
}
}  // namespace

Intrinsics intrinsicsForSensorMode(Intrinsics intrinsics, int calWidth, int calHeight, int modeWidth, int modeHeight) {
    if(sameAspectRatio(calWidth, calHeight, modeWidth, modeHeight)) {
        return scaleIntrinsics(intrinsics, static_cast<float>(modeWidth) / calWidth, static_cast<float>(modeHeight) / calHeight);
    }
    // Different aspect ratio: the sensor exposes the mode as a centered window of the readout it was
    // calibrated at, binned by a power of two. The binning factor is the power of two nearest to the
    // size ratio (1 for 12MP -> 4K, 2 for 12MP -> 1080P, 1 for 800P -> 720P, 2 for 800P -> 400P); a
    // factor below one means the calibration frame is the smaller window (1080P calibration, 12MP mode).
    float ratio = std::min(static_cast<float>(calWidth) / modeWidth, static_cast<float>(calHeight) / modeHeight);
    float binning = std::exp2(std::round(std::log2(ratio)));
    float windowWidth = modeWidth * binning;
    float windowHeight = modeHeight * binning;
    ROS_INFO("Sensor mode %dx%d interpreted as a centered %gx%g window of the %dx%d calibration frame binned %gx",
             modeWidth,
             modeHeight,
             windowWidth,
             windowHeight,
             calWidth,
             calHeight,
             binning);
    intrinsics = cropIntrinsics(intrinsics, (calWidth - windowWidth) / 2.0f, (calHeight - windowHeight) / 2.0f);
    return scaleIntrinsics(intrinsics, 1.0f / binning, 1.0f / binning);
}

sensor_msgs::CameraInfo getCalibInfo(std::shared_ptr<dai::ros::ImageConverter> converter,
                                     std::shared_ptr<dai::Device> device,
                                     const utils::ImgPublisherConfig& pubConfig) {
    if(pubConfig.sensorWidth <= 0 || pubConfig.sensorHeight <= 0) {
        return getCalibInfo(converter, device, pubConfig.socket, pubConfig.width, pubConfig.height);
    }
    sensor_msgs::CameraInfo info;
    auto calibHandler = device->readCalibration();
    try {
        Intrinsics intrinsics;
        int calWidth, calHeight;
        std::tie(intrinsics, calWidth, calHeight) = calibHandler.getDefaultIntrinsics(pubConfig.socket);

        // 1. calibration frame -> sensor readout of the selected mode
        intrinsics = intrinsicsForSensorMode(intrinsics, calWidth, calHeight, pubConfig.sensorWidth, pubConfig.sensorHeight);
        // 2. ISP scaling (uniform in practice, but apply per axis to match what the ISP produced)
        int ispWidth = pubConfig.ispWidth > 0 ? pubConfig.ispWidth : pubConfig.sensorWidth;
        int ispHeight = pubConfig.ispHeight > 0 ? pubConfig.ispHeight : pubConfig.sensorHeight;
        intrinsics = scaleIntrinsics(
            intrinsics, static_cast<float>(ispWidth) / pubConfig.sensorWidth, static_cast<float>(ispHeight) / pubConfig.sensorHeight);
        // 3. ISP frame -> published frame
        if(pubConfig.croppedFromIsp) {
            // ColorCamera video output: centered crop, offset floored like ColorCamera::getSensorCrop
            if(pubConfig.width > ispWidth || pubConfig.height > ispHeight) {
                throw std::runtime_error("Published frame is larger than the ISP frame it is supposed to be cropped from");
            }
            intrinsics = cropIntrinsics(intrinsics, std::floor((ispWidth - pubConfig.width) / 2.0f), std::floor((ispHeight - pubConfig.height) / 2.0f));
        } else {
            intrinsics = scaleIntrinsics(intrinsics, static_cast<float>(pubConfig.width) / ispWidth, static_cast<float>(pubConfig.height) / ispHeight);
        }
        ROS_INFO("%s camera_info: calibration %dx%d -> sensor %dx%d -> isp %dx%d -> %s %dx%d, fx %.1f fy %.1f cx %.1f cy %.1f",
                 pubConfig.daiNodeName.c_str(),
                 calWidth,
                 calHeight,
                 pubConfig.sensorWidth,
                 pubConfig.sensorHeight,
                 ispWidth,
                 ispHeight,
                 pubConfig.croppedFromIsp ? "crop" : "resize",
                 pubConfig.width,
                 pubConfig.height,
                 intrinsics[0][0],
                 intrinsics[1][1],
                 intrinsics[0][2],
                 intrinsics[1][2]);
        info = converter->calibrationToCameraInfo(calibHandler, pubConfig.socket, pubConfig.width, pubConfig.height, intrinsics);
    } catch(std::runtime_error& e) {
        ROS_ERROR("No usable calibration for socket %d (%s)! Publishing empty camera_info.", static_cast<int>(pubConfig.socket), e.what());
    }
    return info;
}

}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
