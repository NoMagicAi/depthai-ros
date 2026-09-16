#pragma once

#include <deque>
#include <string>
#include <vector>

#include "depthai-shared/common/CameraSensorType.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/properties/ColorCameraProperties.hpp"
#include "depthai-shared/properties/MonoCameraProperties.hpp"
#include "depthai-shared/properties/VideoEncoderProperties.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "image_transport/camera_publisher.h"
#include "sensor_msgs/CameraInfo.h"

namespace dai {
class Device;
class Pipeline;
namespace node {
class VideoEncoder;
}
namespace ros {
class ImageConverter;
}
}  // namespace dai

namespace camera_info_manager {
class CameraInfoManager;
}

namespace depthai_ros_driver {
namespace dai_nodes {
namespace link_types {
enum class RGBLinkType { video, isp, preview };
}
namespace sensor_helpers {
enum class NodeNameEnum { RGB, Left, Right, Stereo, IMU, NN };
struct ImageSensor {
    std::string name;
    std::string defaultResolution;
    std::vector<std::string> allowedResolutions;
    dai::CameraSensorType sensorType;
    void getSizeFromResolution(const dai::ColorCameraProperties::SensorResolution& res, int& width, int& height);
};

extern std::vector<ImageSensor> availableSensors;
extern const std::unordered_map<dai::CameraBoardSocket, std::string> socketNameMap;
extern const std::unordered_map<dai::CameraBoardSocket, std::string> rsSocketNameMap;
extern const std::unordered_map<NodeNameEnum, std::string> rsNodeNameMap;
extern const std::unordered_map<NodeNameEnum, std::string> NodeNameMap;
extern const std::unordered_map<std::string, dai::MonoCameraProperties::SensorResolution> monoResolutionMap;
extern const std::unordered_map<std::string, dai::ColorCameraProperties::SensorResolution> rgbResolutionMap;
extern const std::unordered_map<std::string, dai::CameraControl::FrameSyncMode> fSyncModeMap;
extern const std::unordered_map<std::string, dai::CameraImageOrientation> cameraImageOrientationMap;
extern const std::unordered_map<std::string, dai::ColorCameraProperties::ColorOrder> colorOrderMap;
bool rsCompabilityMode(ros::NodeHandle node);
std::string tfPrefix(ros::NodeHandle node);
std::string getSocketName(ros::NodeHandle node, dai::CameraBoardSocket socket);
std::string getNodeName(ros::NodeHandle node, NodeNameEnum name);
void basicCameraPub(const std::string& /*name*/,
                    const std::shared_ptr<dai::ADatatype>& data,
                    dai::ros::ImageConverter& converter,
                    image_transport::CameraPublisher& pub,
                    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager);

void cameraPub(const std::string& /*name*/,
               const std::shared_ptr<dai::ADatatype>& data,
               dai::ros::ImageConverter& converter,
               image_transport::CameraPublisher& pub,
               std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
               bool lazyPub = true);

sensor_msgs::CameraInfo getCalibInfo(
    std::shared_ptr<dai::ros::ImageConverter> converter, std::shared_ptr<dai::Device> device, dai::CameraBoardSocket socket, int width = 0, int height = 0);
/**
 * camera_info for the frame described by pubConfig (socket, width, height and the sensor/ISP geometry
 * fields). Unlike the size-only overload, this maps the calibrated intrinsics through the actual frame
 * chain: a sensor mode whose aspect ratio differs from the calibration frame is treated as a centered
 * crop (plus power-of-two binning) of it rather than a stretch, ISP scaling as a uniform resize, and a
 * ColorCamera video output smaller than the ISP frame as a centered crop.
 */
sensor_msgs::CameraInfo getCalibInfo(std::shared_ptr<dai::ros::ImageConverter> converter,
                                     std::shared_ptr<dai::Device> device,
                                     const utils::ImgPublisherConfig& pubConfig);
/**
 * Intrinsics of a sensor-mode readout frame (modeWidth x modeHeight) given the calibration intrinsics
 * at calWidth x calHeight. Same aspect ratio: plain resize. Different aspect ratio: the mode is a
 * centered crop of the calibration frame followed by power-of-two binning, e.g. IMX378 12MP
 * (4056x3040) -> 4K is a 3840x2160 crop, -> 1080P is that crop binned 2x2. Exposed for testing.
 */
std::vector<std::vector<float>> intrinsicsForSensorMode(
    std::vector<std::vector<float>> intrinsics, int calWidth, int calHeight, int modeWidth, int modeHeight);
std::shared_ptr<dai::node::VideoEncoder> createEncoder(std::shared_ptr<dai::Pipeline> pipeline,
                                                       int quality,
                                                       dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::MJPEG);

}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
