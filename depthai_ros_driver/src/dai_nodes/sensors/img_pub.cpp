#include "depthai_ros_driver_v3/dai_nodes/sensors/img_pub.hpp"

#include <rclcpp/logging.hpp>

#if __has_include("cv_bridge/cv_bridge.hpp")
    #include "cv_bridge/cv_bridge.hpp"
#else
    #include "cv_bridge/cv_bridge.h"
#endif

#include <opencv2/imgproc.hpp>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver_v3/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver_v3/utils.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {
ImagePublisher::ImagePublisher(std::shared_ptr<rclcpp::Node> node,
                               std::shared_ptr<dai::Pipeline> pipeline,
                               const std::string& qName,
                               dai::Node::Output* out,
                               bool synced,
                               bool ipcEnabled,
                               const utils::VideoEncoderConfig& encoderConfig)
    : node(node), encConfig(encoderConfig), out(out), qName(qName), ipcEnabled(ipcEnabled), synced(synced) {
    if(encoderConfig.enabled) {
        encoder = createEncoder(pipeline, encoderConfig);
        this->out->link(encoder->input);
    }
}
void ImagePublisher::setup(std::shared_ptr<dai::Device> device, const utils::ImgConverterConfig& convConf, const utils::ImgPublisherConfig& pubConf) {
    convConfig = convConf;
    pubConfig = pubConf;
    createImageConverter(device);
    createInfoManager(device);
    if(pubConfig.topicName.empty()) {
        throw std::runtime_error("Topic name cannot be empty!");
    }
    rclcpp::PublisherOptions pubOptions;
    pubOptions.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            compressedImgPub =
                node->create_publisher<sensor_msgs::msg::CompressedImage>(pubConfig.topicName + pubConfig.compressedTopicSuffix, rclcpp::QoS(10), pubOptions);
        } else {
            ffmpegPub = node->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
                pubConfig.topicName + pubConfig.compressedTopicSuffix, rclcpp::QoS(10), pubOptions);
        }
        infoPub =
            node->create_publisher<sensor_msgs::msg::CameraInfo>(pubConfig.topicName + pubConfig.infoSuffix + "/camera_info", rclcpp::QoS(10), pubOptions);
    } else {
        imgPubIT = image_transport::create_camera_publisher(node.get(), pubConfig.topicName + pubConfig.topicSuffix);
        if(pubConfig.hostSideUpscale > 0.0) {
            upscaledPubIT = image_transport::create_camera_publisher(node.get(), upscaledTopicName() + pubConfig.topicSuffix);
        }
#ifdef NOMAGIC_ROS1
        // NoMagic: mirror the image + camera_info pair on ROS1 (plain publishers).
        if(ros1::Ros1Node::active()) {
            // resolve_topic_name() (not expand_topic_or_service_name()) so that `--ros-args -r`
            // remappings apply: the ROS1 mirror has to land on the same names as the ROS2 side.
            auto& topics = *node->get_node_topics_interface();
            ros1Pub = ros1::Ros1Node::advertiseCamera(topics.resolve_topic_name(pubConfig.topicName + pubConfig.topicSuffix),
                                                      topics.resolve_topic_name(pubConfig.topicName + pubConfig.infoSuffix + "/camera_info"));
            if(pubConfig.hostSideUpscale > 0.0) {
                ros1UpscaledPub =
                    ros1::Ros1Node::advertiseCamera(topics.resolve_topic_name(upscaledTopicName() + pubConfig.topicSuffix),
                                                    topics.resolve_topic_name(upscaledTopicName() + pubConfig.infoSuffix + "/camera_info"));
            }
        }
#endif
    }
    if(!synced) {
        if(encConfig.enabled) {
            dataQ = encoder->out.createOutputQueue(pubConf.maxQSize, pubConf.qBlocking);
        } else {
            dataQ = out->createOutputQueue(pubConf.maxQSize, pubConf.qBlocking);
        }
        addQueueCB();
    }
}

void ImagePublisher::createImageConverter(std::shared_ptr<dai::Device> device) {
    converter = std::make_shared<depthai_bridge::ImageConverter>(convConfig.tfPrefix, convConfig.interleaved, convConfig.getBaseDeviceTimestamp);
    converter->setUpdateRosBaseTimeOnToRosMsg(convConfig.updateROSBaseTimeOnRosMsg);
    if(convConfig.lowBandwidth) {
        converter->convertFromBitstream(convConfig.encoding);
        if(convConfig.isStereo && !convConfig.outputDisparity) {
            try {
                auto calHandler = device->readCalibration();
                double baseline = calHandler.getBaselineDistance(pubConfig.leftSocket, pubConfig.rightSocket, false);
                if(convConfig.reverseSocketOrder) {
                    baseline = calHandler.getBaselineDistance(pubConfig.rightSocket, pubConfig.leftSocket, false);
                }
                converter->convertDispToDepth(baseline);
            } catch(const std::exception& e) {
                RCLCPP_DEBUG(node->get_logger(), "Failed to convert disparity to depth: %s", e.what());
            }
        }
    }
    if(convConfig.addExposureOffset) {
        converter->addExposureOffset(convConfig.expOffset);
    }
    if(convConfig.reverseSocketOrder) {
        converter->reverseStereoSocketOrder();
    }
    if(convConfig.alphaScalingEnabled) {
        converter->setAlphaScaling(convConfig.alphaScaling);
    }
    if(convConfig.isStereo && !convConfig.outputDisparity) {
        auto calHandler = device->readCalibration();
        double baseline = calHandler.getBaselineDistance(pubConfig.leftSocket, pubConfig.rightSocket, false);
        if(convConfig.reverseSocketOrder) {
            baseline = calHandler.getBaselineDistance(pubConfig.rightSocket, pubConfig.leftSocket, false);
        }
        converter->convertDispToDepth(baseline);
    }
    converter->setFFMPEGEncoding(convConfig.ffmpegEncoder);
}

std::shared_ptr<dai::node::VideoEncoder> ImagePublisher::createEncoder(std::shared_ptr<dai::Pipeline> pipeline,
                                                                       const utils::VideoEncoderConfig& encoderConfig) {
    auto enc = pipeline->create<dai::node::VideoEncoder>();
    enc->setQuality(encoderConfig.quality);
    enc->setProfile(encoderConfig.profile);
    if(encoderConfig.profile != dai::VideoEncoderProperties::Profile::MJPEG) {
        enc->setBitrate(encoderConfig.bitrate);
        enc->setKeyframeFrequency(encoderConfig.frameFreq);
    }
    return enc;
}
void ImagePublisher::createInfoManager(std::shared_ptr<dai::Device> device) {
    infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
        node->create_sub_node(std::string(node->get_name()) + "/" + pubConfig.daiNodeName).get(), "/" + pubConfig.daiNodeName + pubConfig.infoMgrSuffix);
    if(pubConfig.calibrationFile.empty()) {
        auto calHandler = device->readCalibration();
        auto info = sensor_helpers::getCalibInfo(node->get_logger(), converter, calHandler, pubConfig.socket, pubConfig.width, pubConfig.height);
        if(pubConfig.rectified) {
            std::fill(info.d.begin(), info.d.end(), 0.0);
            info.r[0] = info.r[4] = info.r[8] = 1.0;
        }
        infoManager->setCameraInfo(info);
    } else {
        infoManager->loadCameraInfo(pubConfig.calibrationFile);
    }
};
ImagePublisher::~ImagePublisher() {
    closeQueue();
};

void ImagePublisher::closeQueue() {
    if(dataQ && !dataQ->isClosed()) {
        dataQ->removeCallback(cbID);
        dataQ->close();
    }
}
void ImagePublisher::link(dai::Node::Input& in) {
    out->link(in);
}
std::shared_ptr<dai::MessageQueue> ImagePublisher::getQueue() {
    return dataQ;
}
bool ImagePublisher::isSynced() {
    return synced;
}
void ImagePublisher::addQueueCB() {
    cbID = dataQ->addCallback([this](const std::shared_ptr<dai::ADatatype>& data) { publish(data); });
}

std::string ImagePublisher::getQueueName() {
    return qName;
}
std::shared_ptr<Image> ImagePublisher::convertData(const std::shared_ptr<dai::ADatatype>& data) {
    sensor_msgs::msg::CameraInfo info;
    auto img = std::make_shared<Image>();
    if(encConfig.enabled) {
        auto daiImg = std::dynamic_pointer_cast<dai::EncodedFrame>(data);
        if(pubConfig.calibrationFile.empty()) {
            info = converter->generateCameraInfo(daiImg);
        } else {
            info = infoManager->getCameraInfo();
        }
        if(pubConfig.publishCompressed) {
            auto rawMsg = converter->toRosMsgRawPtr(daiImg, info);
            info.header = rawMsg.header;
            if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
                std::deque<sensor_msgs::msg::CompressedImage> deq;
                converter->toRosCompressedMsg(daiImg, deq);
                img->compressedImg = std::make_unique<sensor_msgs::msg::CompressedImage>(deq.front());
            } else {
                std::deque<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> deq;
                converter->toRosFFMPEGPacket(daiImg, deq);
                img->ffmpegPacket = std::make_unique<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(deq.front());
            }
        } else {
            auto rawMsg = converter->toRosMsgRawPtr(daiImg, info);
            info.header = rawMsg.header;
            sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>(rawMsg);
            img->image = std::move(msg);
        }
    } else {
        auto daiImg = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        if(pubConfig.calibrationFile.empty()) {
            info = converter->generateCameraInfo(daiImg);
        } else {
            info = infoManager->getCameraInfo();
        }
        auto rawMsg = converter->toRosMsgRawPtr(daiImg, info);
        info.header = rawMsg.header;
        sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>(rawMsg);
        img->image = std::move(msg);
    }
    if(pubConfig.rectified) {
        info.r[0] = info.r[4] = info.r[8] = 1.0;
    }
    if(pubConfig.undistorted) {
        std::fill(info.d.begin(), info.d.end(), 0.0);
    }
    sensor_msgs::msg::CameraInfo::UniquePtr infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(info);
    img->info = std::move(infoMsg);
    return img;
}
std::string ImagePublisher::upscaledTopicName() const {
    return pubConfig.topicName + "/upscaled";
}

bool ImagePublisher::upscaleNeeded() {
    if(pubConfig.hostSideUpscale <= 0.0) {
        return false;
    }
    if(!pubConfig.lazyPub) {
        return true;
    }
    if(upscaledPubIT.getNumSubscribers() > 0) {
        return true;
    }
#ifdef NOMAGIC_ROS1
    if(ros1UpscaledPub && ros1UpscaledPub->hasSubscribers()) {
        return true;
    }
#endif
    return false;
}

std::shared_ptr<Image> ImagePublisher::upscaleImage(const Image& img) const {
    const auto& src = *img.image;
    cv::Mat srcMat(static_cast<int>(src.height),
                   static_cast<int>(src.width),
                   cv_bridge::getCvType(src.encoding),
                   const_cast<uint8_t*>(src.data.data()),
                   static_cast<size_t>(src.step));
    cv::Mat dstMat;
    // INTER_NEAREST: any interpolating mode would blend the 0 = "no depth" pixels with their
    // neighbours and invent depth values along object edges.
    cv::resize(srcMat, dstMat, cv::Size(), pubConfig.hostSideUpscale, pubConfig.hostSideUpscale, cv::INTER_NEAREST);

    auto scaled = std::make_shared<Image>();
    auto image = std::make_unique<sensor_msgs::msg::Image>();
    image->header = src.header;
    image->height = static_cast<uint32_t>(dstMat.rows);
    image->width = static_cast<uint32_t>(dstMat.cols);
    image->encoding = src.encoding;
    image->is_bigendian = src.is_bigendian;
    image->step = static_cast<uint32_t>(dstMat.step);
    image->data.assign(dstMat.datastart, dstMat.dataend);

    // Same intrinsics scaling as image_proc/resize, so the upscaled pair stays usable for
    // projection: focal lengths and principal point follow the raster, the rest is unchanged.
    const double scaleX = static_cast<double>(dstMat.cols) / static_cast<double>(src.width);
    const double scaleY = static_cast<double>(dstMat.rows) / static_cast<double>(src.height);
    auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(*img.info);
    info->width = image->width;
    info->height = image->height;
    info->k[0] *= scaleX;  // fx
    info->k[2] *= scaleX;  // cx
    info->k[4] *= scaleY;  // fy
    info->k[5] *= scaleY;  // cy
    info->p[0] *= scaleX;  // fx
    info->p[2] *= scaleX;  // cx
    info->p[3] *= scaleX;  // Tx
    info->p[5] *= scaleY;  // fy
    info->p[6] *= scaleY;  // cy
    info->roi.x_offset = static_cast<uint32_t>(info->roi.x_offset * scaleX);
    info->roi.y_offset = static_cast<uint32_t>(info->roi.y_offset * scaleY);
    info->roi.width = static_cast<uint32_t>(info->roi.width * scaleX);
    info->roi.height = static_cast<uint32_t>(info->roi.height * scaleY);

    scaled->image = std::move(image);
    scaled->info = std::move(info);
    return scaled;
}

void ImagePublisher::publishUpscaled(std::shared_ptr<Image> img) {
#ifdef NOMAGIC_ROS1
    // NoMagic: publish on ROS1 first (the ROS2 path may move the data out).
    if(ros1UpscaledPub && (!pubConfig.lazyPub || ros1UpscaledPub->hasSubscribers())) {
        ros1UpscaledPub->publish(*img->image, *img->info);
    }
#endif
    if(!pubConfig.lazyPub || upscaledPubIT.getNumSubscribers() > 0) {
        if(ipcEnabled) {
            upscaledPubIT.publish(std::move(img->image), std::move(img->info));
        } else {
            upscaledPubIT.publish(*img->image, *img->info);
        }
    }
}

void ImagePublisher::publish(std::shared_ptr<Image> img) {
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            compressedImgPub->publish(std::move(img->compressedImg));
        } else {
            ffmpegPub->publish(std::move(img->ffmpegPacket));
        }
        infoPub->publish(std::move(img->info));
    } else {
        // Resize before the publishes below: with intra-process comms they move the message out.
        std::shared_ptr<Image> upscaled;
        if(img->image && upscaleNeeded()) {
            upscaled = upscaleImage(*img);
        }
#ifdef NOMAGIC_ROS1
        // NoMagic: publish on ROS1 first (the ROS2 path may move the data out).
        // Lazy publishing is evaluated per graph.
        if(ros1Pub && (!pubConfig.lazyPub || ros1Pub->hasSubscribers())) {
            ros1Pub->publish(*img->image, *img->info);
        }
#endif
        if(!pubConfig.lazyPub || imgPubIT.getNumSubscribers() > 0) {
            if(ipcEnabled) {
                imgPubIT.publish(std::move(img->image), std::move(img->info));
            } else {
                imgPubIT.publish(*img->image, *img->info);
            }
        }
        if(upscaled) {
            publishUpscaled(std::move(upscaled));
        }
    }
}
void ImagePublisher::publish(std::shared_ptr<Image> img, rclcpp::Time timestamp) {
    img->info->header.stamp = timestamp;
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            img->compressedImg->header.stamp = timestamp;
        } else {
            img->ffmpegPacket->header.stamp = timestamp;
        }
    } else {
        img->image->header.stamp = timestamp;
    }
    publish(img);
}

void ImagePublisher::publish(const std::shared_ptr<dai::ADatatype>& data) {
    if(rclcpp::ok()) {
        auto img = convertData(data);
        publish(img);
    }
}
}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
