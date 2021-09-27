//
// Created by jing on 9/26/21.
//

#ifndef COSLAM_DataCollection_H
#define COSLAM_DataCollection_H

#include <string>
#include <thread>

#include <boost/tokenizer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"

struct TopicNames {
    std::string rgbImageTopic = "/camera/color/image_raw";
    std::string rgbCameraInfoTopic = "/camera/color/camera_info";
    std::string depthImageTopic = "/camera/depth/image_rect_raw";
    std::string depthCameraInfoTopic = "/camera/depth/camera_info";
};

class DataCollection {
public:
    DataCollection();
    ~DataCollection();

    cv::Mat getRGB() const;
    const unsigned char * getRGBPtr();
    cv::Mat getDepth() const;
    const unsigned char * getDepthPtr();
    sensor_msgs::CameraInfo getRGBCameraInfo() const;
    sensor_msgs::CameraInfo getDepthCameraInfo() const;
    uint64_t  getTimeStamp() const;
    bool dataReady();

    sensor_msgs::Image rgb, depth;
    sensor_msgs::CameraInfo rgbCameraInfo, depthCameraInfo;
    ros::Time depthTimeStamp;
    ros::Time colorTimeStamp;

    std::vector<int> rgbSizeV, depthSizeV;
    size_t rgbSize, depthSize;

private:
    ros::NodeHandle n;
    TopicNames topicNames;
    ros::Subscriber subDepth, subRGB, subDepthCameraInfo, subRGBCameraInfo;
    std::thread rosThread;

    unsigned char * depthPixels;
    unsigned char * rgbPixels;

    void ros_total_spin() {ros::spin();}

    void depthCallback(const sensor_msgs::Image& data);
    void RGBCallback(const sensor_msgs::Image& data);
    void colorCameraInfoCallback(const sensor_msgs::CameraInfo & data);
    void depthCameraInfoCallback(const sensor_msgs::CameraInfo & data);
};

typedef std::shared_ptr<DataCollection> DataCollectionPtr;
#endif //COSLAM_DataCollection_H
