//
// Created by jing on 4/4/21.
//

#ifndef COSLAM_DATAINTERFACE_H
#define COSLAM_DATAINTERFACE_H

//#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/CameraInfo.h"
#include "cv_bridge/cv_bridge.h"

class DataInterface {
private:
    sensor_msgs::Image rgb;
    sensor_msgs::Image depth;
    sensor_msgs::CameraInfo rgbCameraInfo;
    nav_msgs::Odometry odom;

    int cameraNum = 0;
    std::vector<sensor_msgs::Image> surveillanceImages={};

public:
    explicit DataInterface(const int& cameraNumber);
    void robotDepthCallback(const sensor_msgs::Image& data);
    void robotRGBCallback(const sensor_msgs::Image& data);
    void robotCameraInfo(const sensor_msgs::CameraInfo & data);
    void robotOdomCallback(const nav_msgs::Odometry & data);

    void robotSurveillanceCallback(int i, const boost::shared_ptr<sensor_msgs::Image const>& data);

    cv::Mat getRGB();
    cv::Mat getDepth();
    sensor_msgs::CameraInfo getRGBCameraInfo();

};


typedef std::shared_ptr<DataInterface> dataInterfacePtr;
#endif //COSLAM_DATAINTERFACE_H
