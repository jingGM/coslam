//
// Created by jing on 4/4/21.
//

#ifndef COSLAM_DATAINTERFACE_H
#define COSLAM_DATAINTERFACE_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/CameraInfo.h"
#include "cv_bridge/cv_bridge.h"

#include <zconf.h>

class DataInterface {
private:
    sensor_msgs::Image rgb;
    sensor_msgs::Image depth;
    sensor_msgs::CameraInfo rgbCameraInfo;
    sensor_msgs::Imu imu;
    nav_msgs::Odometry odom;
    uint64_t timeStamp;

    int cameraNum = 0;
    std::vector<sensor_msgs::Image> surveillanceImages={};
    std::vector<uint64_t> globalTimeStamp;
//    sensor_msgs::CameraInfo globalCameraInfo;
    unsigned char * depthPixels = new unsigned char[640*480*2];

public:
    explicit DataInterface(const int& cameraNumber);
    ~DataInterface() {delete depthPixels;};
    void robotDepthCallback(const sensor_msgs::Image& data);
    void robotRGBCallback(const sensor_msgs::Image& data);
    void robotCameraInfo(const sensor_msgs::CameraInfo & data);
    void robotIMUCallback(const sensor_msgs::Imu & data);
    void robotOdomCallback(const nav_msgs::Odometry & data);

    void robotSurveillanceCallback(int i, const boost::shared_ptr<sensor_msgs::Image const>& data);

    cv::Mat getRGB();
    const unsigned char * getRGBPtr();
    cv::Mat getDepth();
    const unsigned char * getDepthPtr();
    sensor_msgs::CameraInfo getRGBCameraInfo();
    uint64_t  getTimeStamp() const;
    std::vector<double> getOdom();
    std::vector<double> getImu();
    cv::Mat getSurveillanceRGB(int index);
    const unsigned char * getSurveillanceRGBPtr(int index);
    uint64_t getSurveillanceTimeStamp(int index) const;

    bool dataReady() const;
};


typedef std::shared_ptr<DataInterface> dataInterfacePtr;
#endif //COSLAM_DATAINTERFACE_H