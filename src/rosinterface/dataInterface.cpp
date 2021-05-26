//
// Created by jing on 4/4/21.
//

#include "dataInterface.h"

DataInterface::DataInterface(const int& cameraNumber) {
    cameraNum = cameraNumber;

    for(int i=0; i<cameraNum; i++) {
        sensor_msgs::Image singleImage;
        surveillanceImages.push_back(singleImage);
    }
}

void DataInterface::robotDepthCallback(const sensor_msgs::Image& data) {
    depth = data;
}

void DataInterface::robotRGBCallback(const sensor_msgs::Image& data) {
    rgb = data;
}

void DataInterface::robotCameraInfo(const sensor_msgs::CameraInfo &data) {
    rgbCameraInfo = data;
}

void DataInterface::robotOdomCallback(const nav_msgs::Odometry &data) {
    odom = data;
}

void DataInterface::robotSurveillanceCallback(int i, const boost::shared_ptr<sensor_msgs::Image const>& data) {

    surveillanceImages[i] = *data;
}

sensor_msgs::CameraInfo DataInterface::getRGBCameraInfo() {
    return rgbCameraInfo;
}

cv::Mat DataInterface::getRGB() {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
        return cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

cv::Mat DataInterface::getDepth() {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
        return cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
