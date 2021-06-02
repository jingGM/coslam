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
    if (rgb.height==0) return cv::Mat();
    else {
        sensor_msgs::Image rgbTMP = rgb;
        try
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(rgbTMP, sensor_msgs::image_encodings::BGR8);
            return cv_ptr->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
}

const unsigned char *DataInterface::getRGBPtr() {
    if (rgb.height==0) return nullptr;
    else {
        sensor_msgs::Image rgbTMP = rgb;
        try
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(rgbTMP, sensor_msgs::image_encodings::RGB8);
            unsigned char * tmp_ptr = cv_ptr->image.data;
            return tmp_ptr;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        return nullptr;
    }
}

const unsigned char *DataInterface::getDepthPtr() {
    if(rgb.height==0) return nullptr;
    else{
        sensor_msgs::Image depthTMP = depth;
        try
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(depthTMP, sensor_msgs::image_encodings::TYPE_16UC1);

//            TODO: this one doesn't work
            int i = 0;
            for (int r=0;r<cv_ptr->image.rows;r++) {
                for (int c=0;c<cv_ptr->image.cols; c++) {
                    depthPixels[i+1] = cv_ptr->image.at<unsigned short>(r,c);
                    i++;i++;
                }
            }
            return depthPixels;

//            TODO: this one works a little, but not right
//            auto * tmp_ptr = reinterpret_cast<unsigned short *>(cv_ptr->image.data);
//            return tmp_ptr;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
}

cv::Mat DataInterface::getDepth() {
    if(rgb.height==0) return cv::Mat();
    else{
        sensor_msgs::Image depthTMP = depth;
        try
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(depthTMP, sensor_msgs::image_encodings::TYPE_16UC1);
            return cv_ptr->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
}

unsigned int DataInterface::getTimeStamp() {
    return ros::Time::now().nsec;
}

std::vector<double> DataInterface::getOdom() {
    return {odom.twist.twist.linear.x, odom.twist.twist.angular.z};
}

bool DataInterface::dataReady() const {
    auto rate = ros::Rate(10);
    while (rgb.height==0 or depth.height==0 or odom.header.seq==0) {
        std::cout<<"waiting for data "<<std::endl;
        rate.sleep();
    }
}
