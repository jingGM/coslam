//
// Created by jing on 9/26/21.
//

#include "DataCollection.h"

DataCollection::DataCollection() {
    subDepth = n.subscribe(topicNames.depthImageTopic, 1000, &DataCollection::depthCallback, this);
    subRGB = n.subscribe(topicNames.rgbImageTopic, 1000, &DataCollection::RGBCallback, this);
    subRGBCameraInfo = n.subscribe(topicNames.rgbCameraInfoTopic, 1000, &DataCollection::colorCameraInfoCallback, this);
    subDepthCameraInfo = n.subscribe(topicNames.rgbCameraInfoTopic, 1000, &DataCollection::depthCameraInfoCallback, this);

    rosThread = std::thread(&DataCollection::ros_total_spin, this);
}

DataCollection::~DataCollection() {
    rosThread.join();
    delete [] depthPixels;
    delete [] rgbPixels;
}

void DataCollection::depthCallback(const sensor_msgs::Image& data) {
    depth = data;
    depthTimeStamp = data.header.stamp;
}

void DataCollection::RGBCallback(const sensor_msgs::Image& data) {
    rgb = data;
    colorTimeStamp = data.header.stamp;
}

void DataCollection::colorCameraInfoCallback(const sensor_msgs::CameraInfo &data) {
    rgbCameraInfo = data;
}

void DataCollection::depthCameraInfoCallback(const sensor_msgs::CameraInfo &data) {
    depthCameraInfo = data;
}

cv::Mat DataCollection::getRGB() const {
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

const unsigned char *DataCollection::getRGBPtr() {
    if (rgb.height==0) return nullptr;
    else {
        sensor_msgs::Image rgbTMP = rgb;
        try
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(rgbTMP, sensor_msgs::image_encodings::RGB8);

            memcpy(rgbPixels, cv_ptr->image.data, rgbSize);
            return rgbPixels;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        return nullptr;
    }
}

cv::Mat DataCollection::getDepth() const {
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

const unsigned char *DataCollection::getDepthPtr() {
    if(rgb.height==0) return nullptr;
    else{
        if (!depthPixels) return nullptr;
        sensor_msgs::Image depthTMP = depth;
        try
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(depthTMP, sensor_msgs::image_encodings::TYPE_16UC1);

            int i = 0;
            for (int r=0;r<cv_ptr->image.rows;r++) {
                for (int c=0;c<cv_ptr->image.cols; c++) {
                    depthPixels[i+1] = cv_ptr->image.at<unsigned short>(r,c);
                    i++;i++;
                }
            }
            return depthPixels;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
}

sensor_msgs::CameraInfo DataCollection::getRGBCameraInfo() const {
    return rgbCameraInfo;
}

sensor_msgs::CameraInfo DataCollection::getDepthCameraInfo() const {
    return depthCameraInfo;
}

uint64_t DataCollection::getTimeStamp() const {
    return colorTimeStamp.toNSec();
}

bool DataCollection::dataReady() {
    auto rate = ros::Rate(10);
    bool cameraTag = false;
    while (rgb.header.seq==0 or depth.header.seq==0 or rgbCameraInfo.header.seq==0 or depthCameraInfo.header.seq==0) {
        std::cout<<"waiting for data "<<std::endl;
        rate.sleep();
    }

    cv::Mat rgbMat = this->getRGB();
    rgbSizeV = {rgbMat.rows, rgbMat.cols};
    rgbSize = rgbMat.rows*rgbMat.cols*3;
    rgbPixels = new unsigned char[rgbSize];

    cv::Mat depthMat = this->getDepth();
    depthSizeV = {depthMat.rows, depthMat.cols};
    depthSize = depthMat.rows*depthMat.cols*2;
    depthPixels = new unsigned char[depthSize];
}

