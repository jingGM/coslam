//
// Created by jing on 4/4/21.
//

#include "dataInterface.h"

DataInterface::DataInterface(const int& cameraNumber) {
    cameraNum = cameraNumber;

    for(int i=0; i<cameraNum; i++) {
        sensor_msgs::Image singleImage;
        surveillanceImages.push_back(singleImage);
        globalTimeStamp.push_back(0);
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
    timeStamp = data.header.stamp.toNSec();
}

void DataInterface::robotIMUCallback(const sensor_msgs::Imu &data) {
    imu = data;
}

void DataInterface::robotOdomCallback(const nav_msgs::Odometry &data) {
    odom = data;
}

void DataInterface::robotSurveillanceCallback(int i, const boost::shared_ptr<sensor_msgs::Image const>& data) {
    surveillanceImages[i] = *data;
    globalTimeStamp[i] = data->header.stamp.toNSec();
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

uint64_t DataInterface::getTimeStamp() const {
//    return ros::Time::now().nsec;
    return timeStamp;
}

std::vector<double> DataInterface::getOdom() {
    return {odom.twist.twist.linear.x, odom.twist.twist.angular.z};
}

std::vector<double> DataInterface::getImu() {
    double heading = std::atan2(2*imu.orientation.y*imu.orientation.w-2*imu.orientation.x*imu.orientation.z ,
                           1 - 2*imu.orientation.y*imu.orientation.y - 2*imu.orientation.z*imu.orientation.z);
    double attitude = std::asin(2*imu.orientation.x*imu.orientation.y + 2*imu.orientation.z*imu.orientation.w);
    double bank = std::atan2(2*imu.orientation.x*imu.orientation.w-2*imu.orientation.y*imu.orientation.z ,
                 1 - 2*imu.orientation.x*imu.orientation.x - 2*imu.orientation.z*imu.orientation.z);

    return {heading, attitude, bank,
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z};
}


bool DataInterface::dataReady() const {
    auto rate = ros::Rate(10);
    bool cameraTag = false;
    if (cameraNum > 0) {
        for (int i=0;i<cameraNum;i++)
            cameraTag |= surveillanceImages[i].header.seq == 0;
    }
    while (rgb.header.seq==0 or depth.header.seq==0 or odom.header.seq==0 or imu.header.seq==0 or cameraTag) {
        if (cameraNum > 0) {
            cameraTag = false;
            for (int i=0;i<cameraNum;i++) {
                cameraTag |= surveillanceImages[i].header.seq == 0;
            }
        }
        std::cout<<"waiting for data "<<std::endl;
        rate.sleep();
    }

    transR.waitForTransform(fNames.robotTargetFrame, fNames.robotSourceFrame, ros::Time(), ros::Duration(2.0));
    for (int i=0; i<cameraNum;i++)
        transG.waitForTransform(fNames.globalTargetFrame[i], fNames.globalSourceFrame[i], ros::Time(), ros::Duration(2.0));
}

cv::Mat DataInterface::getSurveillanceRGB(int index) {
    if (surveillanceImages[index].height==0) return cv::Mat();
    else {
        sensor_msgs::Image rgbTMP = surveillanceImages[index];
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

const unsigned char *DataInterface::getSurveillanceRGBPtr(int index) {
    if (surveillanceImages[index].height==0) return nullptr;
    else {
        sensor_msgs::Image rgbTMP = surveillanceImages[index];
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

uint64_t DataInterface::getSurveillanceTimeStamp(int index) const {
    return globalTimeStamp[index];
}

std::vector<double> DataInterface::getCamTransform() {
    double rR, rP, rY;
    tf::StampedTransform robotTransform;

    transR.lookupTransform(fNames.robotTargetFrame, fNames.robotSourceFrame, ros::Time(), robotTransform);

    tf::Vector3 vR = robotTransform.getOrigin();

    robotTransform.getBasis().getRPY(rR, rP, rY);
    return {vR.getX(), vR.getY(), vR.getZ(), rR, rP, rY};
}

std::vector<double> DataInterface::getSurveillanceTransform(int index) {
    double gR, gP, gY;
    tf::StampedTransform globalTransform;
    transG.lookupTransform(fNames.globalTargetFrame[index], fNames.globalSourceFrame[index], ros::Time(), globalTransform);
    tf::Vector3 vG = globalTransform.getOrigin();
    globalTransform.getBasis().getRPY(gR, gP, gY);
    return {vG.getX(), vG.getY(), vG.getZ(), gR, gP, gY};
}

