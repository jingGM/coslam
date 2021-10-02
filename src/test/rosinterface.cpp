//
// Created by jing on 8/19/21.
//

#include <iostream>
#include "../Utils/LocalCameraInfo.h"
#include "../Utils/GlobalCamInfo.h"
#include "../Tools/ROSLogReader.h"
#include "../Tools/RawLogReader.h"
#include "../Tools/rosinterface/rosInterface.h"
#include "../Tools/rosinterface/dataInterface.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"
#include <zconf.h>
#include <opencv2/highgui.hpp>

void testROS(int argc, char *argv[]) {
    ros::init(argc, argv, "ros_efusion");
    auto dataInferPtr = std::make_shared<DataInterface>(1);
    auto rosInterPtr = std::make_shared<rosInterface>(dataInferPtr, 1);
    LogReaderPtr logReader = std::make_shared<ROSLogReader>("", false, true, false, dataInferPtr, 100);

    std::string robotSourceFrame = "odom", robotTargetFrame = "camera_depth_optical_frame";
    std::vector<std::string> globalSourceFrame = {"rgb0/camera_base"}, globalTargetFrame = {"rgb0/camera_optical_frame"};

    tf::TransformListener transR, transG;
    transR.waitForTransform(robotSourceFrame, robotTargetFrame, ros::Time(), ros::Duration(2.0));
    transG.waitForTransform(globalSourceFrame[0], globalTargetFrame[0], ros::Time(), ros::Duration(2.0));

    while (not ros::isShuttingDown()) {
        logReader->getNext();
        auto rgb = logReader->rgb;
        auto depth = logReader->depth;
        auto global = logReader->globalRGB[0];


        double rR, rP, rY, gR, gP, gY;
        tf::StampedTransform robotTransform, globalTransform;

        transR.lookupTransform(robotSourceFrame, robotTargetFrame, ros::Time(), robotTransform);
        transG.lookupTransform(globalSourceFrame[0], globalTargetFrame[0], ros::Time(), globalTransform);
        tf::Vector3 vR = robotTransform.getOrigin();
        tf::Vector3 vG = globalTransform.getOrigin();
        robotTransform.getBasis().getRPY(rR, rP, rY);
        robotTransform.getBasis().getRPY(gR, gP, gY);

        std::cout<< ros::Time::now().toSec()<< vR << vG<< std::endl;

    }
}

void showRGB(unsigned char * image) {
    cv::Mat newimage;
    newimage= cv::Mat(480,640,CV_8UC3);
    memcpy(newimage.data, image, 640*480*3);
    cv::namedWindow("rgb",cv::WINDOW_AUTOSIZE);
    cv::imshow("rgb",newimage);
    cv::waitKey(0);
}
void showDepth(unsigned short * image) {
    cv::Mat newimage;
    newimage= cv::Mat(480,640,CV_16UC1);
    memcpy(newimage.data, image, 640*480*2);
    cv::namedWindow("rgb",cv::WINDOW_AUTOSIZE);
    cv::imshow("rgb",newimage);
    cv::waitKey(0);
}

void testRawLogger() {
    std::string logFile = "/home/jing/Documents/catkinws/slam/src/coslam/dataset/realsense/";
    LogReaderPtr logReader = std::make_shared<RawLogReader>(logFile, false, true, true);

    logReader->getNext(false);
    for (int i=0;i<100; i++) {
        showRGB(logReader->rgb);
        showDepth(logReader->depth);
    }

}

int main(int argc, char *argv[])
{
    LocalCameraInfo::getInstance(528, 528, 320, 240, 640, 480);
    GlobalCamInfo::getInstance(640, 480, 457, 457, 320, 224, 1);

//    testROS(argc,argv);
    testRawLogger();
    return 0;
}

