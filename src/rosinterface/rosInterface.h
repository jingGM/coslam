//
// Created by jing on 4/5/21.
//

#ifndef COSLAM_ROSINTERFACE_H
#define COSLAM_ROSINTERFACE_H

#include "ros/ros.h"
#include <sstream>
#include<string>
#include <thread>

#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/CameraInfo.h"
#include <boost/tokenizer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "dataInterface.h"

struct TopicNames {
    std::string rgbImageTopic = "/jackal/camera/image_raw";
    std::string rgbCameraInfoTopic = "/jackal/camera/camera_info";
    std::string depthImageTopic = "/jackal/camera/depth/image_raw";
    std::string odometryTopic = "/jackal/jackal_velocity_controller/odom";
    std::string actionTopic = "/jackal/jackal_velocity_controller/cmd_vel";

    std::vector<std::string> surveillanceTopics = {};

    std::string testTopic = "testing/image_raw";
    std::string testTopic1 = "testing/camera_info";

    void setSurveillanceCameras(const int& number) {
        for(int i=0; i<number; i++) {
            surveillanceTopics.push_back("/rgb"+ std::to_string(i) + "/camera/image_raw");
        }
    }
};



class rosInterface {
private:
    ros::NodeHandle n;
    TopicNames topicNames;

    dataInterfacePtr dataInferPtr;

    int cameraNum = 0;
    void ros_total_spin() {ros::spin();}

    ros::Subscriber subDepth, subRGB, subOdom, subRGBCameraInfo;
    std::vector<ros::Subscriber> surveillanceSubscribers={};

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>(topicNames.actionTopic, 5);
//    ros::Publisher pub1 = n.advertise<sensor_msgs::Image>(topicNames.testTopic, 5);
    std::thread rosThread;

public:
    explicit rosInterface(dataInterfacePtr  dataIfPtr, const int& surveillanceNumber);
    ~rosInterface();
};

typedef std::shared_ptr<rosInterface> rosInterfacePtr;

#endif //COSLAM_ROSINTERFACE_H
