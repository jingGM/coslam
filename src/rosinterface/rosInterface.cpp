//
// Created by jing on 4/5/21.
//

#include "rosInterface.h"

#include <utility>

rosInterface::rosInterface(dataInterfacePtr  dataIfPtr,const int &surveillanceNumber):dataInferPtr(dataIfPtr) {
    subDepth = n.subscribe(topicNames.depthImageTopic, 1000, &DataInterface::robotDepthCallback, dataInferPtr.get());
    subRGB = n.subscribe(topicNames.rgbImageTopic, 1000, &DataInterface::robotRGBCallback, dataInferPtr.get());
    subRGBCameraInfo = n.subscribe(topicNames.rgbCameraInfoTopic, 1000, &DataInterface::robotCameraInfo, dataInferPtr.get());
    subOdom = n.subscribe(topicNames.odometryTopic, 1000, &DataInterface::robotOdomCallback, dataInferPtr.get());
    subImu = n.subscribe(topicNames.imuTopic, 1000, &DataInterface::robotIMUCallback, dataInferPtr.get());

    topicNames.setSurveillanceCameras(surveillanceNumber);
    for(int i=0; i< surveillanceNumber; i++) {
        auto sub = n.subscribe<sensor_msgs::Image>(topicNames.surveillanceTopics[i], 1000,
                                             boost::bind(&DataInterface::robotSurveillanceCallback, dataInferPtr, i, _1));
        surveillanceSubscribers.push_back(sub);
    }
    rosThread = std::thread(&rosInterface::ros_total_spin, this);


}


rosInterface::~rosInterface() {
    rosThread.join();
}

