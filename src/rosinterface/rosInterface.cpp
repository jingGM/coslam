//
// Created by jing on 4/5/21.
//

#include "rosInterface.h"

rosInterface::rosInterface(const dataInterfacePtr& dataIfPtr,const int &surveillanceNumber) {
    subDepth = n.subscribe(topicNames.depthImageTopic, 1000, &DataInterface::robotDepthCallback, dataIfPtr.get());
    subRGB = n.subscribe(topicNames.rgbImageTopic, 1000, &DataInterface::robotRGBCallback, dataIfPtr.get());
    subRGBCameraInfo = n.subscribe(topicNames.rgbCameraInfoTopic, 1000, &DataInterface::robotCameraInfo, dataIfPtr.get());
    subOdom = n.subscribe(topicNames.odometryTopic, 1000, &DataInterface::robotOdomCallback, dataIfPtr.get());

    topicNames.setSurveillanceCameras(surveillanceNumber);
    for(int i=0; i< surveillanceNumber; i++) {
        auto sub = n.subscribe<sensor_msgs::Image>(topicNames.surveillanceTopics[i], 1000,
                                             boost::bind(&DataInterface::robotSurveillanceCallback, dataIfPtr, i, _1));
        surveillanceSubscribers.push_back(sub);
    }
}

