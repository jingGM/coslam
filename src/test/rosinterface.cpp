//
// Created by jing on 8/19/21.
//

#include <iostream>
#include "../Tools/ROSLogReader.h"
#include "../Tools/rosinterface/rosInterface.h"
#include "../Tools/rosinterface/dataInterface.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_efusion");
    auto dataInferPtr = std::make_shared<DataInterface>(1);
    auto rosInterPtr = std::make_shared<rosInterface>(dataInferPtr, 1);
    LogReaderPtr logReader = std::make_shared<ROSLogReader>("", false, true, dataInferPtr, 100);

    while (not ros::isShuttingDown()) {
        logReader->getNext();
        auto rgb = logReader->rgb;
        auto depth = logReader->depth;
        auto global = logReader->globalRGB[0];


    }
    return 0;
}

