//
// Created by jing on 4/5/21.
//

#include <memory>
#include "rosInterface.h"
#include "dataInterface.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_interface");


    dataInterfacePtr dataInferPtr = std::make_shared<DataInterface>(4);
    rosInterface rosInter(dataInferPtr,4);

    dataInferPtr->getDepth();

    return 0;
}
