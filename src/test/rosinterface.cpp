//
// Created by jing on 5/26/21.
//

#include <opencv2/highgui.hpp>
#include "../rosinterface/dataInterface.h"
#include "../rosinterface/rosInterface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_interface");

    dataInterfacePtr dataInferPtr = std::make_shared<DataInterface>(4);
    rosInterface rosInter(dataInferPtr,4);

    dataInferPtr->dataReady();
    ros::Rate rate(10);
    while(ros::ok())
    {
        auto rgb = dataInferPtr->getRGB();
        cv::imwrite("rgb.jpg", rgb);

//        auto rgbRead = cv::imread("rgb.jpg",cv::IMREAD_COLOR);
//        cv::imwrite("rgb_copy.jpg", rgb);
//
//        auto rgbPtr = dataInferPtr->getRGBPtr();


        auto depth = dataInferPtr->getDepth();
        cv::imwrite("depth.jpg", depth);

        auto depthPtr = dataInferPtr->getDepthPtr();
//        std::cout<<"main"<<std::endl;

        rate.sleep();
    }

    return 0;
}
