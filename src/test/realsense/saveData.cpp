//
// Created by jing on 9/26/21.
//

#include <iostream>
#include <memory>
#include <fstream>
#include <zconf.h>
#include <opencv2/highgui.hpp>

#include "DataCollection.h"

void saveFile(bool onece,
              const DataCollectionPtr& collectorPtr,
              std::string filename="rosrecord",
              int timeLimit=70) {
    auto depthSize = collectorPtr->depthSize, rgbSize = collectorPtr->rgbSize;
    Bytef * decompressionBufferDepth;
    Bytef * decompressionBufferImage;
    decompressionBufferDepth = new Bytef[depthSize];
    decompressionBufferImage = new Bytef[rgbSize];

    filename.append(".ply");
    remove(filename.c_str());

    std::ofstream fpout (filename.c_str (), std::ios::app | std::ios::binary);

    ros::Rate r(10);
    auto startTime = ros::Time::now().toSec();
    while(not ros::isShuttingDown()) {
        int64_t frameTime = collectorPtr->getTimeStamp();
        fpout.write (reinterpret_cast<const char*> (&frameTime), sizeof (int64_t));


        memcpy(&decompressionBufferImage[0], collectorPtr->getRGB().data, rgbSize);
        fpout.write (reinterpret_cast<const char*> (&rgbSize), sizeof(int32_t));
        fpout.write (reinterpret_cast<const char*> (decompressionBufferImage), rgbSize);

        memcpy(&decompressionBufferDepth[0], collectorPtr->getDepth().data, depthSize);
        fpout.write (reinterpret_cast<const char*> (&depthSize), sizeof(int32_t));
        fpout.write (reinterpret_cast<const char*> (decompressionBufferDepth), depthSize);

        std::cout<<ros::Time::now().toSec() - startTime<<std::endl;
        if (onece) break;

        r.sleep();

        if (ros::Time::now().toSec() - startTime > timeLimit) break;
    }

    std::cout<<"stoped"<<std::endl;
    fpout.close ();
    collectorPtr->~DataCollection();
}

void testTime(const DataCollectionPtr& collectorPtr, bool testSpeed) {
    ros::Rate r(10);
    auto startTime = ros::Time::now();
    while(not ros::isShuttingDown()) {
        auto rgbTime = collectorPtr->colorTimeStamp;
        auto depthTime = collectorPtr->depthTimeStamp;
        auto difference = std::abs(long(rgbTime.toNSec() - depthTime.toNSec()));

        if (testSpeed) {
            auto rgbPtr = collectorPtr->getRGBPtr();
            auto depthPtr = collectorPtr->getDepthPtr();
            std::cout<< collectorPtr->colorTimeStamp.toNSec() - startTime.toNSec()<<"/"<<collectorPtr->colorTimeStamp.toSec() - startTime.toSec()  << std::endl;
            startTime = collectorPtr->colorTimeStamp;
        } else {
            if (difference > 0.1)
                std::cout<< difference<<": "<<rgbTime << "/" << depthTime<< std::endl;
            else std::cout<< difference<< std::endl;
            r.sleep();
        }
    }

}

void showImages(std::vector<int> rgbSize, std::vector<int> depthSize, std::string filename="rosrecord") {
    filename.append(".ply");
    std::ifstream recordFile;
    recordFile.open(filename.c_str());

    Bytef * decompressionBufferDepth;
    Bytef * decompressionBufferImage;
    decompressionBufferDepth = new Bytef[depthSize[0]*depthSize[1]*2];
    decompressionBufferImage = new Bytef[rgbSize[0]*rgbSize[1]*3];

    while (recordFile.peek()!=EOF) {
        cv::Mat rgb(rgbSize[0], rgbSize[1], CV_8UC3);
        cv::Mat depth(depthSize[0], depthSize[1],CV_16UC1);
        int64_t showTime;
        recordFile.read(reinterpret_cast<char *>(&showTime), sizeof(int64_t));
        std::cout<< showTime<<std::endl;

        recordFile.read(reinterpret_cast<char *>(&decompressionBufferImage[0]), rgbSize[0]*rgbSize[1]*3);
        memcpy(rgb.data,&decompressionBufferImage[0],rgbSize[0]*rgbSize[1]*3);
        cv::namedWindow("rgb",cv::WINDOW_AUTOSIZE);
        cv::imshow("rgb",rgb);
        cv::waitKey(0);

        recordFile.read(reinterpret_cast<char *>(&decompressionBufferDepth[0]), depthSize[0]*depthSize[1]*2);
        memcpy(depth.data,&decompressionBufferDepth[0],depthSize[0]*depthSize[1]*2);
        cv::namedWindow("depth",cv::WINDOW_AUTOSIZE);
        cv::imshow("depth",depth);
        cv::waitKey(0);
    }
    recordFile.close();
}

void testStream(DataCollectionPtr collectorPtr) {
    auto rgb= collectorPtr->getRGB();
    auto depth = collectorPtr->getDepth();

    Bytef * decompressionBufferDepth;
    Bytef * decompressionBufferImage;
    decompressionBufferDepth = new Bytef[collectorPtr->depthSize];
    decompressionBufferImage = new Bytef[collectorPtr->rgbSize];

    Bytef * newDecompressionBufferDepth;
    Bytef * newDecompressionBufferImage;
    newDecompressionBufferDepth = new Bytef[collectorPtr->depthSize];
    newDecompressionBufferImage = new Bytef[collectorPtr->rgbSize];

    std::string filename="rosrecord";
    filename.append(".ply");
    remove(filename.c_str());
    std::ofstream fpout (filename.c_str (), std::ios::app | std::ios::binary);

//    memcpy(&decompressionBufferImage[0], rgb.data, collectorPtr->rgbSize);
//    fpout.write (reinterpret_cast<const char*> (decompressionBufferImage), collectorPtr->rgbSize);

    memcpy(&decompressionBufferDepth[0], depth.data, collectorPtr->depthSize);
    fpout.write (reinterpret_cast<const char*> (decompressionBufferDepth), collectorPtr->depthSize);
    fpout.close();

    std::ifstream recordFile;
    recordFile.open(filename.c_str());

//    recordFile.read(reinterpret_cast<char *>(&newDecompressionBufferImage[0]), collectorPtr->rgbSize);
//    cv::Mat newrgb(collectorPtr->rgbSizeV[0],collectorPtr->rgbSizeV[1],CV_8UC3);
//    memcpy(newrgb.data, &newDecompressionBufferImage[0], collectorPtr->rgbSize);
//    cv::namedWindow("rgb",cv::WINDOW_AUTOSIZE);
//    cv::imshow("rgb",newrgb);
//    cv::waitKey(0);

    recordFile.read(reinterpret_cast<char *>(&newDecompressionBufferDepth[0]), collectorPtr->depthSize);

    for (int i=0; i<collectorPtr->depthSize; i++) {
        if (newDecompressionBufferDepth[i] != decompressionBufferDepth[i])
            std::cout<<i<<": "<<newDecompressionBufferDepth[i]<<"/"<<decompressionBufferDepth[i]<<std::endl;
    }

    cv::Mat newdepth(collectorPtr->depthSizeV[0],collectorPtr->depthSizeV[1],CV_16UC1);
    memcpy(newdepth.data, &newDecompressionBufferDepth[0], collectorPtr->depthSize);
    cv::namedWindow("depth",cv::WINDOW_AUTOSIZE);
    cv::imshow("depth",newdepth);
    cv::waitKey(0);

    recordFile.close();

    delete[] decompressionBufferDepth;
    delete[] decompressionBufferImage;
    delete[] newDecompressionBufferDepth;
    delete[] newDecompressionBufferImage;
}

int main(int argc, char *argv[])
{
//    ros::init(argc, argv, "realsense_collector");
//    auto collectorPtr = std::make_shared<DataCollection>();;
//    collectorPtr->dataReady();
//
//    testStream(collectorPtr);
//    saveFile(false, collectorPtr, "walkingfiles");
//    testTime(collectorPtr, true);
    showImages({480,640},{480,640}, "walkingfile");

    return 0;
}