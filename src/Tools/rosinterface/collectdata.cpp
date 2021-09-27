//
// Created by jing on 9/20/21.
//
#include <memory>
#include <fstream>

#include "rosInterface.h"
#include "dataInterface.h"

#include "../../Utils/LocalCameraInfo.h"
#include "../../Utils/GlobalCamInfo.h"

void collectData(int argc, char *argv[], bool oneTimeGlobal) {

    ros::init(argc, argv, "ros_efusion");
    auto dataInferPtr = std::make_shared<DataInterface>(1);
    auto rosInterPtr = std::make_shared<rosInterface>(dataInferPtr, 1);
    dataInferPtr->dataReady();

    Bytef * decompressionBufferDepth;
    Bytef * decompressionBufferImage;
    decompressionBufferDepth = new Bytef[LocalCameraInfo::getInstance().numPixels() * 2];
    decompressionBufferImage = new Bytef[LocalCameraInfo::getInstance().numPixels() * 3];

    std::vector<Bytef *> decompressionBufferGlobalImages = {};
    decompressionBufferGlobalImages.push_back(new Bytef[GlobalCamInfo::getInstance().numPixels() * 3]);

    const std::string file;
    std::string filename = "rosrecord";
    filename.append(".ply");
    remove(filename.c_str());

    std::ofstream fpout (filename.c_str (), std::ios::app | std::ios::binary);

    std::vector<double> globalCamPose = dataInferPtr->getSurveillanceTransform(0);
    fpout.write (reinterpret_cast<const char*> (&globalCamPose[0]), 6*sizeof (double));

    if (oneTimeGlobal) {
        memcpy(&decompressionBufferGlobalImages[0][0], dataInferPtr->getSurveillanceRGBPtr(0), GlobalCamInfo::getInstance().numPixels() * 3);
        fpout.write (reinterpret_cast<const char*> (decompressionBufferGlobalImages[0]), GlobalCamInfo::getInstance().numPixels() * 3);
    }

    ros::Rate r(10);
    auto startTime = ros::Time::now().toSec();
    while (not ros::isShuttingDown()) {
        int64_t frameTime = dataInferPtr->getTimeStamp();
        fpout.write (reinterpret_cast<const char*> (&frameTime), sizeof (int64_t));

        std::vector<double> camPose = dataInferPtr->getCamTransform();
        fpout.write (reinterpret_cast<const char*> (&camPose[0]), 6*sizeof (double));

        memcpy(&decompressionBufferImage[0], dataInferPtr->getRGBPtr(),LocalCameraInfo::getInstance().numPixels() * 3);
        fpout.write (reinterpret_cast<const char*> (decompressionBufferImage), LocalCameraInfo::getInstance().numPixels() * 3);

        memcpy(&decompressionBufferDepth[0], dataInferPtr->getDepthPtr(), LocalCameraInfo::getInstance().numPixels() * 2);
        fpout.write (reinterpret_cast<const char*> (decompressionBufferDepth), LocalCameraInfo::getInstance().numPixels() * 2);

        if (!oneTimeGlobal) {
            memcpy(&decompressionBufferGlobalImages[0][0], dataInferPtr->getSurveillanceRGBPtr(0), GlobalCamInfo::getInstance().numPixels() * 3);
            fpout.write (reinterpret_cast<const char*> (decompressionBufferGlobalImages[0]), GlobalCamInfo::getInstance().numPixels() * 3);
        }

        std::cout<<ros::Time::now().toSec() - startTime<<std::endl;
        r.sleep();
        if (ros::Time::now().toSec() - startTime > 200)
            break;
    }

    std::cout<<"stoped"<<std::endl;
    fpout.close ();
}

void testData(int argc, char *argv[]) {

    ros::init(argc, argv, "ros_efusion");
    auto dataInferPtr = std::make_shared<DataInterface>(1);
    auto rosInterPtr = std::make_shared<rosInterface>(dataInferPtr, 1);
    dataInferPtr->dataReady();

    Bytef * decompressionBufferImage;
    decompressionBufferImage = new Bytef[LocalCameraInfo::getInstance().numPixels() * 3];

    const std::string file;
    std::string filename = "rosrecord";
    filename.append(".ply");
    std::ofstream fpout (filename.c_str (), std::ios::app | std::ios::binary);

    int64_t frameTime = dataInferPtr->getTimeStamp();
    memcpy(&decompressionBufferImage[0], dataInferPtr->getRGBPtr(),LocalCameraInfo::getInstance().numPixels() * 3);

    fpout.write (reinterpret_cast<const char*> (&frameTime), sizeof (int64_t));
    fpout.write (reinterpret_cast<const char*> (decompressionBufferImage), LocalCameraInfo::getInstance().numPixels() * 3);

    fpout.close ();
    delete [] decompressionBufferImage;

    int64_t lastFrameTime=0;
    unsigned char * imageReadBuffer = new unsigned char[LocalCameraInfo::getInstance().numPixels() * 3];
    decompressionBufferImage = new Bytef[LocalCameraInfo::getInstance().numPixels() * 3];

    std::ifstream recordFile;
    recordFile.open(file.c_str());
    recordFile.read(reinterpret_cast<char*>(&lastFrameTime), sizeof(int64_t));
    std::cout<< lastFrameTime<<std::endl;
    recordFile.read(reinterpret_cast<char *>(imageReadBuffer), LocalCameraInfo::getInstance().numPixels() * 3);
//    auto a = *imageReadBuffer;
//    std::cout<< a<<std::endl;
    memcpy(&decompressionBufferImage[0], imageReadBuffer, LocalCameraInfo::getInstance().numPixels() * 3);

    recordFile.close ();
    delete [] decompressionBufferImage;
}

int main(int argc, char *argv[])
{
    LocalCameraInfo::getInstance(528, 528, 320, 240, 640, 480);
    GlobalCamInfo::getInstance(640, 480, 457, 457, 320, 224, 1);

    collectData(argc, argv, true);
//    testData(argc, argv);

    return 0;
}