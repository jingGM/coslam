//
// Created by jing on 5/22/21.
//

#include "ROSLogReader.h"

ROSLogReader::ROSLogReader(std::string file, bool flipColors, bool glc, bool record, dataInterfacePtr dataPointer, int64_t timeDiff):
LogReader(file, flipColors, glc),
globalCamOn(glc),
record(record),
dataPtr(dataPointer),
frequency(timeDiff)
{
    decompressionBufferDepth = new Bytef[LocalCameraInfo::getInstance().numPixels() * 2];
    decompressionBufferImage = new Bytef[LocalCameraInfo::getInstance().numPixels() * 3];
    if (useGlobalCam){
        for (int i=0; i<GlobalCamInfo::getInstance().camNumber(); i++) {
            decompressionBufferGlobalImages.push_back(new Bytef[GlobalCamInfo::getInstance().numPixels() * 3]);
            globalRGB.push_back(nullptr);
        }
    }

    currentFrame = 0;

    if (!record) dataPointer->dataReady();

    recordFile.open(file.c_str());

    if (record) {
        double x,y,z,r,p,ya;
        recordFile.read(reinterpret_cast<char *>(&x), sizeof(double));
        recordFile.read(reinterpret_cast<char *>(&y), sizeof(double));
        recordFile.read(reinterpret_cast<char *>(&z), sizeof(double));
        recordFile.read(reinterpret_cast<char *>(&r), sizeof(double));
        recordFile.read(reinterpret_cast<char *>(&p), sizeof(double));
        recordFile.read(reinterpret_cast<char *>(&ya), sizeof(double));
        globalCamPoses.push_back({x,y,z,r,p,ya});

        globalImageSize = GlobalCamInfo::getInstance().numPixels() * 3;
        auto * globalTmpImg = new unsigned char[globalImageSize];
        globalImageReadBuffer.push_back(globalTmpImg);
        recordFile.read(reinterpret_cast<char *>(globalImageReadBuffer[0]), globalImageSize);
        memcpy(&(decompressionBufferGlobalImages[0][0]), globalImageReadBuffer[0], globalImageSize);
        globalRGB[0] = (unsigned char *)&decompressionBufferGlobalImages[0][0];
        delete [] globalTmpImg;
    }
}

ROSLogReader::~ROSLogReader()
{
    delete [] decompressionBufferDepth;
    delete [] decompressionBufferImage;
    if (useGlobalCam){
        for (int i=0; i<GlobalCamInfo::getInstance().camNumber(); i++) {
            delete [] decompressionBufferGlobalImages[i];
        }
    }

//    fclose(fp);
    recordFile.close();
}

void ROSLogReader::getNext(bool gRGB) {
    if (record) {
        imageSize = LocalCameraInfo::getInstance().numPixels() * 3;
        depthSize = LocalCameraInfo::getInstance().numPixels() * 2;

        recordFile.read(reinterpret_cast<char*>(&lastFrameTime), sizeof(int64_t));

        double x,y,z,r,p,ya;
        recordFile.read(reinterpret_cast<char *>(&x), sizeof(double));
        recordFile.read(reinterpret_cast<char *>(&y), sizeof(double));
        recordFile.read(reinterpret_cast<char *>(&z), sizeof(double));
        recordFile.read(reinterpret_cast<char *>(&r), sizeof(double));
        recordFile.read(reinterpret_cast<char *>(&p), sizeof(double));
        recordFile.read(reinterpret_cast<char *>(&ya), sizeof(double));
        camPose = {x,y,z,r,p,ya};

        imageReadBuffer = new unsigned char[imageSize];
        recordFile.read(reinterpret_cast<char *>(imageReadBuffer), imageSize);
        memcpy(&decompressionBufferImage[0], imageReadBuffer, imageSize);

        depthReadBuffer = new unsigned char[depthSize];
        recordFile.read(reinterpret_cast<char *>(depthReadBuffer), depthSize);
        memcpy(&decompressionBufferDepth[0], depthReadBuffer, depthSize);
//        std::cout<<gRGB<<std::endl;
    }
    else {
        if(gRGB) {
            assert(useGlobalCam && "you didn't initialize global cameras!");

            for (int i=0; i<GlobalCamInfo::getInstance().camNumber(); i++) {
                memcpy(&decompressionBufferGlobalImages[i][0], dataPtr->getSurveillanceRGBPtr(i), GlobalCamInfo::getInstance().numPixels() * 3);
                globalRGB[i] = (unsigned char *)&decompressionBufferGlobalImages[i][0];
                globalCamPoses.clear();
                globalCamPoses.push_back(dataPtr->getSurveillanceTransform(i));
            }
        }

        memcpy(&decompressionBufferDepth[0], dataPtr->getDepthPtr(), LocalCameraInfo::getInstance().numPixels() * 2);
        memcpy(&decompressionBufferImage[0], dataPtr->getRGBPtr(),LocalCameraInfo::getInstance().numPixels() * 3);

        lastFrameTime = dataPtr->getTimeStamp();

        camPose = dataPtr->getCamTransform();
    }

    timestamp = lastFrameTime;

    rgb = (unsigned char *)&decompressionBufferImage[0];
    depth = (unsigned short *)&decompressionBufferDepth[0];

    imageSize = LocalCameraInfo::getInstance().numPixels() * 3;
    depthSize = LocalCameraInfo::getInstance().numPixels() * 2;
    globalImageSize = GlobalCamInfo::getInstance().numPixels() * 3;

    if(flipColors)
    {
        for(int i = 0; i < LocalCameraInfo::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }
    currentFrame++;
}

