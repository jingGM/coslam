//
// Created by jing on 5/22/21.
//

#include "ROSLogReader.h"

ROSLogReader::ROSLogReader(std::string file, bool flipColors, bool glc, dataInterfacePtr dataPointer):
LogReader(file, flipColors, glc),
dataPtr(dataPointer)
{
    decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];
    decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];
    if (useGlobalCam){
        for (int i=0; i<GlobalCamInfo::getInstance().camNumber(); i++) {
            decompressionBufferGlobalImages.push_back(new Bytef[GlobalCamInfo::getInstance().numPixels() * 3]);
            globalRGB.push_back(nullptr);
        }
    }

    dataPointer->dataReady();
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
}

void ROSLogReader::getNext(bool gRGB) {
    if(gRGB) {
        assert(useGlobalCam && "you didn't initialize global cameras!");

        for (int i=0; i<GlobalCamInfo::getInstance().camNumber(); i++) {
            memcpy(&decompressionBufferGlobalImages[i][0], dataPtr->getSurveillanceRGBPtr(i), GlobalCamInfo::getInstance().numPixels() * 3);
            globalRGB[i] = (unsigned char *)&decompressionBufferGlobalImages[i][0];
        }

    }

    memcpy(&decompressionBufferDepth[0], dataPtr->getDepthPtr(), Resolution::getInstance().numPixels() * 2);
    memcpy(&decompressionBufferImage[0], dataPtr->getRGBPtr(),Resolution::getInstance().numPixels() * 3);

    lastFrameTime = dataPtr->getTimeStamp();

    timestamp = lastFrameTime;

    rgb = (unsigned char *)&decompressionBufferImage[0];
    depth = (unsigned short *)&decompressionBufferDepth[0];

    imageSize = Resolution::getInstance().numPixels() * 3;
    depthSize = Resolution::getInstance().numPixels() * 2;
    globalImageSize = GlobalCamInfo::getInstance().numPixels() * 3;

    if(flipColors)
    {
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }
}
