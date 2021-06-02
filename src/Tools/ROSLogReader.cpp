//
// Created by jing on 5/22/21.
//

#include "ROSLogReader.h"

ROSLogReader::ROSLogReader(std::string file, bool flipColors, dataInterfacePtr dataPointer):
LogReader(file, flipColors),
dataPtr(dataPointer)
{
    decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];
    decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];
    dataPointer->dataReady();
}

ROSLogReader::~ROSLogReader()
{
    delete [] decompressionBufferDepth;
    delete [] decompressionBufferImage;
}

void ROSLogReader::getNext() {
    memcpy(&decompressionBufferDepth[0], dataPtr->getDepthPtr(), Resolution::getInstance().numPixels() * 2);
    memcpy(&decompressionBufferImage[0], dataPtr->getRGBPtr(),Resolution::getInstance().numPixels() * 3);

    lastFrameTime = dataPtr->getTimeStamp();

    timestamp = lastFrameTime;

    rgb = (unsigned char *)&decompressionBufferImage[0];
    depth = (unsigned short *)&decompressionBufferDepth[0];

//    rgb = dataPtr->getRGBPtr();
//    depth = dataPtr->getDepthPtr();

    imageSize = Resolution::getInstance().numPixels() * 3;
    depthSize = Resolution::getInstance().numPixels() * 2;

    if(flipColors)
    {
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }
}
