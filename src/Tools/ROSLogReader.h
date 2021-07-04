//
// Created by jing on 5/22/21.
//

#ifndef COSLAM_ROSLOGREADER_H
#define COSLAM_ROSLOGREADER_H

#include "LogReader.h"
#include "../rosinterface/dataInterface.h"
#include "../Core/src/Utils/Parse.h"

class ROSLogReader : public LogReader
{
public:
    ROSLogReader(std::string file, bool flipColors, bool glc, dataInterfacePtr dataPtr);
    ~ROSLogReader();

    void getNext(bool gRGB);

    bool rewound() {return false;}
    void rewind() {}
    void getBack() {}
    void fastForward(int frame) {}
    bool hasMore() {return true;}
    void setAuto(bool value) {}
    int getNumFrames() {return 0;}
    const std::string  getFile() {return Parse::get().baseDir().append("live");};
private:
    dataInterfacePtr dataPtr;
    int64_t lastFrameTime=0;
};


#endif //COSLAM_ROSLOGREADER_H
