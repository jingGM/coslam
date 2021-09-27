//
// Created by jing on 5/22/21.
//

#ifndef COSLAM_ROSLOGREADER_H
#define COSLAM_ROSLOGREADER_H

#include <iostream>
#include <fstream>

#include "LogReader.h"
#include "rosinterface/dataInterface.h"
#include "../Utils/Parse.h"

class ROSLogReader : public LogReader
{
public:
    // timeDiff: ms
    ROSLogReader(std::string file, bool flipColors, bool glc, bool record, dataInterfacePtr dataPtr, int64_t timeDiff);
    ~ROSLogReader();

    void getNext(bool gRGB=false);
    void calculateGlobalCam();

    bool rewound() {return false;}
    void rewind() {}
    void getBack() {}
    void fastForward(int frame) {}
    bool hasMore() {return true;}
    void setAuto(bool value) {}
    int getNumFrames() {return 0;}
    const std::string  getFile() {return Parse::get().baseDir().append("live");};
private:
    bool globalCamOn;
    bool record;
    dataInterfacePtr dataPtr;
    int64_t lastFrameTime=0;
    int64_t frequency = 0;
    std::ifstream recordFile;
};


#endif //COSLAM_ROSLOGREADER_H
