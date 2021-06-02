//
// Created by jing on 5/22/21.
//

#ifndef COSLAM_ROSLOGREADER_H
#define COSLAM_ROSLOGREADER_H

#include "../src/rosinterface/dataInterface.h"

#include "LogReader.h"
#include <Utils/Parse.h>

class ROSLogReader : public LogReader
{
public:
explicit ROSLogReader(std::string file, bool flipColors, dataInterfacePtr dataPtr);
~ROSLogReader() override;

void getNext() override;

bool rewound() override{return false;}
void rewind() override{}
void getBack() override{}
void fastForward(int frame) override{}
bool hasMore() override{return true;}
void setAuto(bool value) override{}
int getNumFrames() override{return 0;}
const std::string getFile(){return Parse::get().baseDir().append("live");};
private:
    dataInterfacePtr dataPtr;
    int64_t lastFrameTime;
};


#endif //COSLAM_ROSLOGREADER_H
