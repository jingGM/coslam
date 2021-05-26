//
// Created by jing on 5/22/21.
//

#ifndef COSLAM_ROSLOGREADER_H
#define COSLAM_ROSLOGREADER_H

#include "../src/rosinterface/dataInterface.h"

#include "LogReader.h"

class ROSLogReader : public LogReader
{
public:
ROSLogReader(dataInterfacePtr dataPtr);
};


#endif //COSLAM_ROSLOGREADER_H
