//
// Created by jing on 6/23/21.
//


#include "GlobalCamInfo.h"

const GlobalCamInfo & GlobalCamInfo::getInstance(int width,int height, float fx, float fy, float cx, float cy, int number)
{
    static const GlobalCamInfo instance(width,height, fx, fy, cx, cy, number);
    return instance;
}

