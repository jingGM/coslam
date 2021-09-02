//
// Created by jing on 7/9/21.
//

#include "LocalCameraInfo.h"

const LocalCameraInfo & LocalCameraInfo::getInstance(float fx,float fy,float cx,float cy, int width, int height)
{
    static const LocalCameraInfo instance(fx,fy,cx,cy,width,height);
    return instance;
}
