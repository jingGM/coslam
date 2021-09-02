//
// Created by jing on 7/9/21.
//

#ifndef SLAM_LOCALCAMERAINFO_H
#define SLAM_LOCALCAMERAINFO_H
#include <cassert>

class LocalCameraInfo {
public:
    static const LocalCameraInfo & getInstance(float fx = 0,float fy = 0,float cx = 0,float cy = 0, int width=0, int height=0);

    const float & fx() const
    {
        return fx_;
    }

    const float & fy() const
    {
        return fy_;
    }

    const float & cx() const
    {
        return cx_;
    }

    const float & cy() const
    {
        return cy_;
    }

    const int & width() const
    {
        return imgWidth;
    }

    const int & height() const
    {
        return imgHeight;
    }

    const int & cols() const
    {
        return imgWidth;
    }

    const int & rows() const
    {
        return imgHeight;
    }

    const int & numPixels() const
    {
        return imgNumPixels;
    }

private:
    LocalCameraInfo(float fx, float fy, float cx, float cy, int width, int height)
            : fx_(fx),
              fy_(fy),
              cx_(cx),
              cy_(cy),
              imgWidth(width),
              imgHeight(height),
              imgNumPixels(width * height)
    {
        assert(fx != 0 && fy != 0 && width > 0 && height > 0 && "You haven't initialised the LocalCameraInfo class!");
    }

    const float fx_, fy_, cx_, cy_;
    const int imgWidth;
    const int imgHeight;
    const int imgNumPixels;
};


#endif //SLAM_LOCALCAMERAINFO_H
