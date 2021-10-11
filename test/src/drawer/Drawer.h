//
// Created by jing on 10/11/21.
//

#ifndef TEST_DRAWER_H
#define TEST_DRAWER_H

#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "FrameDrawer.h"

using namespace std;

class Drawer {
public:
    Drawer() = default;;
    Drawer(int rows, int cols);

    void updateFrame(cv::Mat im, Frame frame, vector<int> matches, bool isglobal);
    void showFrame();

private:
    int rows,cols;
    cv::Mat mIm;
    FrameDrawer globalFrame, localFrame;
};


#endif //TEST_DRAWER_H
