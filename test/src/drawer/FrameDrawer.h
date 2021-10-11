//
// Created by jing on 10/11/21.
//

#ifndef TEST_FRAMEDRAWER_H
#define TEST_FRAMEDRAWER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../Frame.h"

using namespace std;

class FrameDrawer {
public:
    FrameDrawer() = default;
    FrameDrawer(int rows, int cols);

    void updateFrame(cv::Mat im, Frame frame, vector<int> matches);

    // Info of the frame to be drawn
    ///当前绘制的图像
    cv::Mat mIm;
    ///当前帧中特征点的数目
    int N = 0;
    ///当前帧中的特征点
    vector<cv::KeyPoint> mvCurrentKeys;
    ///当前帧特征点和参考帧特征点的匹配关系
    vector<int> mvIniMatches;
};


#endif //TEST_FRAMEDRAWER_H
