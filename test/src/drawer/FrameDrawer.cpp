//
// Created by jing on 10/11/21.
//

#include "FrameDrawer.h"

FrameDrawer::FrameDrawer(int rows, int cols) {
    // 初始化图像显示画布
    // 包括：图像、特征点连线形成的轨迹（初始化时）、框（跟踪时的MapPoint）、圈（跟踪时的特征点）
    // ！！！固定画布大小为640*480
    mIm = cv::Mat(rows,cols,CV_8UC1, cv::Scalar(0,0,0));
}

void FrameDrawer::updateFrame(cv::Mat im, Frame frame, vector<int> matches) {
    im.copyTo(mIm);
    mvCurrentKeys = frame.mvKeys;
    N = mvCurrentKeys.size();
    mvIniMatches = matches;
}