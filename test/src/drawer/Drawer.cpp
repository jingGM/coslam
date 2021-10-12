//
// Created by jing on 10/11/21.
//

#include "Drawer.h"

Drawer::Drawer(int rows, int cols) :
        rows(rows), cols(cols),
        globalFrame(rows, cols),
        localFrame(rows, cols)
{
    mIm = cv::Mat(2*rows,cols,CV_8UC1, cv::Scalar(0,0,0));
}

void Drawer::updateFrame(cv::Mat im, Frame frame, vector<int> matches, bool isglobal) {
    if (isglobal) {
        globalFrame.updateFrame(im, frame, matches);
    }else {
        localFrame.updateFrame(im, frame, matches);
    }
}

void Drawer::showFrame() {
    // show images:
    auto a = globalFrame.mIm.type();
    auto b = localFrame.mIm.type();
    vconcat(globalFrame.mIm, localFrame.mIm, mIm);
    cvtColor(mIm, mIm, cv::COLOR_GRAY2RGB);

    // show features:
    for (int i=0; i<globalFrame.N; i++) {
        cv::Point2f position(globalFrame.mvCurrentKeys[i].pt);
        cv::circle(mIm,position,2,cv::Scalar(0,0,255),1);
    }
    for (int i=0; i<localFrame.N; i++) {
        cv::Point2f position(localFrame.mvCurrentKeys[i].pt);
        position.y = position.y + rows;
        cv::circle(mIm,position,2,cv::Scalar(0,0,255),1);
    }


    // show matches:
    for(unsigned int i=0; i<localFrame.mvIniMatches.size(); i++)
    {
        //绘制当前帧特征点到下一帧特征点的连线,其实就是匹配关系
        //NOTICE 就是当初看到的初始化过程中图像中显示的绿线
        if(localFrame.mvIniMatches[i]>=0)
        {
            cv::Point2f position(localFrame.mvCurrentKeys[localFrame.mvIniMatches[i]].pt);
            position.y = position.y + rows;
            cv::line(mIm,
                     globalFrame.mvCurrentKeys[i].pt,
                     position,
                     cv::Scalar(0,255,0));
        }
    }

    cv::imshow("ORB-SLAM2: Current Frame",mIm);
    //NOTICE 注意对于我所遇到的问题,ORB-SLAM2是这样子来处理的
    cv::waitKey(1e3/30);
}


