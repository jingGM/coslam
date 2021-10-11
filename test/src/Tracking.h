//
// Created by jing on 10/9/21.
//

#ifndef TEST_TRACKING_H
#define TEST_TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "System.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "Initializer.h"
#include "ORBmatcher.h"
#include "drawer/Drawer.h"

class System;

class Tracking {
public:
    Tracking(System* pSys, ORBVocabulary* pVoc, const string &strSettingPath, bool draw);

    cv::Mat processImage(const cv::Mat &im, const double &timestamp, const bool& isGlobalFrame);

    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };
    eTrackingState mState;
    eTrackingState mLastProcessedState;

//    Frame mCurrentFrame;
    cv::Mat mImGray;
    cv::Mat mImOrigin;

private:
    ORBextractor* mpIniORBextractor;
    ORBextractor* mpORBextractorLeft;
    int nLevels;

    ORBVocabulary* mpORBVocabulary;

    System* mpSystem;

    Frame mCurrentFrame;
    Frame mLastFrame;
    Frame mInitialFrame;

    bool draw;
    Drawer drawer;

    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    ///所有的参考关键帧的位姿;看上面注释的意思,这里存储的也是相对位姿
    list<cv::Mat> mlRelativeFramePoses;
    ///所有帧的时间戳  //? 还是关键帧的时间戳?
    list<double> mlFrameTimes;

    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;
    bool mbRGB;
    int rows,cols;

    int globalFeatureNThreshold = 200;
    int localFeatureNThreshold = 100;

    int mnMatchesInliers;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    void initializeIntrinsics(const cv::FileStorage &fSettings);
    void initializeORBPrams(const cv::FileStorage &fSettings);
    void matchFrames(const bool& isGlobalFrame);
    void initializeGlobal();
    void processLocal();
    void initializeDrawer();
};


#endif //TEST_TRACKING_H
