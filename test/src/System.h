//
// Created by jing on 10/5/21.
//

#ifndef TEST_SYSTEM_H
#define TEST_SYSTEM_H
#include <opencv2/core/core.hpp>

#include "ORBVocabulary.h"
#include "Tracking.h"

class Tracking;

class System {
public:
    System(const string &strVocFile, const string &strSettingsFile);
    ORBVocabulary* mpVocabulary;
    cv::Mat TrackCamera(const cv::Mat &im, const double &timeStamp, const bool &isGlobal);
private:
    Tracking* mpTracker;
};


#endif //TEST_SYSTEM_H
