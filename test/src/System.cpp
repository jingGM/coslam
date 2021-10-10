//
// Created by jing on 10/5/21.
//

#include "System.h"

System::System(const string &strVocFile, const string &strSettingsFile) {
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }

    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    mpTracker = new Tracking(this, mpVocabulary, strSettingsFile);
}

cv::Mat System::TrackCamera(const cv::Mat &im, const double &timeStamp, const bool &isGlobal) {
    cv::Mat Tcw = mpTracker->processImage(im, timeStamp, isGlobal);
    std::cout<<Tcw<<std::endl;
    return Tcw;
}

