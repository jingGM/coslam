#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>

#include "src/System.h"

using namespace std;

void LoadImages(const string &strImagePath,
                const string &strPathTimes,
                vector<string> &vstrImages,
                vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(150);
    vstrImages.reserve(150);

    for (int i=0;i<150;i++) {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }

    int nImages = vstrImages.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
    }
}

int main() {
//    string imagePath   = "/home/jing/Documents/catkinws/slam/test/dataset/data";
//    string pathTimes   = "/home/jing/Documents/catkinws/slam/test/dataset/MH01.txt";
//    string vocFile     = "/home/jing/Documents/catkinws/slam/test/dataset/ORBvoc.txt";
//    string settingFile = "/home/jing/Documents/catkinws/slam/test/dataset/EuRoC.yaml";

    string imagePathG   = "/home/jing/Documents/catkinws/slam/src/coslam/test/dataset/test_set/global/global.png";
    string imagePathL   = "/home/jing/Documents/catkinws/slam/src/coslam/test/dataset/test_set/local";
    string pathTimes   = "/home/jing/Documents/catkinws/slam/src/coslam/test/dataset/test_set/MH01.txt";
    string vocFile     = "/home/jing/Documents/catkinws/slam/src/coslam/test/dataset/ORBvoc_no.txt";
    string settingFile = "/home/jing/Documents/catkinws/slam/src/coslam/test/dataset/test_set/EuRoC.yaml";

    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(imagePathL, pathTimes, vstrImageFilenames, vTimestamps);
    int nImages = vstrImageFilenames.size();

    cv::Mat im;
    im = cv::imread(imagePathG, CV_LOAD_IMAGE_UNCHANGED);

    System SLAM(vocFile,settingFile, true);
    double tframe = 0;
    SLAM.TrackCamera(im,tframe, true);
    for(int ni=0; ni<4; ni++) {
        im = cv::imread(vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
        SLAM.TrackCamera(im,tframe, false);
    }
//    SLAM.Shutdown();
    return 0;
}
