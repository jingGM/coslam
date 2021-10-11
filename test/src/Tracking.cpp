//
// Created by jing on 10/9/21.
//
#include "Tracking.h"

using namespace std;

Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, const string &strSettingPath, bool draw):
        mState(NO_IMAGES_YET),mpORBVocabulary(pVoc),mpSystem(pSys), draw(draw) {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    initializeIntrinsics(fSettings);
    initializeORBPrams(fSettings);
    if (draw) {
        drawer = Drawer(rows,cols);
    }
}

void Tracking::initializeIntrinsics(const cv::FileStorage &fSettings) {
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    rows = fSettings["Camera.rows"];
    cols = fSettings["Camera.cols"];
}

void Tracking::initializeORBPrams(const cv::FileStorage &fSettings) {
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
}

cv::Mat Tracking::processImage(const cv::Mat &im, const double &timestamp, const bool& isGlobalFrame) {
    mImGray = im;
    mImOrigin = im;
    if(mImOrigin.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImOrigin,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImOrigin,mImGray,CV_BGR2GRAY);
    }
    else if(mImOrigin.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImOrigin,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImOrigin,mImGray,CV_BGRA2GRAY);
    }

    if(isGlobalFrame){
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef, mbf, isGlobalFrame);
        if (mCurrentFrame.mvKeys.size() < globalFeatureNThreshold){
            std::cout<<"global features are not enough: <"<<globalFeatureNThreshold<<std::endl;
            return cv::Mat();
        }
    } else {
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef, mbf, isGlobalFrame);
        if (mCurrentFrame.mvKeys.size() < localFeatureNThreshold) {
            std::cout<<"local features are not enough: <"<<localFeatureNThreshold<<std::endl;
            return cv::Mat();
        }

    }
    std::cout<<"useful keys: "<< mCurrentFrame.usefulN<<std::endl;

    matchFrames(isGlobalFrame);
    return mCurrentFrame.mTcw.clone();
}

void Tracking::matchFrames(const bool& isGlobalFrame) {
    if (isGlobalFrame) {
        initializeGlobal();
    } else {
        if (!mpInitializer) assert("Please initialize global image first");
        mLastProcessedState=mState;
        processLocal();
    }

    if(draw) {
        drawer.showFrame();
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    // Step 11：记录位姿信息，用于最后保存所有的轨迹
    if(!mCurrentFrame.mTcw.empty())
    {
        // 计算相对姿态Tcr = Tcw * Twr, Twr = Trw^-1
//        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        //保存各种状态
        mlRelativeFramePoses.push_back(mCurrentFrame.mTcw);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
    }
    else
    {
        // 跟踪失败
        mlRelativeFramePoses.push_back(cv::Mat());
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
    }
}

void Tracking::initializeGlobal() {
    // 初始化需要两帧，分别是mInitialFrame，mCurrentFrame
    mInitialFrame = Frame(mCurrentFrame);

    // mvbPrevMatched  记录"上一帧"所有特征点
    mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
    for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
        mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

    // 删除前判断一下，来避免出现段错误。不过在这里是多余的判断
    // 不过在这里是多余的判断，因为前面已经判断过了
    if(mpInitializer)
        delete mpInitializer;

    // 由当前帧构造初始器 sigma:1.0 iterations:200
    mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

    // 初始化为-1 表示没有任何匹配。这里面存储的是匹配的点的id
    fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

    if (draw) {
        drawer.updateFrame(mImGray, mInitialFrame, mvIniMatches, true);
    }
}

void Tracking::processLocal() {
    // Find correspondences
    // Step 3 在mInitialFrame与mCurrentFrame中找匹配的特征点对
    ORBmatcher matcher(
            0.7,        //最佳的和次佳特征点评分的比值阈值，这里是比较宽松的，跟踪时一般是0.7
            true);      //检查特征点的方向

    // 对 mInitialFrame,mCurrentFrame 进行特征点匹配
    // mvbPrevMatched为参考帧的特征点坐标，初始化存储的是mInitialFrame中特征点坐标，匹配后存储的是匹配好的当前帧的特征点坐标
    // mvIniMatches 保存参考帧F1中特征点是否匹配上，index保存是F1对应特征点索引，值保存的是匹配好的F2特征点索引
    int nmatches = matcher.SearchForInitialization(
            mInitialFrame,mCurrentFrame,    //初始化时的参考帧和当前帧
            mvbPrevMatched,                 //在初始化参考帧中提取得到的特征点
            mvIniMatches,                   //保存匹配关系
            500, nLevels);                           //搜索窗口大小

    // Check if there are enough correspondences
    // Step 4 验证匹配结果，如果初始化的两帧之间的匹配点太少
//    if(nmatches<10)
//    {
//        mCurrentFrame.mTcw = cv::Mat();
//        return;
//    }

    if (draw) {
        drawer.updateFrame(mImGray, mCurrentFrame, mvIniMatches, false);
    }

    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    // Step 5 通过H模型或F模型进行单目初始化，得到两帧间相对运动、初始MapPoints
//    if(mpInitializer->Initialize(
//            mCurrentFrame,      //当前帧
//            mvIniMatches,       //当前帧和参考帧的特征点的匹配关系
//            Rcw, tcw,           //初始化得到的相机的位姿
//            mvIniP3D,           //进行三角化得到的空间点集合
//            vbTriangulated))    //以及对应于mvIniMatches来讲,其中哪些点被三角化了
//    {
//        // Step 6 初始化成功后，删除那些无法进行三角化的匹配点
//        for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
//        {
//            if(mvIniMatches[i]>=0 && !vbTriangulated[i])
//            {
//                mvIniMatches[i]=-1;
//                nmatches--;
//            }
//        }
//
//        // Set Frame Poses
//        // Step 7 将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
//        mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
//        // 由Rcw和tcw构造Tcw,并赋值给mTcw，mTcw为世界坐标系到相机坐标系的变换矩阵
//        cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
//        Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
//        tcw.copyTo(Tcw.rowRange(0,3).col(3));
//        mCurrentFrame.SetPose(Tcw);
//
//        mLastFrame = Frame(mCurrentFrame);
//
//        // Step 8 创建初始化地图点MapPoints
//        // Initialize函数会得到mvIniP3D，
//        // mvIniP3D是cv::Point3f类型的一个容器，是个存放3D点的临时变量，
//        // CreateInitialMapMonocular将3D点包装成MapPoint类型存入KeyFrame和Map中
////        CreateInitialMapMonocular();
//    }//当初始化成功的时候进行
}