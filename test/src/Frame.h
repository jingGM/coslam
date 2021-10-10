//
// Created by jing on 10/9/21.
//

#ifndef TEST_FRAME_H
#define TEST_FRAME_H

#include <opencv2/opencv.hpp>

#include "../Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "../Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"

/// 网格的行数
#define FRAME_GRID_ROWS 48
/// 网格的列数
#define FRAME_GRID_COLS 64

class Frame {
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc,
          cv::Mat &K, cv::Mat &distCoef, const float &bf, const bool &isGlobalFrame);

    void ExtractORB(const cv::Mat &im);
    void UndistortKeyPoints();
    void ComputeImageBounds(const cv::Mat &imLeft);
    void AssignFeaturesToGrid();
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r,
                                     int minLevel, int maxLevel) const;

    void SetPose(cv::Mat Tcw);

    ORBVocabulary* mpORBvocabulary;
    ORBextractor* mpORBextractor;

    double mTimeStamp;
    bool mIsGlobal;

    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;
    // Stereo baseline multiplied by fx.
    float mbf;
    // Stereo baseline in meters.
    float mb;

    // Number of KeyPoints.
    int N;
    int usefulN;
    //原始左图像提取出的特征点（未校正）
    std::vector<cv::KeyPoint> mvKeys;
    //校正mvKeys后的特征点
    std::vector<cv::KeyPoint> mvKeysUn;

    // Bag of Words Vector structures.
    // 内部实际存储的是std::map<WordId, WordValue>
    // WordId 和 WordValue 表示Word在叶子中的id 和权重
    DBoW2::BowVector mBowVec;
    // 内部实际存储 std::map<NodeId, std::vector<unsigned int> >
    // NodeId 表示节点id，std::vector<unsigned int> 中实际存的是该节点id下所有特征点在图像中的索引
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    //原来通过对图像分区域还能够降低重投影地图点时候的匹配复杂度啊。。。。。
    //注意到上面也是类的静态成员变量， 有一个专用的标志mbInitialComputations用来在帧的构造函数中标记这些静态成员变量是否需要被赋值
    // 坐标乘以mfGridElementWidthInv和mfGridElementHeightInv就可以确定在哪个格子
    static float mfGridElementWidthInv;
    // 坐标乘以mfGridElementWidthInv和mfGridElementHeightInv就可以确定在哪个格子
    static float mfGridElementHeightInv;

    // 每个格子分配的特征点数，将图像分成格子，保证提取的特征点比较均匀
    // FRAME_GRID_ROWS 48
    // FRAME_GRID_COLS 64
    //这个向量中存储的是每个图像网格内特征点的id（左图）
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw; //< 相机姿态 世界坐标系到相机坐标坐标系的变换矩阵,是我们常规理解中的相机位姿

    cv::Mat mRcw; ///< Rotation from world to camera
    cv::Mat mtcw; ///< Translation from world to camera
    cv::Mat mRwc; ///< Rotation from camera to world
    cv::Mat mOw;  ///< mtwc,Translation from camera to world

    // Current and Next Frame id.
    // 类的静态成员变量，这些变量则是在整个系统开始执行的时候被初始化的——它在全局区被初始化
    static long unsigned int nNextId; //< Next Frame id.
    long unsigned int mnId; //< Current Frame id.

    // Reference Keyframe.
    // 普通帧与自己共视程度最高的关键帧作为参考关键帧
//    KeyFrame* mpReferenceKF;
    // TODO: this is temp
    bool mpReferenceKF;

    // MapPoints associated to keypoints, NULL pointer if no association.
    /// 每个特征点对应的MapPoint.如果特征点没有对应的地图点,那么将存储一个空指针
//    std::vector<MapPoint*> mvpMapPoints;
    // TODO: this is temp
    bool mvpMapPoints;

    // Flag to identify outlier associations.
    // 观测不到Map中的3D点
    /// 属于外点的特征点标记,在 Optimizer::PoseOptimization 使用了
    std::vector<bool> mvbOutlier;

    // Scale pyramid info.
    int mnScaleLevels;                  //<图像金字塔的层数
    float mfScaleFactor;                //<图像金字塔的尺度因子
    float mfLogScaleFactor;             //<图像金字塔的尺度因子的对数值，用于仿照特征点尺度预测地图点的尺度

    vector<float> mvScaleFactors;		//<图像金字塔每一层的缩放因子
    vector<float> mvInvScaleFactors;	//以及上面的这个变量的倒数
    vector<float> mvLevelSigma2;		//@todo 目前在frame.c中没有用到，无法下定论
    vector<float> mvInvLevelSigma2;		//<上面变量的倒数

    // Undistorted Image Bounds (computed once).
    // 用于确定画格子时的边界
    // 未校正图像的边界，只需要计算一次，因为是类的静态成员变量）
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    // 一个标志，标记是否已经进行了这些初始化计算
    // 由于第一帧以及SLAM系统进行重新校正后的第一帧会有一些特殊的初始化处理操作，所以这里设置了这个变量. \n
    // 如果这个标志被置位，说明下一帧的帧构造函数中要进行这个“特殊的初始化操作”，如果没有被置位则不用。
    static bool mbInitialComputations;
};


#endif //TEST_FRAME_H
