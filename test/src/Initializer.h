//
// Created by jing on 10/9/21.
//

#ifndef TEST_INITIALIZER_H
#define TEST_INITIALIZER_H

#include<thread>
#include<opencv2/opencv.hpp>

#include "../Thirdparty/DBoW2/DUtils/Random.h"
#include "Frame.h"

class Initializer {
    typedef pair<int,int> Match;
public:
    explicit Initializer(const Frame &ReferenceFrame,
                         bool debug=true,
                float sigma = 1.0,
                int iterations = 200,
                float minParallax=1.0,
                int minTriangulated=20,//50,
                float bestNumberRate=0.5,
                float BestToSecondBest=0.75,
                float RHThreshold=0.40);

    /**
     * @brief 计算基础矩阵和单应性矩阵，选取最佳的来恢复出最开始两帧之间的相对姿态，并进行三角化得到初始地图点
     * Step 1 重新记录特征点对的匹配关系
     * Step 2 在所有匹配特征点对中随机选择8对匹配特征点为一组，用于估计H矩阵和F矩阵
     * Step 3 计算fundamental 矩阵 和homography 矩阵，为了加速分别开了线程计算
     * Step 4 计算得分比例来判断选取哪个模型来求位姿R,t
     *
     * @param[in] CurrentFrame          当前帧，也就是SLAM意义上的第二帧
     * @param[in] vMatches12            当前帧（2）和参考帧（1）图像中特征点的匹配关系
     *                                  vMatches12[i]解释：i表示帧1中关键点的索引值，vMatches12[i]的值为帧2的关键点索引值
     *                                  没有匹配关系的话，vMatches12[i]值为 -1
     * @param[in & out] R21                   相机从参考帧到当前帧的旋转
     * @param[in & out] t21                   相机从参考帧到当前帧的平移
     * @param[in & out] vP3D                  三角化测量之后的三维地图点
     * @param[in & out] vbTriangulated        标记三角化点是否有效，有效为true
     * @return true                     该帧可以成功初始化，返回true
     * @return false                    该帧不满足初始化条件，返回false
     */
    bool Initialize(const Frame &CurrentFrame,
                    const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21,
                    vector<cv::Point3f> &vP3D,
                    vector<bool> &vbTriangulated);

private:
    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

    void FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21);
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2);
    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

    bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K, cv::Mat &R21, cv::Mat &t21,
                      vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1,
                const vector<cv::KeyPoint> &vKeys2, const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);
    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                     const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K, cv::Mat &R21, cv::Mat &t21,
                      vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

    // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> mvKeys1;

    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> mvKeys2;

    // Current Matches from Reference to Current
    // Reference Frame: 1, Current Frame: 2
    /** Match的数据结构是pair,mvMatches12只记录Reference到Current匹配上的特征点对  */
    vector<Match> mvMatches12;
    /** 记录Reference Frame的每个特征点在Current Frame是否有匹配的特征点 */
    vector<bool> mvbMatched1;

    // Calibration
    cv::Mat mK;

    // Standard Deviation and Variance
    float mSigma, mSigma2;

    // 算Fundamental和Homography矩阵时 Ransac max iterations
    int mMaxIterations;

    float minParallax;
    int minTriangulated;
    float bestNumberRate;
    float BestToSecondBest;
    float RHThreshold;
    bool debug;

    /** 二维容器，外层容器的大小为迭代次数，内层容器大小为每次迭代算H或F矩阵需要的点,实际上是八对 */
    vector<vector<size_t> > mvSets;
};


#endif //TEST_INITIALIZER_H
