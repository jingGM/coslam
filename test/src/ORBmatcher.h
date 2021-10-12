//
// Created by jing on 10/9/21.
//

#ifndef TEST_ORBMATCHER_H
#define TEST_ORBMATCHER_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Frame.h"

class ORBmatcher {
public:
    static const int TH_LOW;            ///< 判断描述子距离时比较低的那个阈值,主要用于基于词袋模型加速的匹配过程，可能是感觉使用词袋模型的时候对匹配的效果要更加严格一些
    static const int TH_HIGH;           ///< 判断描述子距离时比较高的那个阈值,用于计算投影后能够匹配上的特征点的数目；如果匹配的函数中没有提供阈值的话，默认就使用这个阈值
    static const int HISTO_LENGTH;      ///< 判断特征点旋转用直方图的长度
    static const float ROTATION_FACTOR; ///< check the rotation differences with the most rotating angle

    ORBmatcher(float nnratio=0.6, bool checkOri=true, int bestDisNumber=3);
    int SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12,
                                int windowSize, int levelSize);
    int SearchByBoW(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12,
                    int windowSize, int levelSize);
    int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
    void ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

private:
    float mfNNratio;            ///< 最优评分和次优评分的比例
    bool mbCheckOrientation;    ///< 是否检查特征点的方向

    std::vector<int> bestDistance = {};
    int bestDisNumber = 0;
};


#endif //TEST_ORBMATCHER_H
