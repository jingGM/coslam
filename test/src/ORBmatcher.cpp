//
// Created by jing on 10/9/21.
//

#include <chrono>
#include "ORBmatcher.h"


// 要用到的一些阈值
const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 600; // 50
const int ORBmatcher::HISTO_LENGTH = 30;
const float ORBmatcher::ROTATION_FACTOR = 0.1;

// 构造函数,参数默认值为0.6,true
ORBmatcher::ORBmatcher(float nnratio, bool checkOri, int bestDisNumber):
mfNNratio(nnratio), mbCheckOrientation(checkOri), bestDisNumber(bestDisNumber){}

/**
 * @brief 单目初始化中用于参考帧和当前帧的特征点匹配
 * 步骤
 * Step 1 构建旋转直方图
 * Step 2 在半径窗口内搜索当前帧F2中所有的候选匹配特征点
 * Step 3 遍历搜索搜索窗口中的所有潜在的匹配候选点，找到最优的和次优的
 * Step 4 对最优次优结果进行检查，满足阈值、最优/次优比例，删除重复匹配
 * Step 5 计算匹配点旋转角度差所在的直方图
 * Step 6 筛除旋转直方图中“非主流”部分
 * Step 7 将最后通过筛选的匹配好的特征点保存
 * @param[in] F1                        初始化参考帧
 * @param[in] F2                        当前帧
 * @param[in & out] vbPrevMatched       本来存储的是参考帧的所有特征点坐标，该函数更新为匹配好的当前帧的特征点坐标
 * @param[in & out] vnMatches12         保存参考帧F1中特征点是否匹配上，index保存是F1对应特征点索引，值保存的是匹配好的F2特征点索引
 * @param[in] windowSize                搜索窗口
 * @return int                          返回成功匹配的特征点数目
 */
int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2,
                                        vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12,
                                        int windowSize, int levelSize)
{
    int nmatches=0;
    // F1中特征点和F2中匹配关系，注意是按照F1特征点数目分配空间
    vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);

    // Step 1 构建旋转直方图，HISTO_LENGTH = 30
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        // 每个bin里预分配500个，因为使用的是vector不够的话可以自动扩展容量
        rotHist[i].reserve(500);

    //! 原作者代码是 const float factor = 1.0f/HISTO_LENGTH; 是错误的，更改为下面代码
    const float factor = HISTO_LENGTH/360.0f;

    // 匹配点对距离，注意是按照F2特征点数目分配空间
    vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);
    // 从帧2到帧1的反向匹配，注意是按照F2特征点数目分配空间
    vector<int> vnMatches21(F2.mvKeysUn.size(),-1);

    // 遍历帧1中的所有特征点
    for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
    {
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;
        // 只使用原始图像上提取的特征点
//        if(level1>0)
//            continue;

        // Step 2 在半径窗口内搜索当前帧F2中所有的候选匹配特征点
        // vbPrevMatched 输入的是参考帧 F1的特征点
        // windowSize = 100，输入最大最小金字塔层级 均为0
        vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y,
                                                        windowSize,level1,levelSize - 1);
//        auto a = vIndices2.size();
        // 没有候选特征点，跳过
        if(vIndices2.empty())
            continue;

        // 取出参考帧F1中当前遍历特征点对应的描述子
        cv::Mat d1 = F1.mDescriptors.row(i1);

        int bestDist = INT_MAX;     //最佳描述子匹配距离，越小越好
        int bestDist2 = INT_MAX;    //次佳描述子匹配距离
        int bestIdx2 = -1;          //最佳候选特征点在F2中的index

        // Step 3 遍历搜索搜索窗口中的所有潜在的匹配候选点，找到最优的和次优的
        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;
            // 取出候选特征点对应的描述子
            cv::Mat d2 = F2.mDescriptors.row(i2);
            // 计算两个特征点描述子距离
            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[i2]<=dist)
                continue;
            // 如果当前匹配距离更小，更新最佳次佳距离
            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        // Step 4 对最优次优结果进行检查，满足阈值、最优/次优比例，删除重复匹配
        // 即使算出了最佳描述子匹配距离，也不一定保证配对成功。要小于设定阈值
        if(bestDist<=TH_LOW)
        {
            // 最佳距离比次佳距离要小于设定的比例，这样特征点辨识度更高
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                // 如果找到的候选特征点对应F1中特征点已经匹配过了，说明发生了重复匹配，将原来的匹配也删掉
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                // 次优的匹配关系，双向建立
                // vnMatches12保存参考帧F1和F2匹配关系，index保存是F1对应特征点索引，值保存的是匹配好的F2特征点索引
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                // Step 5 计算匹配点旋转角度差所在的直方图
                if(mbCheckOrientation)
                {
                    // 计算匹配特征点的角度差，这里单位是角度°，不是弧度
                    float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    // 前面factor = HISTO_LENGTH/360.0f
                    // bin = rot / 360.of * HISTO_LENGTH 表示当前rot被分配在第几个直方图bin
                    int bin = round(rot*factor);
                    // 如果bin 满了又是一个轮回
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    // Step 6 筛除旋转直方图中“非主流”部分
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;
//        auto a= std::chrono::system_clock::now();
//        bestDistance.clear();
//        for(int i=0; i<bestDisNumber; i++) {
//            bestDistance.push_back(-1);
//        }
//        // 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
//        ComputeThreeMaxima(rotHist,HISTO_LENGTH);

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
//            if(std::find(bestDistance.begin(), bestDistance.end(), i) != bestDistance.end())
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            // 剔除掉不在前N的匹配对，因为他们不符合“主流旋转方向”
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }
//        auto b = std::chrono::system_clock::now() - a;
//        std::cout <<"TIME: "<< b.count() <<std::endl;
    }

    //Update prev matched
    // Step 7 将最后通过筛选的匹配好的特征点保存到vbPrevMatched
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

    return nmatches;
}

// Bit set count operation from
// Hamming distance：两个二进制串之间的汉明距离，指的是其不同位数的个数
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    // 8*32=256bit

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;        // 相等为0,不等为1
        // 下面的操作就是计算其中bit为1的个数了,这个操作看上面的链接就好
        // 其实我觉得也还阔以直接使用8bit的查找表,然后做32次寻址操作就完成了;不过缺点是没有利用好CPU的字长
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

/**
 * @brief 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
 *
 * @param[in] histo         匹配特征点对旋转方向差直方图
 * @param[in] L             直方图尺寸
 * @param[in & out] ind1          bin值第一大对应的索引
 * @param[in & out] ind2          bin值第二大对应的索引
 * @param[in & out] ind3          bin值第三大对应的索引
 */
void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
//    std::priority_queue<std::pair<double, int>> q;
//    for (int i=0; i<L; i++) {
//        q.push(std::pair<double, int>(histo[i].size(), i));
//    }
//
//    double firstValue = q.top().first;
//    bestDistance[0] = q.top().second;
//    q.pop();
//
//    // 如果差距太大了,说明次优的非常不好,这里就索性放弃了,都置为-1
//    for (int i=1; i<bestDisNumber; i++) {
//        if(q.top().first<ROTATION_FACTOR*(float)firstValue)
//            break;
//        else {
//            bestDistance[i] = q.top().second;
//            q.pop();
//        }
//    }
    int max1=0;
    int max2=0;
    int max3=0;
    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    // 如果差距太大了,说明次优的非常不好,这里就索性放弃了,都置为-1
    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}

int ORBmatcher::SearchByBoW(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12,
                            int windowSize, int levelSize) {
    F1.ComputeBoW();
    F2.ComputeBoW();

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    // 将属于同一节点(特定层)的ORB特征进行匹配
    DBoW2::FeatureVector::const_iterator KFit = F1.mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator Fit = F2.mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator KFend = F1.mFeatVec.end();
    DBoW2::FeatureVector::const_iterator Fend = F2.mFeatVec.end();

    int nmatches=0;
    // F1中特征点和F2中匹配关系，注意是按照F1特征点数目分配空间
    vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);
    // 匹配点对距离，注意是按照F2特征点数目分配空间
    vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);
    // 从帧2到帧1的反向匹配，注意是按照F2特征点数目分配空间
    vector<int> vnMatches21(F2.mvKeysUn.size(),-1);

    // Step 1 构建旋转直方图，HISTO_LENGTH = 30
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        // 每个bin里预分配500个，因为使用的是vector不够的话可以自动扩展容量
        rotHist[i].reserve(500);

    // 将0~360的数转换到0~HISTO_LENGTH的系数
    // !原作者代码是 const float factor = 1.0f/HISTO_LENGTH; 是错误的，更改为下面代码
    const float factor = HISTO_LENGTH/360.0f;

    while(KFit != KFend && Fit != Fend)
    {
        // Step 1：分别取出属于同一node的ORB特征点(只有属于同一node，才有可能是匹配点)
        // first 元素就是node id，遍历
        if(KFit->first == Fit->first)
        {
            // second 是该node内存储的feature index
            const vector<unsigned int> vIndicesKF = KFit->second;
            const vector<unsigned int> vIndicesF = Fit->second;
            // Step 2：遍历KF中属于该node的特征点
            for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
            {
                // 关键帧该节点中特征点的索引
                const unsigned int realIdxKF = vIndicesKF[iKF];

                cv::Mat d1 = F1.mDescriptors.row(realIdxKF);

                int bestDist = INT_MAX;     //最佳描述子匹配距离，越小越好
                int bestDist2 = INT_MAX;    //次佳描述子匹配距离
                int bestIdx2 = -1;          //最佳候选特征点在F2中的index

                // Step 3：遍历F中属于该node的特征点，寻找最佳匹配点
                for(size_t iF=0; iF<vIndicesF.size(); iF++)
                {
                    // 和上面for循环重名了,这里的realIdxF是指普通帧该节点中特征点的索引
                    const unsigned int realIdxF = vIndicesF[iF];
                    cv::Mat d2 = F2.mDescriptors.row(realIdxF);
                    int dist = DescriptorDistance(d1,d2);

                    if(vMatchedDistance[realIdxF]<=dist)
                        continue;
                    // 如果当前匹配距离更小，更新最佳次佳距离
                    if(dist<bestDist)
                    {
                        bestDist2=bestDist;
                        bestDist=dist;
                        bestIdx2=realIdxF;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                // Step 4 对最优次优结果进行检查，满足阈值、最优/次优比例，删除重复匹配
                // 即使算出了最佳描述子匹配距离，也不一定保证配对成功。要小于设定阈值
                if(bestDist<=TH_LOW)
                {
                    // 最佳距离比次佳距离要小于设定的比例，这样特征点辨识度更高
                    if(bestDist<(float)bestDist2*mfNNratio)
                    {
                        // 如果找到的候选特征点对应F1中特征点已经匹配过了，说明发生了重复匹配，将原来的匹配也删掉
                        if(vnMatches21[bestIdx2]>=0)
                        {
                            vnMatches12[vnMatches21[bestIdx2]]=-1;
                            nmatches--;
                        }
                        // 次优的匹配关系，双向建立
                        // vnMatches12保存参考帧F1和F2匹配关系，index保存是F1对应特征点索引，值保存的是匹配好的F2特征点索引
                        vnMatches12[realIdxKF]=bestIdx2;
                        vnMatches21[bestIdx2]=realIdxKF;
                        vMatchedDistance[bestIdx2]=bestDist;
                        nmatches++;

                        // Step 5 计算匹配点旋转角度差所在的直方图
                        if(mbCheckOrientation)
                        {
                            // 计算匹配特征点的角度差，这里单位是角度°，不是弧度
                            float rot = F1.mvKeysUn[realIdxKF].angle-F2.mvKeysUn[bestIdx2].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            // 前面factor = HISTO_LENGTH/360.0f
                            // bin = rot / 360.of * HISTO_LENGTH 表示当前rot被分配在第几个直方图bin
                            int bin = round(rot*factor);
                            // 如果bin 满了又是一个轮回
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(realIdxKF);
                        }
                    }
                }

            }
            KFit++;
            Fit++;
        }
        else if(KFit->first < Fit->first)
        {
            // 对齐
            KFit = F1.mFeatVec.lower_bound(Fit->first);
        }
        else
        {
            // 对齐
            Fit = F2.mFeatVec.lower_bound(KFit->first);
        }
    }

    // Step 5 根据方向剔除误匹配的点
    if(mbCheckOrientation)
    {
        // index
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        // 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            // 如果特征点的旋转角度变化量属于这三个组，则保留
            if(i==ind1 || i==ind2 || i==ind3)
                continue;

            // 剔除掉不在前三的匹配对，因为他们不符合“主流旋转方向”
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }
    }

    //Update prev matched
    // Step 7 将最后通过筛选的匹配好的特征点保存到vbPrevMatched
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

    return nmatches;
}

