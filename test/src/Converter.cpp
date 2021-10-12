//
// Created by jing on 10/11/21.
//

#include "Converter.h"


//将描述子转换为描述子向量，其实本质上是cv:Mat->std:vector
std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    //存储转换结果的向量
    std::vector<cv::Mat> vDesc;
    //创建保留空间
    vDesc.reserve(Descriptors.rows);
    //对于每一个特征点的描述子
    for (int j=0;j<Descriptors.rows;j++)
        //从描述子这个矩阵中抽取出来存到向量中
        vDesc.push_back(Descriptors.row(j));

    //返回转换结果
    return vDesc;
}