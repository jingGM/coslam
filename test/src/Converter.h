//
// Created by jing on 10/11/21.
//

#ifndef TEST_CONVERTER_H
#define TEST_CONVERTER_H

#include<opencv2/core/core.hpp>

class Converter {
public:
    std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);
};


#endif //TEST_CONVERTER_H
