//
// Created by jing on 9/27/21.
//

#ifndef COSLAM_GLOBALODOMETRY_H
#define COSLAM_GLOBALODOMETRY_H

#include "../Utils/Stopwatch.h"
#include "../Core/GPUTexture.h"
#include "../Core/cuda/cudafuncs.cuh"
#include "../Tools/OdometryProvider.h"
#include "../Core/cuda/GPUConfig.h"

#include "../Utils/LocalCameraInfo.h"
#include "../Utils/GlobalCamInfo.h"

#include <vector>
#include <vector_types.h>

class GlobalOdometry {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit GlobalOdometry(float distThresh = 0.10f,
                 float angleThresh = sin(20.f * 3.14159254f / 180.f));

    virtual ~GlobalOdometry();

    void initRGB(GPUTexture * rgb, GPUTexture *depth, const float depthCutoff);

    void initRGBModel(GPUTexture * rgb);

    void initFirstRGB(GPUTexture * rgb);

    void getIncrementalTransformation(Eigen::Vector3f &trans,
                                      Eigen::Matrix<float, 3, 3, Eigen::RowMajor> &rot,
                                      const bool &pyramid,
                                      const bool &fastOdom,
                                      const bool &so3);

    Eigen::MatrixXd getCovariance();

    float lastICPError;
    float lastICPCount;
    float lastRGBError;
    float lastRGBCount;
    float lastSO3Error;
    float lastSO3Count;

    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> lastA;
    Eigen::Matrix<double, 6, 1> lastb;

private:
    void populateRGBDData(GPUTexture * rgb,
                          DeviceArray2D<unsigned char> * destImages,
                          DeviceArray2D<float> * destDepths= nullptr);

    std::vector<DeviceArray2D<unsigned short> > depth_tmp;

    DeviceArray<float> vmaps_tmp;
    DeviceArray<float> nmaps_tmp;

    std::vector<DeviceArray2D<float> > vmaps_g_prev_;
    std::vector<DeviceArray2D<float> > nmaps_g_prev_;

    std::vector<DeviceArray2D<float> > vmaps_curr_;
    std::vector<DeviceArray2D<float> > nmaps_curr_;

    CameraModel gintr,lintr;

    DeviceArray<JtJJtrSE3> sumDataSE3;
    DeviceArray<JtJJtrSE3> outDataSE3;
    DeviceArray<int2> sumResidualRGB;

    DeviceArray<JtJJtrSO3> sumDataSO3;
    DeviceArray<JtJJtrSO3> outDataSO3;

    const int sobelSize;
    const float sobelScale;
    const float maxDepthDeltaRGB;
    const float maxDepthRGB;

    std::vector<int2> pyrDims;

    static const int NUM_PYRS = 3;

    DeviceArray2D<unsigned char> lastImage[NUM_PYRS];

    DeviceArray2D<unsigned char> nextImage[NUM_PYRS];
    DeviceArray2D<float> nextDepth[NUM_PYRS];
    DeviceArray2D<short> nextdIdx[NUM_PYRS];
    DeviceArray2D<short> nextdIdy[NUM_PYRS];

    DeviceArray2D<unsigned char> globalImage[NUM_PYRS];

    DeviceArray2D<DataTerm> corresImg[NUM_PYRS];

    DeviceArray2D<float3> pointClouds[NUM_PYRS];

    std::vector<int> iterations;
    std::vector<float> minimumGradientMagnitudes;

    float distThres_;
    float angleThres_;

    Eigen::Matrix<double, 6, 6> lastCov;

    const int width,height;
    const float gcx, gcy, gfx, gfy, lcx, lcy, lfx, lfy;
};


#endif //COSLAM_GLOBALODOMETRY_H
