//
// Created by jing on 9/27/21.
//

#include "GlobalOdometry.h"

GlobalOdometry::GlobalOdometry(float distThresh, float angleThresh):
lastICPError(0),
lastICPCount(GlobalCamInfo::getInstance().width() * GlobalCamInfo::getInstance().height()),
lastRGBError(0),
lastRGBCount(GlobalCamInfo::getInstance().width() * GlobalCamInfo::getInstance().height()),
lastSO3Error(0),
lastSO3Count(GlobalCamInfo::getInstance().width() * GlobalCamInfo::getInstance().height()),
lastA(Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero()),
lastb(Eigen::Matrix<double, 6, 1>::Zero()),
sobelSize(3),
sobelScale(1.0 / pow(2.0, sobelSize)),
maxDepthDeltaRGB(0.07),
maxDepthRGB(6.0),
distThres_(distThresh),
angleThres_(angleThresh),
width(GlobalCamInfo::getInstance().width()),
height(GlobalCamInfo::getInstance().height()),
gcx(GlobalCamInfo::getInstance().cx()),
gcy(GlobalCamInfo::getInstance().cy()),
gfx(GlobalCamInfo::getInstance().fx()),
gfy(GlobalCamInfo::getInstance().fy()),
lcx(LocalCameraInfo::getInstance().cx()),
lcy(LocalCameraInfo::getInstance().cy()),
lfx(LocalCameraInfo::getInstance().fx()),
lfy(LocalCameraInfo::getInstance().fy())
{
    assert(GlobalCamInfo::getInstance().width()==LocalCameraInfo::getInstance().width() &&
                   GlobalCamInfo::getInstance().height()==LocalCameraInfo::getInstance().height());
    sumDataSE3.create(MAX_THREADS);
    outDataSE3.create(1);
    sumResidualRGB.create(MAX_THREADS);

    sumDataSO3.create(MAX_THREADS);
    outDataSO3.create(1);

    for(int i = 0; i < NUM_PYRS; i++)
    {
        int2 nextDim = {height >> i, width >> i};
        pyrDims.push_back(nextDim);
    }

    for(int i = 0; i < NUM_PYRS; i++)
    {
        lastImage[i].create(pyrDims.at(i).x, pyrDims.at(i).y);
        nextImage[i].create(pyrDims.at(i).x, pyrDims.at(i).y);
        nextDepth[i].create(pyrDims.at(i).x, pyrDims.at(i).y);
        globalImage[i].create(pyrDims.at(i).x, pyrDims.at(i).y);

        nextdIdx[i].create(pyrDims.at(i).x, pyrDims.at(i).y);
        nextdIdy[i].create(pyrDims.at(i).x, pyrDims.at(i).y);

        pointClouds[i].create(pyrDims.at(i).x, pyrDims.at(i).y);

        corresImg[i].create(pyrDims.at(i).x, pyrDims.at(i).y);
    }

    lintr.cx = lcx;
    lintr.cy = lcy;
    lintr.fx = lfx;
    lintr.fy = lfy;
    gintr.cx = gcx;
    gintr.cy = gcy;
    gintr.fx = gfx;
    gintr.fy = gfy;

    iterations.resize(NUM_PYRS);

    depth_tmp.resize(NUM_PYRS);

    vmaps_g_prev_.resize(NUM_PYRS);
    nmaps_g_prev_.resize(NUM_PYRS);

    vmaps_curr_.resize(NUM_PYRS);
    nmaps_curr_.resize(NUM_PYRS);

    for (int i = 0; i < NUM_PYRS; ++i)
    {
        int pyr_rows = height >> i;
        int pyr_cols = width >> i;

        depth_tmp[i].create (pyr_rows, pyr_cols);

        vmaps_g_prev_[i].create (pyr_rows*3, pyr_cols);
        nmaps_g_prev_[i].create (pyr_rows*3, pyr_cols);

        vmaps_curr_[i].create (pyr_rows*3, pyr_cols);
        nmaps_curr_[i].create (pyr_rows*3, pyr_cols);
    }

    vmaps_tmp.create(height * 4 * width);

    minimumGradientMagnitudes.resize(NUM_PYRS);
    minimumGradientMagnitudes[0] = 5;
    minimumGradientMagnitudes[1] = 3;
    minimumGradientMagnitudes[2] = 1;
}

GlobalOdometry::~GlobalOdometry() {

}

void GlobalOdometry::initFirstRGB(GPUTexture *rgb) {
    cudaArray * textPtr;

    cudaGraphicsMapResources(1, &rgb->cudaRes);

    cudaGraphicsSubResourceGetMappedArray(&textPtr, rgb->cudaRes, 0, 0);

    imageBGRToIntensity(textPtr, globalImage[0]);

    cudaGraphicsUnmapResources(1, &rgb->cudaRes);

    for(int i = 0; i + 1 < NUM_PYRS; i++)
    {
        pyrDownUcharGauss(globalImage[i], globalImage[i + 1]);
    }
}

void GlobalOdometry::initRGBModel(GPUTexture *rgb) {
    populateRGBDData(rgb, &lastImage[0]);
}

void GlobalOdometry::initRGB(GPUTexture *rgb, GPUTexture *predictedVertices, const float depthCutoff) {
    cudaArray * textPtr;

    cudaGraphicsMapResources(1, &predictedVertices->cudaRes);
    cudaGraphicsSubResourceGetMappedArray(&textPtr, predictedVertices->cudaRes, 0, 0);
    cudaMemcpyFromArray(vmaps_tmp.ptr(), textPtr, 0, 0, vmaps_tmp.sizeBytes(), cudaMemcpyDeviceToDevice);
    cudaGraphicsUnmapResources(1, &predictedVertices->cudaRes);

    cudaDeviceSynchronize();

    populateRGBDData(rgb, &nextImage[0], &nextDepth[0]);
}

void GlobalOdometry::populateRGBDData(GPUTexture * rgb, DeviceArray2D<unsigned char> *destImages, DeviceArray2D<float> * destDepths) {
    if (destDepths) {
        verticesToDepth(vmaps_tmp, destDepths[0], maxDepthRGB);

        for(int i = 0; i + 1 < NUM_PYRS; i++)
        {
            pyrDownGaussF(destDepths[i], destDepths[i + 1]);
        }
    }

    cudaArray * textPtr;

    cudaGraphicsMapResources(1, &rgb->cudaRes);

    cudaGraphicsSubResourceGetMappedArray(&textPtr, rgb->cudaRes, 0, 0);

    imageBGRToIntensity(textPtr, destImages[0]);

    cudaGraphicsUnmapResources(1, &rgb->cudaRes);

    for(int i = 0; i + 1 < NUM_PYRS; i++)
    {
        pyrDownUcharGauss(destImages[i], destImages[i + 1]);
    }

    cudaDeviceSynchronize();
}

void GlobalOdometry::getIncrementalTransformation(Eigen::Vector3f &trans,
                                                  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> &rot,
                                                  const bool &pyramid,
                                                  const bool &fastOdom,
                                                  const bool &so3)

{
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Rprev = rot;
    Eigen::Vector3f tprev = trans;

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Rcurr = Rprev;
    Eigen::Vector3f tcurr = tprev;


    for(int i = 0; i < NUM_PYRS; i++)
    {
        computeDerivativeImages(nextImage[i], nextdIdx[i], nextdIdy[i]);
    }

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> resultR = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Identity();

    if(so3)
    {
        int pyramidLevel = 2;

        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_lr = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>::Identity();

        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> gK = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> lK = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();

        gK(0, 0) = gintr(pyramidLevel).fx;
        gK(1, 1) = gintr(pyramidLevel).fy;
        gK(0, 2) = gintr(pyramidLevel).cx;
        gK(1, 2) = gintr(pyramidLevel).cy;
        gK(2, 2) = 1;

//        lK(0, 0) = lintr(pyramidLevel).fx;
//        lK(1, 1) = lintr(pyramidLevel).fy;
//        lK(0, 2) = lintr(pyramidLevel).cx;
//        lK(1, 2) = lintr(pyramidLevel).cy;
//        lK(2, 2) = 1;

        float lastError = std::numeric_limits<float>::max() / 2;
        float lastCount = std::numeric_limits<float>::max() / 2;

        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> lastResultR = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Identity();

        for(int i = 0; i < 10; i++)
        {
            Eigen::Matrix<float, 3, 3, Eigen::RowMajor> jtj;
            Eigen::Matrix<float, 3, 1> jtr;

//            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> lhomography = lK * resultR * lK.inverse();
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> ghomography = gK * resultR * gK.inverse();

            mat33 imageBasis;
            memcpy(&imageBasis.data[0], ghomography.cast<float>().eval().data(), sizeof(mat33));

            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_inv = gK.inverse();
            mat33 kinv;
            memcpy(&kinv.data[0], K_inv.cast<float>().eval().data(), sizeof(mat33));

            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_R_lr = gK * resultR;
            mat33 krlr;
            memcpy(&krlr.data[0], K_R_lr.cast<float>().eval().data(), sizeof(mat33));

            float residual[2];

            TICK("so3Step");
            so3Step(globalImage[pyramidLevel],
                    nextImage[pyramidLevel],
                    imageBasis,
                    kinv,
                    krlr,
                    sumDataSO3,
                    outDataSO3,
                    jtj.data(),
                    jtr.data(),
                    &residual[0],
                    GPUConfig::getInstance().so3StepThreads,
                    GPUConfig::getInstance().so3StepBlocks);
            TOCK("so3Step");

            lastSO3Error = sqrt(residual[0]) / residual[1];
            lastSO3Count = residual[1];

            //Converged
            if(lastSO3Error < lastError && lastCount == lastSO3Count)
            {
                break;
            }
            else if(lastSO3Error > lastError + 0.001) //Diverging
            {
                lastSO3Error = lastError;
                lastSO3Count = lastCount;
                resultR = lastResultR;
                break;
            }

            lastError = lastSO3Error;
            lastCount = lastSO3Count;
            lastResultR = resultR;

            Eigen::Vector3f delta = jtj.ldlt().solve(jtr);

            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotUpdate = OdometryProvider::rodrigues(delta.cast<double>());

            R_lr = rotUpdate.cast<float>() * R_lr;

            for(int x = 0; x < 3; x++)
            {
                for(int y = 0; y < 3; y++)
                {
                    resultR(x, y) = R_lr(x, y);
                }
            }
        }
    }

    iterations[0] = fastOdom ? 3 : 10;
    iterations[1] = pyramid ? 5 : 0;
    iterations[2] = pyramid ? 4 : 0;

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Rprev_inv = Rprev.inverse();
    mat33 device_Rprev_inv = Rprev_inv;
    float3 device_tprev = *reinterpret_cast<float3*>(tprev.data());

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> resultRt = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>::Identity();

    if(so3)
    {
        for(int x = 0; x < 3; x++)
        {
            for(int y = 0; y < 3; y++)
            {
                resultRt(x, y) = resultR(x, y);
            }
        }
    }

    for(int i = NUM_PYRS - 1; i >= 0; i--)
    {
//        projectToPointCloud(lastDepth[i], pointClouds[i], gintr, i);

        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();

        K(0, 0) = gintr(i).fx;
        K(1, 1) = gintr(i).fy;
        K(0, 2) = gintr(i).cx;
        K(1, 2) = gintr(i).cy;
        K(2, 2) = 1;

        lastRGBError = std::numeric_limits<float>::max();

        for(int j = 0; j < iterations[i]; j++)
        {
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Rt = resultRt.inverse();

            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = Rt.topLeftCorner(3, 3);

            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> KRK_inv = K * R * K.inverse();
            mat33 krkInv;
            memcpy(&krkInv.data[0], KRK_inv.cast<float>().eval().data(), sizeof(mat33));

            Eigen::Vector3d Kt = Rt.topRightCorner(3, 1);
            Kt = K * Kt;
            float3 kt = {(float)Kt(0), (float)Kt(1), (float)Kt(2)};

            int sigma = 0;
            int rgbSize = 0;

            TICK("computeRgbResidual");
            computeRgbResidual(pow(minimumGradientMagnitudes[i], 2.0) / pow(sobelScale, 2.0),
                               nextdIdx[i],
                               nextdIdy[i],
                               lastDepth[i],
                               nextDepth[i],
                               lastImage[i],
                               nextImage[i],
                               corresImg[i],
                               sumResidualRGB,
                               maxDepthDeltaRGB,
                               kt,
                               krkInv,
                               sigma,
                               rgbSize,
                               GPUConfig::getInstance().rgbResThreads,
                               GPUConfig::getInstance().rgbResBlocks);
            TOCK("computeRgbResidual");


            float sigmaVal = std::sqrt((float)sigma / rgbSize == 0 ? 1 : rgbSize);
            float rgbError = std::sqrt(sigma) / (rgbSize == 0 ? 1 : rgbSize);

            if(rgbError > lastRGBError)
            {
                break;
            }

            lastRGBError = rgbError;
            lastRGBCount = rgbSize;


            sigmaVal = -1; //Signals the internal optimisation to weight evenly


//            Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A_icp;
//            Eigen::Matrix<float, 6, 1> b_icp;
//
//            mat33 device_Rcurr = Rcurr;
//            float3 device_tcurr = *reinterpret_cast<float3*>(tcurr.data());
//
//            DeviceArray2D<float>& vmap_curr = vmaps_curr_[i];
//            DeviceArray2D<float>& nmap_curr = nmaps_curr_[i];
//
//            DeviceArray2D<float>& vmap_g_prev = vmaps_g_prev_[i];
//            DeviceArray2D<float>& nmap_g_prev = nmaps_g_prev_[i];
//
//            float residual[2];
//
//            if(icp)
//            {
//                TICK("icpStep");
//                icpStep(device_Rcurr,
//                        device_tcurr,
//                        vmap_curr,
//                        nmap_curr,
//                        device_Rprev_inv,
//                        device_tprev,
//                        intr(i),
//                        vmap_g_prev,
//                        nmap_g_prev,
//                        distThres_,
//                        angleThres_,
//                        sumDataSE3,
//                        outDataSE3,
//                        A_icp.data(),
//                        b_icp.data(),
//                        &residual[0],
//                        GPUConfig::getInstance().icpStepThreads,
//                        GPUConfig::getInstance().icpStepBlocks);
//                TOCK("icpStep");
//            }
//
//            lastICPError = sqrt(residual[0]) / residual[1];
//            lastICPCount = residual[1];

            Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A_rgbd;
            Eigen::Matrix<float, 6, 1> b_rgbd;


            TICK("rgbStep");
            rgbStep(corresImg[i],
                    sigmaVal,
                    pointClouds[i],
                    gintr(i).fx,
                    gintr(i).fy,
                    nextdIdx[i],
                    nextdIdy[i],
                    sobelScale,
                    sumDataSE3,
                    outDataSE3,
                    A_rgbd.data(),
                    b_rgbd.data(),
                    GPUConfig::getInstance().rgbStepThreads,
                    GPUConfig::getInstance().rgbStepBlocks);
            TOCK("rgbStep");


            Eigen::Matrix<double, 6, 1> result;
            Eigen::Matrix<double, 6, 6, Eigen::RowMajor> dA_rgbd = A_rgbd.cast<double>();
            Eigen::Matrix<double, 6, 1> db_rgbd = b_rgbd.cast<double>();

            lastA = dA_rgbd;
            lastb = db_rgbd;
            result = lastA.ldlt().solve(lastb);

            Eigen::Isometry3f rgbOdom;

            OdometryProvider::computeUpdateSE3(resultRt, result, rgbOdom);

            Eigen::Isometry3f currentT;
            currentT.setIdentity();
            currentT.rotate(Rprev);
            currentT.translation() = tprev;

            currentT = currentT * rgbOdom.inverse();

            tcurr = currentT.translation();
            Rcurr = currentT.rotation();
        }
    }

    if((tcurr - tprev).norm() > 0.3)
    {
        Rcurr = Rprev;
        tcurr = tprev;
    }

//    if(so3)
//    {
//        for(int i = 0; i < NUM_PYRS; i++)
//        {
//            std::swap(lastNextImage[i], nextImage[i]);
//        }
//    }

    trans = tcurr;
    rot = Rcurr;
}

Eigen::MatrixXd GlobalOdometry::getCovariance() {
    return lastA.cast<double>().lu().inverse();
}


