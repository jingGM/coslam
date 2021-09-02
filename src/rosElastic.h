//
// Created by jing on 7/9/21.
//

#ifndef SLAM_ROSELASTIC_H
#define SLAM_ROSELASTIC_H

#include <memory>

#include "Tools/rosinterface/rosInterface.h"
#include "Tools/rosinterface/dataInterface.h"
#include "Tools/RawLogReader.h"
#include "Tools/LiveLogReader.h"
#include "Tools/ROSLogReader.h"
#include "Tools/GroundTruthOdometry.h"
#include "GLDisplay/GUI.h"

#include "Utils/LocalCameraInfo.h"
#include "Utils/GlobalCamInfo.h"

#include "Core/ElasticFusion.h"


class rosElastic {
public:
    rosElastic(int argc, char * argv[]);
    void run();

    // TODO: need to comment out when release
    void test_GUI();
    void test_run();

private:
    void initializeROSInterface(int argc, char **argv);
    void initializeLogger();
    void initializeGUI();
    void initializeEfusion();

    LogReaderPtr logReader=nullptr;
    GroundTruthOdometryPtr gtOdometry=nullptr;
    dataInterfacePtr dataInferPtr = nullptr;
    rosInterfacePtr rosInterPtr = nullptr;
    GUIPtr gui=nullptr;
    ElasticFusionPtr eFusion=nullptr;

    std::string logFile, groundTruthFile;
    std::string logMode;

    float   confidence,
            depth,
            icp,
            icpErrThresh,
            covThresh,
            photoThresh,
            fernThresh;

    int     framesToSkip,
            timeDelta,
            icpCountThresh,
            start,
            end,
            globalTimeDiff,
            globalCamNum;

    bool    good,
            quiet,
            frameskip,
            iclnuim,
            resetButton,
            flipColors,
            globalCamOn,
            fastOdom,
            so3,
            frameToFrameRGB,
            showcaseMode,
            openLoop,
            rewind,
            reloc
            ;
};


#endif //SLAM_ROSELASTIC_H
