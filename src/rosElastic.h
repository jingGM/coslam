//
// Created by jing on 4/25/21.
//

#ifndef COSLAM_ROSELASTIC_H
#define COSLAM_ROSELASTIC_H

#include <memory>
//#include <ElasticFusion.h>
//#include <Utils/Parse.h>
#include "Core/src/Utils/Parse.h"
#include "Core/src/ElasticFusion.h"
#include "rosinterface/rosInterface.h"
#include "rosinterface/dataInterface.h"

#include "Tools/GUI.h"
#include "Tools/GroundTruthOdometry.h"
#include "Tools/RawLogReader.h"
#include "Tools/LiveLogReader.h"
#include "Tools/ROSLogReader.h"
//#include "GUI/src/MainController.h"

class rosElastic {
public:
    dataInterfacePtr dataInferPtr = nullptr;
    rosInterfacePtr rosInterPtr = nullptr;
//    MainControllerPtr mainControllerPtr = nullptr;

    rosElastic(int argc, char * argv[]);

    void run();
//    void runSample(int argc, char **argv);

private:
    void initializeGUI();
    void initializeEfusion();
    void initializeLogger();
    void initializeROSInterface(int argc, char **argv);

    bool good;
    ElasticFusionPtr eFusion=nullptr;
    GUIPtr gui=nullptr;
    LogReaderPtr logReader=nullptr;
    GroundTruthOdometryPtr groundTruthOdometry= nullptr;
    ResizePtr resizeStream=nullptr;

    bool iclnuim;
    std::string logFile;
    std::string logMode;
    std::string poseFile;

    float confidence,
            depth,
            icp,
            icpErrThresh,
            covThresh,
            photoThresh,
            fernThresh;

    int timeDelta,
            icpCountThresh,
            start,
            end;

    bool fillIn{},
            openLoop,
            reloc,
            frameskip,
            quiet,
            fastOdom,
            so3,
            rewind,
            frameToFrameRGB,
            showcaseMode,
            flipColors,
            useGlobalCam;

    int framesToSkip;
    bool streaming{};
    bool resetButton;


};


#endif //COSLAM_ROSELASTIC_H
