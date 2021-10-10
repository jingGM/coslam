//
// Created by jing on 7/9/21.
//

#include "rosElastic.h"



rosElastic::rosElastic(int argc, char **argv): good(true),
                                               framesToSkip(0),
                                               resetButton(false)
                                               {
    confidence = 10.0f;
    depth = 3.0f;
    icp = 10.0f;
    icpErrThresh = 5e-05;
    covThresh = 1e-05;
    photoThresh = 115;
    fernThresh = 0.3095f;
    timeDelta = 200;
    icpCountThresh = 40000;
    start = 1;
    end = std::numeric_limits<unsigned short>::max(); //Funny bound, since we predict times in this format really!

    Parse::get().arg(argc, argv, "-c", confidence);
    Parse::get().arg(argc, argv, "-d", depth);
    Parse::get().arg(argc, argv, "-i", icp);
    Parse::get().arg(argc, argv, "-ie", icpErrThresh);
    Parse::get().arg(argc, argv, "-cv", covThresh);
    Parse::get().arg(argc, argv, "-pt", photoThresh);
    Parse::get().arg(argc, argv, "-ft", fernThresh);
    Parse::get().arg(argc, argv, "-t", timeDelta);
    Parse::get().arg(argc, argv, "-ic", icpCountThresh);
    Parse::get().arg(argc, argv, "-s", start);
    Parse::get().arg(argc, argv, "-e", end);
    Parse::get().arg(argc, argv, "-l", logFile);
    Parse::get().arg(argc, argv, "-lm", logMode);
    Parse::get().arg(argc, argv, "-gt", groundTruthFile);

    std::string empty;
    iclnuim = Parse::get().arg(argc, argv, "-icl", empty) > -1;
    flipColors = Parse::get().arg(argc,argv,"-f",empty) > -1;
    quiet = Parse::get().arg(argc, argv, "-q", empty) > -1;
    so3 = !(Parse::get().arg(argc, argv, "-nso", empty) > -1);
    fastOdom = Parse::get().arg(argc, argv, "-fo", empty) > -1;
    showcaseMode = Parse::get().arg(argc, argv, "-sc", empty) > -1;
    frameToFrameRGB = Parse::get().arg(argc, argv, "-ftf", empty) > -1;
    rewind = Parse::get().arg(argc, argv, "-r", empty) > -1;
    reloc = Parse::get().arg(argc, argv, "-rl", empty) > -1;
    deformation = (Parse::get().arg(argc, argv, "-df", empty) > -1);
    realsense = (Parse::get().arg(argc, argv, "-rs", empty) > -1);

    openLoop = groundTruthFile.length() > 0 && Parse::get().arg(argc, argv, "-o", empty) > -1;
    frameskip = Parse::get().arg(argc, argv, "-fs", empty) > -1;

    globalTimeDiff = 1;
    globalCamNum = 1;
    globalCamOn = Parse::get().arg(argc,argv,"-gl",empty) > -1;
    Parse::get().arg(argc, argv, "-gf", globalTimeDiff);

    LocalCameraInfo::getInstance(613.6345825195312, 613.6775512695312, 314.1249084472656, 246.6942138671875, 640, 480);
//    LocalCameraInfo::getInstance(528, 528, 320, 240, 640, 480);
//    if (globalCamOn){
    GlobalCamInfo::getInstance(640, 480, 613.6345825195312, 613.6775512695312, 314.1249084472656, 246.6942138671875, 1);
//        GlobalCamInfo::getInstance(640, 480, 457, 457, 320, 224, 1);
//    }

    initializeLogger(argc, argv);
    initializeGUI();
    initializeEfusion();
}

void rosElastic::initializeLogger(int argc, char **argv) {
//    if(logFile.length()) logMode = "record";

    if (logMode=="simulation")
    {
        ros::init(argc, argv, "ros_efusion");
        dataInferPtr = std::make_shared<DataInterface>(globalCamNum);
        rosInterPtr = std::make_shared<rosInterface>(dataInferPtr, globalCamNum);
        logReader = std::make_shared<ROSLogReader>(logFile, flipColors, globalCamOn, false, dataInferPtr, globalTimeDiff);
    }
    else if(logMode == "simrecord") {
        logReader = std::make_shared<ROSLogReader>(logFile, flipColors, globalCamOn, true, dataInferPtr, globalTimeDiff);
    }
    else if(logMode=="record")
    {
        assert((logFile.length()));
        logReader = std::make_shared<RawLogReader>(logFile, flipColors, globalCamOn, realsense);
    }
    else
    {
        logReader = std::make_shared<LiveLogReader>(logFile, flipColors, globalCamOn, LiveLogReader::CameraType::OpenNI2);
        good = std::dynamic_pointer_cast<LiveLogReader>(logReader)->cam->ok();
    }
    logReader->rewind();

    if (groundTruthFile.length()) {
//        gtOdometry=std::make_shared<GroundTruthOdometry>(groundTruthFile);
    }
}

void rosElastic::initializeGUI(){
    gui = std::make_shared<GUI>(logFile.length() == 0, showcaseMode);

    gui->flipColors->Ref().Set(logReader->flipColors);
    gui->rgbOnly->Ref().Set(false);
    gui->pyramid->Ref().Set(true);
    gui->fastOdom->Ref().Set(fastOdom);
    gui->confidenceThreshold->Ref().Set(confidence);
    gui->depthCutoff->Ref().Set(depth);
    gui->icpWeight->Ref().Set(icp);
    gui->so3->Ref().Set(so3);
    gui->frameToFrameRGB->Ref().Set(frameToFrameRGB);
}

void rosElastic::initializeEfusion() {
//    std::vector<std::vector<double>> globalCamPoses;
//    if (globalCamOn) {
//        for (int i=0;i<globalCamNum; i++) {
//            globalCamPoses.push_back(logReader->globalCamPoses[i]);
//        }
//    }
    eFusion = std::make_shared<ElasticFusion>(openLoop ? std::numeric_limits<int>::max() / 2 : timeDelta,
                                              icpCountThresh,
                                              icpErrThresh,
                                              covThresh,
                                              !openLoop,
                                              iclnuim,
                                              photoThresh,
                                              confidence,
                                              depth,
                                              icp,
                                              fastOdom,
                                              reloc,
                                              fernThresh,
                                              so3,
                                              frameToFrameRGB,
                                              logReader->getFile(),
                                              globalCamOn,
                                              deformation);
}

void rosElastic::run() {
    while(!pangolin::ShouldQuit() && !(!logReader->hasMore() && quiet) && !(eFusion->getTick() == end && quiet))
    {
        if(!gui->pause->Get() || pangolin::Pushed(*gui->step))
        {
            if((logReader->hasMore() || rewind) && eFusion->getTick() < end)
            {
                TICK("LogRead");
                if(rewind)
                {
                    if(!logReader->hasMore())
                    {
                        logReader->getBack();
                    }
                    else
                    {
                        logReader->getNext();
                    }

                    if(logReader->rewound())
                    {
                        logReader->currentFrame = 0;
                    }
                }
                else
                {
                    logReader->getNext();
                }
                TOCK("LogRead");

                if(eFusion->getTick() < start)
                {
                    eFusion->setTick(start);
                    logReader->fastForward(start);
                }

                float weightMultiplier = framesToSkip + 1;

                if(framesToSkip > 0)
                {
                    eFusion->setTick(eFusion->getTick() + framesToSkip);
                    logReader->fastForward(logReader->currentFrame + framesToSkip);
                    framesToSkip = 0;
                }

                Eigen::Matrix4f * currentPose = 0;

                if(gtOdometry)
                {
                    currentPose = new Eigen::Matrix4f;
                    currentPose->setIdentity();
                    *currentPose = gtOdometry->getTransformation(logReader->timestamp);
                }

                eFusion->processFrame(logReader->rgb,
                                      logReader->depth,
                                      logReader->timestamp,
                                      currentPose,
                                      weightMultiplier,
                                      false);
                if(currentPose)
                {
                    delete currentPose;
                }

                if(frameskip && Stopwatch::getInstance().getTimings().at("Run") > 1000.f / 30.f)
                {
                    framesToSkip = int(Stopwatch::getInstance().getTimings().at("Run") / (1000.f / 30.f));
                }
            }
        }
        else
        {
            eFusion->predict();
        }

        TICK("GUI");

        if(gui->followPose->Get())
        {
            pangolin::OpenGlMatrix mv;

            Eigen::Matrix4f currPose = eFusion->getCurrPose();
            Eigen::Matrix3f currRot = currPose.topLeftCorner(3, 3);

            Eigen::Quaternionf currQuat(currRot);
            Eigen::Vector3f forwardVector(0, 0, 1);
            Eigen::Vector3f upVector(0, iclnuim ? 1 : -1, 0);

            Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
            Eigen::Vector3f up = (currQuat * upVector).normalized();

            Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));

            eye -= forward;

            Eigen::Vector3f at = eye + forward;

            Eigen::Vector3f z = (eye - at).normalized();  // Forward
            Eigen::Vector3f x = up.cross(z).normalized(); // Right
            Eigen::Vector3f y = z.cross(x);

            Eigen::Matrix4d m;
            m <<    x(0),  x(1),  x(2),  -(x.dot(eye)),
                    y(0),  y(1),  y(2),  -(y.dot(eye)),
                    z(0),  z(1),  z(2),  -(z.dot(eye)),
                    0,     0,     0,              1;

            memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

            gui->s_cam.SetModelViewMatrix(mv);
        }

        gui->preCall();

        std::stringstream stri;
        stri << eFusion->getModelToModel().lastICPCount;
        gui->trackInliers->Ref().Set(stri.str());

        std::stringstream stre;
        stre << (std::isnan(eFusion->getModelToModel().lastICPError) ? 0 : eFusion->getModelToModel().lastICPError);
        gui->trackRes->Ref().Set(stre.str());

        if(!gui->pause->Get())
        {
            gui->resLog.Log((std::isnan(eFusion->getModelToModel().lastICPError) ? std::numeric_limits<float>::max() : eFusion->getModelToModel().lastICPError), icpErrThresh);
            gui->inLog.Log(eFusion->getModelToModel().lastICPCount, icpCountThresh);
        }

        Eigen::Matrix4f pose = eFusion->getCurrPose();

        if(gui->drawRawCloud->Get() || gui->drawFilteredCloud->Get())
        {
            eFusion->computeFeedbackBuffers();
        }

        if(gui->drawRawCloud->Get())
        {
            eFusion->getFeedbackBuffers().at(FeedbackBuffer::RAW)->render(gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
        }

        if(gui->drawFilteredCloud->Get())
        {
            eFusion->getFeedbackBuffers().at(FeedbackBuffer::FILTERED)->render(gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
        }

        if(gui->drawGlobalModel->Get())
        {
            glFinish();
            TICK("Global");

            if(gui->drawFxaa->Get())
            {
                gui->drawFXAA(gui->s_cam.GetProjectionModelViewMatrix(),
                              gui->s_cam.GetModelViewMatrix(),
                              eFusion->getGlobalModel().model(),
                              eFusion->getConfidenceThreshold(),
                              eFusion->getTick(),
                              eFusion->getTimeDelta(),
                              iclnuim);
            }
            else
            {
                eFusion->getGlobalModel().renderPointCloud(gui->s_cam.GetProjectionModelViewMatrix(),
                                                           eFusion->getConfidenceThreshold(),
                                                           gui->drawUnstable->Get(),
                                                           gui->drawNormals->Get(),
                                                           gui->drawColors->Get(),
                                                           gui->drawPoints->Get(),
                                                           gui->drawWindow->Get(),
                                                           gui->drawTimes->Get(),
                                                           eFusion->getTick(),
                                                           eFusion->getTimeDelta());
            }
            glFinish();
            TOCK("Global");
        }

        if(eFusion->getLost())
        {
            glColor3f(1, 1, 0);
        }
        else
        {
            glColor3f(1, 0, 1);
        }
        gui->drawFrustum(pose);
        glColor3f(1, 1, 1);

        if(gui->drawFerns->Get())
        {
            glColor3f(0, 0, 0);
            for(size_t i = 0; i < eFusion->getFerns().frames.size(); i++)
            {
                if((int)i == eFusion->getFerns().lastClosest)
                    continue;

                gui->drawFrustum(eFusion->getFerns().frames.at(i)->pose);
            }
            glColor3f(1, 1, 1);
        }

        if(gui->drawDefGraph->Get())
        {
            const std::vector<GraphNode*> & graph = eFusion->getLocalDeformation().getGraph();

            for(size_t i = 0; i < graph.size(); i++)
            {
                pangolin::glDrawCross(graph.at(i)->position(0),
                                      graph.at(i)->position(1),
                                      graph.at(i)->position(2),
                                      0.1);

                for(size_t j = 0; j < graph.at(i)->neighbours.size(); j++)
                {
                    pangolin::glDrawLine(graph.at(i)->position(0),
                                         graph.at(i)->position(1),
                                         graph.at(i)->position(2),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(0),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(1),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(2));
                }
            }
        }

        if(eFusion->getFerns().lastClosest != -1)
        {
            glColor3f(1, 0, 0);
            gui->drawFrustum(eFusion->getFerns().frames.at(eFusion->getFerns().lastClosest)->pose);
            glColor3f(1, 1, 1);
        }

        const std::vector<PoseMatch> & poseMatches = eFusion->getPoseMatches();

        int maxDiff = 0;
        for(size_t i = 0; i < poseMatches.size(); i++)
        {
            if(poseMatches.at(i).secondId - poseMatches.at(i).firstId > maxDiff)
            {
                maxDiff = poseMatches.at(i).secondId - poseMatches.at(i).firstId;
            }
        }

        for(size_t i = 0; i < poseMatches.size(); i++)
        {
            if(gui->drawDeforms->Get())
            {
                if(poseMatches.at(i).fern)
                {
                    glColor3f(1, 0, 0);
                }
                else
                {
                    glColor3f(0, 1, 0);
                }
                for(size_t j = 0; j < poseMatches.at(i).constraints.size(); j++)
                {
                    pangolin::glDrawLine(poseMatches.at(i).constraints.at(j).sourcePoint(0), poseMatches.at(i).constraints.at(j).sourcePoint(1), poseMatches.at(i).constraints.at(j).sourcePoint(2),
                                         poseMatches.at(i).constraints.at(j).targetPoint(0), poseMatches.at(i).constraints.at(j).targetPoint(1), poseMatches.at(i).constraints.at(j).targetPoint(2));
                }
            }
        }
        glColor3f(1, 1, 1);

        eFusion->normaliseDepth(0.3f, gui->depthCutoff->Get());

        for(std::map<std::string, GPUTexture*>::const_iterator it = eFusion->getTextures().begin(); it != eFusion->getTextures().end(); ++it)
        {
            if(it->second->draw)
            {
                gui->displayImg(it->first, it->second);
            }
        }

        eFusion->getIndexMap().renderDepth(gui->depthCutoff->Get());

        gui->displayImg("ModelImg", eFusion->getIndexMap().imageTex());
        gui->displayImg("Model", eFusion->getIndexMap().drawTex());

        std::stringstream strs;
        strs << eFusion->getGlobalModel().lastCount();
        gui->totalPoints->operator=(strs.str());

        std::stringstream strs2;
        strs2 << eFusion->getLocalDeformation().getGraph().size();
        gui->totalNodes->operator=(strs2.str());

        std::stringstream strs3;
        strs3 << eFusion->getFerns().frames.size();
        gui->totalFerns->operator=(strs3.str());

        std::stringstream strs4;
        strs4 << eFusion->getDeforms();
        gui->totalDefs->operator=(strs4.str());

        std::stringstream strs5;
        strs5 << eFusion->getTick() << "/" << logReader->getNumFrames();
        gui->logProgress->operator=(strs5.str());

        std::stringstream strs6;
        strs6 << eFusion->getFernDeforms();
        gui->totalFernDefs->operator=(strs6.str());

        gui->postCall();

        logReader->flipColors = gui->flipColors->Get();
        eFusion->setRgbOnly(gui->rgbOnly->Get());
        eFusion->setPyramid(gui->pyramid->Get());
        eFusion->setFastOdom(gui->fastOdom->Get());
        eFusion->setConfidenceThreshold(gui->confidenceThreshold->Get());
        eFusion->setDepthCutoff(gui->depthCutoff->Get());
        eFusion->setIcpWeight(gui->icpWeight->Get());
        eFusion->setSo3(gui->so3->Get());
        eFusion->setFrameToFrameRGB(gui->frameToFrameRGB->Get());

        resetButton = pangolin::Pushed(*gui->reset);

        if(gui->autoSettings)
        {
            static bool last = gui->autoSettings->Get();

            if(gui->autoSettings->Get() != last)
            {
                last = gui->autoSettings->Get();
                std::static_pointer_cast<LiveLogReader>(logReader)->setAuto(last);
            }
        }

        Stopwatch::getInstance().sendAll();

        if(resetButton)
        {
            break;
        }

        if(pangolin::Pushed(*gui->save))
        {
            eFusion->savePly();
        }

        TOCK("GUI");
    }
}

void rosElastic::global_run() {
    while(!pangolin::ShouldQuit() && !(!logReader->hasMore() && quiet) && !(eFusion->getTick() == end && quiet))
    {
        if(!gui->pause->Get() || pangolin::Pushed(*gui->step))
        {
            bool getGlobalRGB = false;
            if(eFusion->getTick() % globalTimeDiff == 0 && globalCamOn) getGlobalRGB = true;

            if((logReader->hasMore() || rewind) && eFusion->getTick() < end)
            {
                TICK("LogRead");
                if(rewind)
                {
                    if(!logReader->hasMore())
                    {
                        logReader->getBack();
                    }
                    else
                    {
                        logReader->getNext(getGlobalRGB);
                    }

                    if(logReader->rewound())
                    {
                        logReader->currentFrame = 0;
                    }
                }
                else
                {
                    logReader->getNext(getGlobalRGB);
                }
                TOCK("LogRead");

                if(eFusion->getTick() < start)
                {
                    eFusion->setTick(start);
                    logReader->fastForward(start);
                }

                float weightMultiplier = framesToSkip + 1;

                if(framesToSkip > 0)
                {
                    eFusion->setTick(eFusion->getTick() + framesToSkip);
                    logReader->fastForward(logReader->currentFrame + framesToSkip);
                    framesToSkip = 0;
                }

                Eigen::Matrix4f * currentPose = 0;

                if(gtOdometry)
                {
                    currentPose = new Eigen::Matrix4f;
                    currentPose->setIdentity();
                    *currentPose = gtOdometry->getTransformation(logReader->timestamp);
                }

                if (getGlobalRGB) {
                    eFusion->processFrame(logReader->rgb,
                                          logReader->depth,
                                          logReader->timestamp,
                                          currentPose,
                                          weightMultiplier,
                                          false,
                                          logReader->globalRGB[0]);
                }
                else {
                    eFusion->processFrame(logReader->rgb,
                                          logReader->depth,
                                          logReader->timestamp,
                                          currentPose,
                                          weightMultiplier,
                                          false);
                }


                if(currentPose)
                {
                    delete currentPose;
                }

                if(frameskip && Stopwatch::getInstance().getTimings().at("Run") > 1000.f / 30.f)
                {
                    framesToSkip = int(Stopwatch::getInstance().getTimings().at("Run") / (1000.f / 30.f));
                }
            }
        }
        else
        {
            eFusion->predict();
        }

        TICK("GUI");

        // calculate the position of camera
        if(gui->followPose->Get())
        {
            pangolin::OpenGlMatrix mv;

            Eigen::Matrix4f currPose = eFusion->getCurrPose();
            Eigen::Matrix3f currRot = currPose.topLeftCorner(3, 3);

            Eigen::Quaternionf currQuat(currRot);
            Eigen::Vector3f forwardVector(0, 0, 1);
            Eigen::Vector3f upVector(0, iclnuim ? 1 : -1, 0);

            Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
            Eigen::Vector3f up = (currQuat * upVector).normalized();

            Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));

            eye -= forward;

            Eigen::Vector3f at = eye + forward;

            Eigen::Vector3f z = (eye - at).normalized();  // Forward
            Eigen::Vector3f x = up.cross(z).normalized(); // Right
            Eigen::Vector3f y = z.cross(x);

            Eigen::Matrix4d m;
            m <<    x(0),  x(1),  x(2),  -(x.dot(eye)),
                    y(0),  y(1),  y(2),  -(y.dot(eye)),
                    z(0),  z(1),  z(2),  -(z.dot(eye)),
                    0,     0,     0,              1;

            memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

            gui->s_cam.SetModelViewMatrix(mv);
        }

        gui->preCall();

        std::stringstream stri;
        stri << eFusion->getModelToModel().lastICPCount;
        gui->trackInliers->Ref().Set(stri.str());

        std::stringstream stre;
        stre << (std::isnan(eFusion->getModelToModel().lastICPError) ? 0 : eFusion->getModelToModel().lastICPError);
        gui->trackRes->Ref().Set(stre.str());

        if(!gui->pause->Get())
        {
            gui->resLog.Log((std::isnan(eFusion->getModelToModel().lastICPError) ? std::numeric_limits<float>::max() : eFusion->getModelToModel().lastICPError), icpErrThresh);
            gui->inLog.Log(eFusion->getModelToModel().lastICPCount, icpCountThresh);
        }

        Eigen::Matrix4f pose = eFusion->getCurrPose();

        if(gui->drawRawCloud->Get() || gui->drawFilteredCloud->Get())
        {
            eFusion->computeFeedbackBuffers();
        }

        if(gui->drawRawCloud->Get())
        {
            eFusion->getFeedbackBuffers().at(FeedbackBuffer::RAW)->render(gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
        }

        if(gui->drawFilteredCloud->Get())
        {
            eFusion->getFeedbackBuffers().at(FeedbackBuffer::FILTERED)->render(gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
        }

        if(gui->drawGlobalModel->Get())
        {
            glFinish();
            TICK("Global");

            if(gui->drawFxaa->Get())
            {
                gui->drawFXAA(gui->s_cam.GetProjectionModelViewMatrix(),
                              gui->s_cam.GetModelViewMatrix(),
                              eFusion->getGlobalModel().model(),
                              eFusion->getConfidenceThreshold(),
                              eFusion->getTick(),
                              eFusion->getTimeDelta(),
                              iclnuim);
            }
            else
            {
                eFusion->getGlobalModel().renderPointCloud(gui->s_cam.GetProjectionModelViewMatrix(),
                                                           eFusion->getConfidenceThreshold(),
                                                           gui->drawUnstable->Get(),
                                                           gui->drawNormals->Get(),
                                                           gui->drawColors->Get(),
                                                           gui->drawPoints->Get(),
                                                           gui->drawWindow->Get(),
                                                           gui->drawTimes->Get(),
                                                           eFusion->getTick(),
                                                           eFusion->getTimeDelta());
            }
            glFinish();
            TOCK("Global");
        }

        if(eFusion->getLost())
        {
            glColor3f(1, 1, 0);
        }
        else
        {
            glColor3f(1, 0, 1);
        }

        gui->drawFrustum(pose);
        glColor3f(1, 1, 1);

        if(gui->drawFerns->Get())
        {
            glColor3f(0, 0, 0);
            for(size_t i = 0; i < eFusion->getFerns().frames.size(); i++)
            {
                if((int)i == eFusion->getFerns().lastClosest)
                    continue;

                gui->drawFrustum(eFusion->getFerns().frames.at(i)->pose);
            }
            glColor3f(1, 1, 1);
        }

        if(gui->drawDefGraph->Get())
        {
            const std::vector<GraphNode*> & graph = eFusion->getLocalDeformation().getGraph();

            for(size_t i = 0; i < graph.size(); i++)
            {
                pangolin::glDrawCross(graph.at(i)->position(0),
                                      graph.at(i)->position(1),
                                      graph.at(i)->position(2),
                                      0.1);

                for(size_t j = 0; j < graph.at(i)->neighbours.size(); j++)
                {
                    pangolin::glDrawLine(graph.at(i)->position(0),
                                         graph.at(i)->position(1),
                                         graph.at(i)->position(2),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(0),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(1),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(2));
                }
            }
        }

        if(eFusion->getFerns().lastClosest != -1)
        {
            glColor3f(1, 0, 0);
            gui->drawFrustum(eFusion->getFerns().frames.at(eFusion->getFerns().lastClosest)->pose);
            glColor3f(1, 1, 1);
        }

        const std::vector<PoseMatch> & poseMatches = eFusion->getPoseMatches();

        int maxDiff = 0;
        for(size_t i = 0; i < poseMatches.size(); i++)
        {
            if(poseMatches.at(i).secondId - poseMatches.at(i).firstId > maxDiff)
            {
                maxDiff = poseMatches.at(i).secondId - poseMatches.at(i).firstId;
            }
        }

        for(size_t i = 0; i < poseMatches.size(); i++)
        {
            if(gui->drawDeforms->Get())
            {
                if(poseMatches.at(i).fern)
                {
                    glColor3f(1, 0, 0);
                }
                else
                {
                    glColor3f(0, 1, 0);
                }
                for(size_t j = 0; j < poseMatches.at(i).constraints.size(); j++)
                {
                    pangolin::glDrawLine(poseMatches.at(i).constraints.at(j).sourcePoint(0), poseMatches.at(i).constraints.at(j).sourcePoint(1), poseMatches.at(i).constraints.at(j).sourcePoint(2),
                                         poseMatches.at(i).constraints.at(j).targetPoint(0), poseMatches.at(i).constraints.at(j).targetPoint(1), poseMatches.at(i).constraints.at(j).targetPoint(2));
                }
            }
        }
        glColor3f(1, 1, 1);

        eFusion->normaliseDepth(0.3f, gui->depthCutoff->Get());

        for(std::map<std::string, GPUTexture*>::const_iterator it = eFusion->getTextures().begin(); it != eFusion->getTextures().end(); ++it)
        {
            if(it->second->draw)
            {
                gui->displayImg(it->first, it->second);
            }
        }

//        eFusion->getIndexMap().renderDepth(gui->depthCutoff->Get());
//        gui->displayImg("Model", eFusion->getIndexMap().drawTex());

        gui->displayImg("ModelImg", eFusion->getIndexMap().imageTex());

        std::stringstream strs;
        strs << eFusion->getGlobalModel().lastCount();
        gui->totalPoints->operator=(strs.str());

        std::stringstream strs2;
        strs2 << eFusion->getLocalDeformation().getGraph().size();
        gui->totalNodes->operator=(strs2.str());

        std::stringstream strs3;
        strs3 << eFusion->getFerns().frames.size();
        gui->totalFerns->operator=(strs3.str());

        std::stringstream strs4;
        strs4 << eFusion->getDeforms();
        gui->totalDefs->operator=(strs4.str());

        std::stringstream strs5;
        strs5 << eFusion->getTick() << "/" << logReader->getNumFrames();
        gui->logProgress->operator=(strs5.str());

        std::stringstream strs6;
        strs6 << eFusion->getFernDeforms();
        gui->totalFernDefs->operator=(strs6.str());

        gui->postCall();

        logReader->flipColors = gui->flipColors->Get();
        eFusion->setRgbOnly(gui->rgbOnly->Get());
        eFusion->setPyramid(gui->pyramid->Get());
        eFusion->setFastOdom(gui->fastOdom->Get());
        eFusion->setConfidenceThreshold(gui->confidenceThreshold->Get());
        eFusion->setDepthCutoff(gui->depthCutoff->Get());
        eFusion->setIcpWeight(gui->icpWeight->Get());
        eFusion->setSo3(gui->so3->Get());
        eFusion->setFrameToFrameRGB(gui->frameToFrameRGB->Get());

        resetButton = pangolin::Pushed(*gui->reset);

        if(gui->autoSettings)
        {
            static bool last = gui->autoSettings->Get();

            if(gui->autoSettings->Get() != last)
            {
                last = gui->autoSettings->Get();
                std::static_pointer_cast<LiveLogReader>(logReader)->setAuto(last);
            }
        }

        Stopwatch::getInstance().sendAll();

        if(resetButton)
        {
            break;
        }

        if(pangolin::Pushed(*gui->save))
        {
            eFusion->savePly();
        }

        TOCK("GUI");
    }
}


void rosElastic::test_GUI() {
    while(!pangolin::ShouldQuit() && (logReader->hasMore() || !quiet) )
    {
        std::cout<< "testing pangolin" <<std::endl;
        if(!gui->pause->Get() || pangolin::Pushed(*gui->step)) {
            logReader->getNext(globalCamOn);
        }

        if(gui->followPose->Get()) {
            pangolin::OpenGlMatrix mv;

            Eigen::Matrix4f currPose ;
            currPose <<         1,  0,  0,  0,
                                0,  1,  0,  0,
                                0,  0,  1,  0,
                                0,  0,  1,  1;
            Eigen::Matrix3f currRot = currPose.topLeftCorner(3, 3);

            Eigen::Quaternionf currQuat(currRot);
            Eigen::Vector3f forwardVector(0, 0, 1);
            Eigen::Vector3f upVector(0, iclnuim ? 1 : -1, 0);

            Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
            Eigen::Vector3f up = (currQuat * upVector).normalized();

            Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));

            eye -= forward;

            Eigen::Vector3f at = eye + forward;

            Eigen::Vector3f z = (eye - at).normalized();  // Forward
            Eigen::Vector3f x = up.cross(z).normalized(); // Right
            Eigen::Vector3f y = z.cross(x);

            Eigen::Matrix4d m;
            m <<    x(0),  x(1),  x(2),  -(x.dot(eye)),
                    y(0),  y(1),  y(2),  -(y.dot(eye)),
                    z(0),  z(1),  z(2),  -(z.dot(eye)),
                    0,     0,     0,              1;

            memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));
            gui->s_cam.SetModelViewMatrix(mv);
        }

        gui->preCall();
//        std::stringstream stri;
//        stri << "test string i";
//        gui->trackInliers->Ref().Set(stri.str());
////        glColor3f(1, 1, 0);
//        std::stringstream stre;
//        stre << "test string e";
//        gui->trackRes->Ref().Set(stre.str());
//        gui->resLog.Log(0.0002, 0.00001);
//        gui->inLog.Log(20000, 30000);

//        eFusion->getIndexMap().renderDepth(gui->depthCutoff->Get());
        std::map<std::string, GPUTexture*> textures;
        textures[GPUTexture::RGB] = new GPUTexture(LocalCameraInfo::getInstance().width(),
                                                   LocalCameraInfo::getInstance().height(),
                                                   GL_RGBA,
                                                   GL_RGB,
                                                   GL_UNSIGNED_BYTE,
                                                   true,
                                                   true);
        textures[GPUTexture::RGB]->texture->Upload(logReader->rgb, GL_RGB, GL_UNSIGNED_BYTE);


        textures[GPUTexture::DEPTH_RAW] = new GPUTexture(LocalCameraInfo::getInstance().width(),
                                                         LocalCameraInfo::getInstance().height(),
                                                         GL_LUMINANCE16UI_EXT,
                                                         GL_LUMINANCE_INTEGER_EXT,
                                                         GL_UNSIGNED_SHORT);
        textures[GPUTexture::DEPTH_NORM] = new GPUTexture(LocalCameraInfo::getInstance().width(),
                                                          LocalCameraInfo::getInstance().height(),
                                                          GL_LUMINANCE,
                                                          GL_LUMINANCE,
                                                          GL_FLOAT,
                                                          true);
        textures[GPUTexture::DEPTH_RAW]->texture->Upload(logReader->depth, GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_SHORT);
        std::map<std::string, ComputePack*> computePacks;
        computePacks[ComputePack::NORM] = new ComputePack(loadProgramFromFile("empty.vert", "depth_norm.frag", "quad.geom"),
                                                          textures[GPUTexture::DEPTH_NORM]->texture);
        std::vector<Uniform> uniforms;
        uniforms.push_back(Uniform("maxVal", gui->depthCutoff->Get() * 1000.f));
        uniforms.push_back(Uniform("minVal", 0.3f * 1000.f));
        computePacks[ComputePack::NORM]->compute(textures[GPUTexture::DEPTH_RAW]->texture, &uniforms);


        textures[GPUTexture::GLOBAL_RAW] = new GPUTexture(GlobalCamInfo::getInstance().width(),
                                                          GlobalCamInfo::getInstance().height(),
                                                   GL_RGBA,
                                                   GL_RGB,
                                                   GL_UNSIGNED_BYTE,
                                                   true,
                                                   true);
        textures[GPUTexture::GLOBAL_RAW]->texture->Upload(logReader->globalRGB[0], GL_RGB, GL_UNSIGNED_BYTE);

        gui->displayImg(GPUTexture::RGB, textures[GPUTexture::RGB]);
        gui->displayImg(GPUTexture::DEPTH_NORM, textures[GPUTexture::DEPTH_NORM]);
        gui->displayImg(GPUTexture::GLOBAL_RAW, textures[GPUTexture::GLOBAL_RAW]);

        gui->postCall();

        std::cout<<""<<std::endl;
    }
}

