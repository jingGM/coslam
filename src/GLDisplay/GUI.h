/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef GUI_H_
#define GUI_H_

#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <map>

#include "../Core/GPUTexture.h"
#include "Shaders/Shaders.h"
#include "Shaders/Vertex.h"

#include "../Utils/LocalCameraInfo.h"


#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

class GUI
{
    public:
        GUI(bool liveCap, bool showcaseMode);
        void createPanel();
        void createPlotter();
        void createImages();
        void createMainScreen();

        virtual ~GUI(){
            delete pause;
            delete reset;
            delete inPlot;
            delete resPlot;

            if(autoSettings)
            {
                delete autoSettings;

            }
            delete step;
            delete save;
            delete trackInliers;
            delete trackRes;
            delete confidenceThreshold;
            delete totalNodes;
            delete drawWindow;
            delete so3;
            delete totalFerns;
            delete totalDefs;
            delete depthCutoff;
            delete logProgress;
            delete drawTimes;
            delete drawFxaa;
            delete fastOdom;
            delete icpWeight;
            delete pyramid;
            delete rgbOnly;
            delete totalFernDefs;
            delete drawFerns;
            delete followPose;
            delete drawDeforms;
            delete drawRawCloud;
            delete totalPoints;
            delete frameToFrameRGB;
            delete flipColors;
            delete drawFilteredCloud;
            delete drawNormals;
            delete drawColors;
            delete drawGlobalModel;
            delete drawUnstable;
            delete drawPoints;
            delete drawDefGraph;
            delete gpuMem;

            delete renderBuffer;
            delete colorFrameBuffer;
            delete colorTexture;
        };

        void preCall();

        void drawFrustum(const Eigen::Matrix4f & pose);

        void displayImg(const std::string & id, GPUTexture * img);

        void postCall();

        void drawFXAA(pangolin::OpenGlMatrix mvp,
                      pangolin::OpenGlMatrix mv,
                      const std::pair<GLuint, GLuint> & model,
                      const float threshold,
                      const int time,
                      const int timeDelta,
                      const bool invertNormals);

        bool showcaseMode;
        int width;
        int height;
        int panel;

        pangolin::Var<bool> * pause,
                            * step,
                            * save,
                            * reset,
                            * flipColors,
                            * rgbOnly,
                            * pyramid,
                            * so3,
                            * frameToFrameRGB,
                            * fastOdom,
                            * followPose,
                            * drawRawCloud,
                            * drawFilteredCloud,
                            * drawNormals,
                            * autoSettings,
                            * drawDefGraph,
                            * drawColors,
                            * drawFxaa,
                            * drawGlobalModel,
                            * drawUnstable,
                            * drawPoints,
                            * drawTimes,
                            * drawFerns,
                            * drawDeforms,
                            * drawWindow;
        pangolin::Var<int> * gpuMem;
        pangolin::Var<std::string> * totalPoints,
                                   * totalNodes,
                                   * totalFerns,
                                   * totalDefs,
                                   * totalFernDefs,
                                   * trackInliers,
                                   * trackRes,
                                   * logProgress;
        pangolin::Var<float> * confidenceThreshold,
                             * depthCutoff,
                             * icpWeight;

        pangolin::DataLog resLog, inLog;
        pangolin::Plotter * resPlot,
                          * inPlot;

        pangolin::OpenGlRenderState s_cam;

        pangolin::GlRenderBuffer * renderBuffer;
        pangolin::GlFramebuffer * colorFrameBuffer;
        GPUTexture * colorTexture;
        std::shared_ptr<Shader> colorProgram;
        std::shared_ptr<Shader> fxaaProgram;
};

typedef std::shared_ptr<GUI> GUIPtr;
#endif /* GUI_H_ */
