//
// Created by jing on 9/26/21.
//

#ifndef COSLAM_GLOBALCAMTEXTURE_H
#define COSLAM_GLOBALCAMTEXTURE_H

#include "Shaders/Shaders.h"
#include "Shaders/Uniform.h"
#include "../Utils/LocalCameraInfo.h"
#include "../Utils/GlobalCamInfo.h"
#include "../Core/GPUTexture.h"

class GlobalCamTexture
{
public:
    GlobalCamTexture(): imageTexture(GlobalCamInfo::getInstance().width(),
                                     GlobalCamInfo::getInstance().height(),
                                     GL_RGBA,
                                     GL_RGB,
                                     GL_UNSIGNED_BYTE,
                                     false,
                                     true),
                        imageProgram(loadProgramFromFile("empty.vert", "global_rgb.frag", "quad.geom")),
                        imageRenderBuffer(GlobalCamInfo::getInstance().width(), GlobalCamInfo::getInstance().height())
    {
        imageFrameBuffer.AttachColour(*imageTexture.texture);
        imageFrameBuffer.AttachDepth(imageRenderBuffer);
    };

    virtual ~GlobalCamTexture(){};

    void image(GPUTexture * existingRgb){
        imageFrameBuffer.Bind();

        glPushAttrib(GL_VIEWPORT_BIT);

        glViewport(0, 0, imageRenderBuffer.width, imageRenderBuffer.height);

        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        imageProgram->Bind();

        imageProgram->setUniform(Uniform("eSampler", 0));

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, existingRgb->texture->tid);

        glDrawArrays(GL_POINTS, 0, 1);

        imageFrameBuffer.Unbind();

        glBindTexture(GL_TEXTURE_2D, 0);

        glActiveTexture(GL_TEXTURE0);

        imageProgram->Unbind();

        glPopAttrib();

        glFinish();
    }

    GPUTexture * imageTex()
    {
        return &imageTexture;
    }

    GPUTexture imageTexture;

    std::shared_ptr<Shader> imageProgram;           // sample data if the value is 0 or passthrough is 1, resample
    pangolin::GlRenderBuffer imageRenderBuffer;     //
    pangolin::GlFramebuffer imageFrameBuffer;
};

#endif //COSLAM_GLOBALCAMTEXTURE_H
