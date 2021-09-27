//
// Created by jing on 9/26/21.
//

#ifndef COSLAM_GLOBALCAMTEXTURE_H
#define COSLAM_GLOBALCAMTEXTURE_H

#include "Shaders/Shaders.h"
#include "Shaders/Uniform.h"
#include "../Utils/LocalCameraInfo.h"
#include "../Core/GPUTexture.h"

class GlobalCamTexture
{
public:
    GlobalCamTexture(): imageTexture(LocalCameraInfo::getInstance().width(),
                                     LocalCameraInfo::getInstance().height(),
                                     GL_RGBA,
                                     GL_RGB,
                                     GL_UNSIGNED_BYTE,
                                     false,
                                     true),
                        imageProgram(loadProgramFromFile("empty.vert", "fill_rgb.frag", "quad.geom")),
                        imageRenderBuffer(LocalCameraInfo::getInstance().width(), LocalCameraInfo::getInstance().height())
    {
        imageFrameBuffer.AttachColour(*imageTexture.texture);
        imageFrameBuffer.AttachDepth(imageRenderBuffer);
    };

    virtual ~GlobalCamTexture(){};

    void image(GPUTexture * existingRgb, GPUTexture * rawRgb, bool passthrough);

    GPUTexture imageTexture;

    std::shared_ptr<Shader> imageProgram;           // sample data if the value is 0 or passthrough is 1, resample
    pangolin::GlRenderBuffer imageRenderBuffer;     //
    pangolin::GlFramebuffer imageFrameBuffer;
};

void GlobalCamTexture::image(GPUTexture *existingRgb, GPUTexture *rawRgb, bool passthrough) {

}

#endif //COSLAM_GLOBALCAMTEXTURE_H
