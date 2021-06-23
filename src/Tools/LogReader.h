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

#ifndef LOGREADER_H_
#define LOGREADER_H_

#ifdef WIN32
#  include <cstdint>
#endif
#include <string>
#include <vector>

#if (defined WIN32) && (defined FAR)
#  undef FAR
#endif
#include <zlib.h>
#ifndef WIN32
#  include <poll.h>
#endif
#include <memory>
#include <Utils/Img.h>
#include <Utils/Resolution.h>
#include <Utils/GlobalCamInfo.h>

#include "JPEGLoader.h"

class LogReader
{
    public:
        LogReader(std::string file, bool flipColors, bool glc)
         : flipColors(flipColors),
           useGlobalCam(glc),
           timestamp(0),
           depth(0),
           rgb(0),
           currentFrame(0),
           decompressionBufferDepth(0),
           decompressionBufferImage(0),
           file(file),
//           width(),
//           height(),
           numPixels(Resolution::getInstance().width() * Resolution::getInstance().height())
        {}

        virtual ~LogReader()
        {}

        virtual void getNext(bool gRGB=false) = 0;

        virtual int getNumFrames() = 0;

        virtual bool hasMore() = 0;

        virtual bool rewound() = 0;

        virtual void rewind() = 0;

        virtual void getBack() = 0;

        virtual void fastForward(int frame) = 0;

        virtual const std::string getFile() = 0;

        virtual void setAuto(bool value) = 0;

        bool flipColors;
        bool useGlobalCam;
        int64_t timestamp;

        unsigned short * depth;
        unsigned char * rgb;
        std::vector<unsigned char *> globalRGB;
        int currentFrame;

    protected:
        Bytef * decompressionBufferDepth;
        Bytef * decompressionBufferImage;
        std::vector<Bytef *> decompressionBufferGlobalImages = {};
        unsigned char * depthReadBuffer;
        unsigned char * imageReadBuffer;
        std::vector<unsigned char *> globalImageReadBuffer = {};
        int32_t depthSize;
        int32_t imageSize;
        int32_t globalImageSize;

        const std::string file;
        FILE * fp;
        int32_t numFrames;
//        int width;
//        int height;
        int numPixels;

        JPEGLoader jpeg;
};
typedef std::shared_ptr<LogReader> LogReaderPtr;
#endif /* LOGREADER_H_ */
