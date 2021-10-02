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

#include "RawLogReader.h"

void showImage(unsigned char * image, bool rgb) {
    cv::Mat newimage;
    if (rgb) {
        newimage= cv::Mat(480,640,CV_8UC3);
        memcpy(newimage.data, &image[0], 640*480*3);
    }
    else {
        newimage= cv::Mat(480,640,CV_16UC1);
        memcpy(newimage.data, &image[0], 640*480*2);
    }
    cv::namedWindow("rgb",cv::WINDOW_AUTOSIZE);
    cv::imshow("rgb",newimage);
    cv::waitKey(0);
}


RawLogReader::RawLogReader(std::string file, bool flipColors, bool glc, bool realsense)
 : LogReader(file, flipColors, glc), realsense(realsense)
{
    currentFrame = 0;

    depthReadBuffer = new unsigned char[numPixels * 2];
    imageReadBuffer = new unsigned char[numPixels * 3];
    decompressionBufferDepth = new Bytef[numPixels* 2];
    decompressionBufferImage = new Bytef[numPixels * 3];

    if(realsense) {
        if (useGlobalCam){
            std::string globalfile = file;
            globalfile.append("oncefile.ply");
            assert(pangolin::FileExists(globalfile.c_str()));
            recordFile.open(globalfile.c_str());
            recordFile.read(reinterpret_cast<char *>(&timestamp), sizeof (int64_t));
            for (int i=0; i<GlobalCamInfo::getInstance().camNumber(); i++) {
                decompressionBufferGlobalImages.push_back(new Bytef[GlobalCamInfo::getInstance().numPixels() * 3]);
                globalRGB.push_back(nullptr);
                recordFile.read(reinterpret_cast<char *>(&decompressionBufferGlobalImages[i][0]), GlobalCamInfo::getInstance().numPixels() * 3);
                globalRGB[i] = (unsigned char *)&decompressionBufferGlobalImages[i][0];
            }
            recordFile.close();
        }
        file.append("walkingfiles.ply");
        assert(pangolin::FileExists(file.c_str()));
        recordFile.open(file.c_str());
    } else {
        assert(pangolin::FileExists(file.c_str()));
        fp = fopen(file.c_str(), "rb");
        auto tmp = fread(&numFrames,sizeof(int32_t),1,fp);
        assert(tmp);
    }

}

RawLogReader::~RawLogReader()
{
    delete [] depthReadBuffer;
    delete [] imageReadBuffer;
    delete [] decompressionBufferDepth;
    delete [] decompressionBufferImage;

    fclose(fp);
}

void RawLogReader::getBack()
{
    assert(filePointers.size() > 0);

    fseek(fp, filePointers.top(), SEEK_SET);

    filePointers.pop();

    getCore();
}

void RawLogReader::getNext(bool gRGB)
{
    if (realsense) getRealsense();
    else {
        filePointers.push(ftell(fp));
        getCore();
    }
}

void RawLogReader::getRealsense() {
    recordFile.read(reinterpret_cast<char *>(&timestamp), sizeof(int64_t));
    recordFile.read(reinterpret_cast<char *>(&decompressionBufferImage[0]), LocalCameraInfo::getInstance().numPixels()*3);
    recordFile.read(reinterpret_cast<char *>(&decompressionBufferDepth[0]), LocalCameraInfo::getInstance().numPixels()*2);

//    showImage(decompressionBufferImage, true);
//    showImage(decompressionBufferDepth, false);

    depth = (unsigned short *)decompressionBufferDepth;
    rgb = (unsigned char *)decompressionBufferImage;

    if(flipColors)
    {
        for(int i = 0; i < LocalCameraInfo::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }

    currentFrame++;
}

void RawLogReader::getCore()
{
    auto tmp = fread(&timestamp,sizeof(int64_t),1,fp);
    assert(tmp);

    tmp = fread(&depthSize,sizeof(int32_t),1,fp);
    assert(tmp);
    tmp = fread(&imageSize,sizeof(int32_t),1,fp);
    assert(tmp);
//    std::cout<<depthSize<<"/"<<imageSize<<std::endl;

    tmp = fread(depthReadBuffer,depthSize,1,fp);
    assert(tmp);

    if(imageSize > 0)
    {
        tmp = fread(imageReadBuffer,imageSize,1,fp);
        assert(tmp);
    }

    if(depthSize == numPixels * 2)
    {
        memcpy(&decompressionBufferDepth[0], depthReadBuffer, numPixels * 2);
    }
    else
    {
        unsigned long decompLength = numPixels * 2;
        uncompress(decompressionBufferDepth, (unsigned long *)&decompLength, (const Bytef *)depthReadBuffer, depthSize);
    }

//    auto dpeth_image = &decompressionBufferDepth[0];
//    std::cout << "show depth decomp" << std::endl;

    if(imageSize == numPixels * 3)
    {
        memcpy(&decompressionBufferImage[0], imageReadBuffer, numPixels * 3);
    }
    else if(imageSize > 0)
    {
        jpeg.readData(imageReadBuffer, imageSize, (unsigned char *)&decompressionBufferImage[0]);
    }
    else
    {
        memset(&decompressionBufferImage[0], 0, numPixels * 3);
    }

    depth = (unsigned short *)&decompressionBufferDepth[0];
    rgb = (unsigned char *)&decompressionBufferImage[0];

//    auto depth_image_depth = *depth;
//    auto rgb_image = *rgb;
//    std::cout << "show depth" << std::endl;

    if(flipColors)
    {
        for(int i = 0; i < LocalCameraInfo::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }

    currentFrame++;
}

void RawLogReader::fastForward(int frame)
{
    while(currentFrame < frame && hasMore())
    {
        if (realsense) {
            recordFile.read(reinterpret_cast<char *>(&timestamp), sizeof(int64_t));
            recordFile.read(reinterpret_cast<char *>(&decompressionBufferImage[0]), LocalCameraInfo::getInstance().numPixels()*2);
            recordFile.read(reinterpret_cast<char *>(&decompressionBufferDepth[0]), LocalCameraInfo::getInstance().numPixels()*3);
        }
        else {
            filePointers.push(ftell(fp));
            auto tmp = fread(&timestamp,sizeof(int64_t),1,fp);
            assert(tmp);

            tmp = fread(&depthSize,sizeof(int32_t),1,fp);
            assert(tmp);
            tmp = fread(&imageSize,sizeof(int32_t),1,fp);
            assert(tmp);

            tmp = fread(depthReadBuffer,depthSize,1,fp);
            assert(tmp);

            if(imageSize > 0)
            {
                tmp = fread(imageReadBuffer,imageSize,1,fp);
                assert(tmp);
            }
        }

        currentFrame++;
    }
}

int RawLogReader::getNumFrames()
{
    return numFrames;
}

bool RawLogReader::hasMore()
{
    if(realsense) return recordFile.peek()!=EOF;
    else return currentFrame + 1 < numFrames;
}


void RawLogReader::rewind()
{
    if (realsense) {}
    else{
        if (filePointers.size() != 0)
        {
            std::stack<int> empty;
            std::swap(empty, filePointers);
        }

        fclose(fp);
        fp = fopen(file.c_str(), "rb");

        auto tmp = fread(&numFrames,sizeof(int32_t),1,fp);
        assert(tmp);
    }

    currentFrame = 0;
}

bool RawLogReader::rewound()
{
    if (realsense) return currentFrame == 0;
    else return filePointers.size() == 0;
}

const std::string RawLogReader::getFile()
{
    return file;
}

void RawLogReader::setAuto(bool value)
{

}
