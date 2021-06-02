//
// Created by jing on 5/22/21.
//

#include "../Tools/RawLogReader.h"
#include <ElasticFusion.h>
#include "../rosinterface/dataInterface.h"
#include "../rosinterface/rosInterface.h"

void testRawLogger() {
    std::string logFile = "/home/jing/Documents/catkinws/slam/src/coslam/dataset/dyson_lab.klg";
    bool flipColors = false;
    int framesToSkip = 0;

    LogReaderPtr logReader=std::make_shared<RawLogReader>(logFile, flipColors);
    logReader->rewind();
    auto logfile = logReader->getFile();

    int frames = 0;
    while(frames < 5) {
        logReader->getNext();
        logReader->fastForward(logReader->currentFrame + framesToSkip);

        auto rgb = logReader->rgb;
        auto depth = logReader->depth;
        auto timestamp = logReader->timestamp;
        frames +=1;
    }
}


void testROSLogger(int argc, char *argv[]) {
    ros::init(argc, argv, "ros_efusion");

    dataInterfacePtr dataInferPtr = std::make_shared<DataInterface>(4);
    rosInterfacePtr rosInterPtr = std::make_shared<rosInterface>(dataInferPtr, 4);

    ros::Rate rate(2.);
    while(ros::ok())
    {
        cv::Mat depth = dataInferPtr->getDepth();

        rate.sleep();
    }
}

void testPangolinFile() {
    int numPixels = Resolution::getInstance().width() * Resolution::getInstance().height();
    FILE * fp;
    int32_t numFrames;
    int64_t timestamp;
    unsigned short * depth;
    unsigned char * rgb;
    int32_t depthSize;
    int32_t imageSize;
    int currentFrame;
    unsigned char * depthReadBuffer;
    unsigned char * imageReadBuffer;
    depthReadBuffer = new unsigned char[numPixels * 2];
    imageReadBuffer = new unsigned char[numPixels * 3];
    Bytef * decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];

    std::string file = "/home/jing/Documents/catkinws/slam/src/coslam/dataset/dyson_lab.klg";
    assert(pangolin::FileExists(file));
    fp = fopen(file.c_str(), "rb");
    auto tmp = fread(&numFrames,sizeof(int32_t),1,fp);

    currentFrame = 0;

    ftell(fp);

    tmp = fread(&timestamp,sizeof(int64_t),1,fp);
    assert(tmp);
    tmp = fread(&depthSize,sizeof(int32_t),1,fp);
    assert(tmp);
    tmp = fread(&imageSize,sizeof(int32_t),1,fp);
    assert(tmp);

    tmp = fread(depthReadBuffer,depthSize,1,fp);
    assert(tmp);

    unsigned long decompLength = numPixels * 2;
    uncompress(&decompressionBufferDepth[0], (unsigned long *)&decompLength, (const Bytef *)depthReadBuffer, depthSize);

    auto depth_show = (unsigned short *)decompressionBufferDepth;
//    auto depth_bind = depth_show;


    pangolin::GlTexture * texture = new pangolin::GlTexture(Resolution::getInstance().width(),
                                                            Resolution::getInstance().height(),
                                                            GL_LUMINANCE,
                                                            true, 0, GL_LUMINANCE, GL_FLOAT);
    texture->Upload(depth_show, GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_SHORT);
//    texture->Save("local_unbind.jpg");
    texture->Bind();
//    texture->Save("local_bind.jpg");
    std::cout<<"depth"<<std::endl;
}

int main(int argc, char *argv[])
{

    Resolution::getInstance(640, 480);
    Intrinsics::getInstance(528, 528, 320, 240);
//    testRawLogger();
    testROSLogger(argc, argv);
//    testPangolinFile();
    return 0;
}