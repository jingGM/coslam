//
// Created by jing on 5/22/21.
//

#include "../Tools/RawLogReader.h"
#include <ElasticFusion.h>

int main(int argc, char *argv[])
{
    auto logFile = "/home/jing/Documents/catkinws/slam/src/coslam/dataset/dyson_lab.klg";
    bool flipColors = false;
    int framesToSkip = 0;

    Resolution::getInstance(640, 480);
    Intrinsics::getInstance(528, 528, 320, 240);

    LogReaderPtr logReader=std::make_shared<RawLogReader>(logFile, flipColors);
    logReader->rewind();
    auto logfile = logReader->getFile();

    int frames = 0;
    while(frames < 10) {
        logReader->getNext();
        logReader->fastForward(logReader->currentFrame + framesToSkip);

        auto rgb = logReader->rgb;
        auto depth = logReader->depth;
        auto timestamp = logReader->timestamp;
        frames +=1;
    }

    return 0;
}