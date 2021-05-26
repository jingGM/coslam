//
// Created by jing on 4/21/21.
//
//#include "GUI/src/MainController.h"
#include "rosElastic.h"


int main(int argc, char *argv[])
{
    rosElastic roselastic(argc, argv);

//    roselastic.runSample(argc, argv);
    roselastic.run();

    return 0;
}