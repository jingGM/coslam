#include <iostream>
#include "src/rosElastic.h"

int main(int argc, char *argv[])
{
    rosElastic roselastic(argc, argv);

//    roselastic.run();
//    roselastic.test_GUI();
    roselastic.test_run();
    return 0;
}