//
// Created by jing on 9/26/21.
//
#include <memory>
#include <fstream>
#include <vector>
#include <iostream>

int main(int argc, char *argv[])
{
    std::string filename = "test";
    filename.append(".ply");
    remove(filename.c_str());
    std::ofstream fpout (filename.c_str (), std::ios::app | std::ios::binary);

    std::vector<double> testvalue = {1,2,3,4};
    fpout.write (reinterpret_cast<const char*> (&testvalue[0]), 4 * sizeof (double));
    double testv = 5;
    fpout.write (reinterpret_cast<const char*> (&testv), sizeof (double));
    fpout.close();

    std::ifstream recordFile;
    recordFile.open(filename.c_str());
//    std::vector<double> readvalue = {};
    while (recordFile.peek()!=EOF) {
        double x;
        recordFile.read(reinterpret_cast<char *>(&x), sizeof(double));
        std::cout<<x<<std::endl;
    }
    recordFile.close();
//    globalCamPoses.push_back({x,y,z,r,p,ya});
}