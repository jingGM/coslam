#apt install sudo
#sudo apt-get install wget software-properties-common

wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.2.89-1_amd64.deb
sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
sudo dpkg -i cuda-repo-ubuntu1804_10.2.89-1_amd64.deb
rm cuda-repo-ubuntu1804_10.2.89-1_amd64.deb
sudo add-apt-repository ppa:openjdk-r/ppa 
sudo apt-get update
sudo apt-get install cuda-10-2

sudo apt-get install -y cmake-qt-gui git build-essential libusb-1.0-0-dev libudev-dev openjdk-11-* freeglut3-dev libglew-dev libsuitesparse-dev libeigen3-dev zlib1g-dev libjpeg-dev


git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ../ -DAVFORMAT_INCLUDE_DIR="" -DCPP11_NO_BOOST=ON
make -j8
cd ../..

git clone https://github.com/occipital/OpenNI2.git
cd OpenNI2
make -j8
cd ..

cd ../Core
mkdir build
cd build
cmake ../src
make -j8
cd ../../GPUTest
mkdir build
cd build
cmake ../src
make -j8
cd ../../GUI
mkdir build
cd build
cmake ../src
make -j8
