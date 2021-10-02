## set up instructions:

#### setup ros environment:
follow the file ubuntu18.sh

#### setup gazebo
```
	mkdir -p ~/Documents/catkin/src && cd ~/Documents/catkin/src
	git clone https://github.com/jingGM/coslam.git
	cd coslam
	export GAZEBO_MODEL_PATH=$PWD/gazebo/models:$GAZEBO_MODEL_PATH

	cd ../../
	catkin_make
	source ./devel/setup.bash
```
If you don't want to source the bash file each in each new terminal:
```
	echo "source $PWD/devel/setup.bash" >> ~/.bashrc
```

#### setup elastic fusion
Install cuda
```
	cd src
	wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.2.89-1_amd64.deb
	sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
	sudo dpkg -i cuda-repo-ubuntu1804_10.2.89-1_amd64.deb
	rm cuda-repo-ubuntu1804_10.2.89-1_amd64.deb
	sudo add-apt-repository ppa:openjdk-r/ppa 
	sudo apt-get update
	sudo apt-get install cuda-10-2
```

Install other dependencies
```
sudo apt-get install -y cmake-qt-gui git build-essential libusb-1.0-0-dev libudev-dev openjdk-11-* freeglut3-dev libglew-dev libsuitesparse-dev libeigen3-dev zlib1g-dev libjpeg-dev ros-melodic-jackal*
```
	
Install pangolin
```
	cd Pangolin
	mkdir build
	cd build
	cmake ../ -DAVFORMAT_INCLUDE_DIR="" -DCPP11_NO_BOOST=ON
	make -j8
	cd ../..
```
	
Install OPENNI2
```
	mkdir -p repos; cd repos  # create $HOME/repos if it doesn't exist; then, enter it
	git clone https://github.com/occipital/OpenNI2.git  # We used to have a fork off 6857677beee08e264fc5aeecb1adf647a7d616ab with working copy of Xtion Pro Live OpenNI2 driver.
	cd OpenNI2
	make -j$(nproc)  # compile
	sudo ln -s $PWD/Bin/x64-Release/libOpenNI2.so /usr/local/lib/  # $PWD should be /yourPathTo/OpenNI2
	sudo ln -s $PWD/Bin/x64-Release/OpenNI2/ /usr/local/lib/  # $PWD should be /yourPathTo/OpenNI2
	sudo ln -s $PWD/Include /usr/local/include/OpenNI2  # $PWD should be /yourPathTo/OpenNI2
	sudo ldconfig
	cd ..
```

Install code
```
	cd ../Core
	mkdir build
	cd build
	cmake ../src
	make -j8
```



## Run
Run existing file
```
ros_efusion -l DIR_TO_COSLAM/coslam/dataset/dyson_lab.klg
```
Run simulation
```
ros_efusion -lm simulation
```

#### run gazebo
```
	roslaunch coslam world_jackal.launch
	
	conda deactivate
	rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/jackal/jackal_velocity_controller/cmd_vel
```
```
  roslaunch coslam world_robot.launch

  conda deactivate
	rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```


#### run realsense
bring up the realsense camera
```
roslaunch realsense2_camera rs_aligned_depth.launch
```



#####install realsense:
Install realsense SDK from https://github.com/IntelRealSense/librealsense/releases

Install realsense packages:https://github.com/IntelRealSense/librealsense/tree/master/doc
- For UBUNTU
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```
- Check installation
```
realsense-viewer
```
```modinfo uvcvideo | grep "version:"``` should include realsense string

- Update packages
```
sudo apt-get update
sudo apt-get upgrade
```

Install REALSENSE ROS
```
mkdir -p ~/Documents/catkinws/realsense/src
cd ~/Documents/catkinws/realsense/src
git clone https://github.com/IntelRealSense/realsense-ros
cd ..
catkin_make
echo "source ~/Documents/catkinws/realsense/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```