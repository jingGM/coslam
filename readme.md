## set up instructions:

#### setup ros environment:
follow the file ubuntu18.sh

#### setup gazebo
```
	mkdir -p ~/Documents/catkin/src && cd ~/Documents/catkin/src
	git clone xxxxxxxxxxxxxxxxxxxxxxxxx
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
sudo apt-get install -y cmake-qt-gui git build-essential libusb-1.0-0-dev libudev-dev openjdk-11-* freeglut3-dev libglew-dev libsuitesparse-dev libeigen3-dev zlib1g-dev libjpeg-dev
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
	roslaunch coslam world.launch

	conda deactivate
	rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/jackal/jackal_velocity_controller/cmd_vel
```
