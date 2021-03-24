#ROS:
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update


# If ERROR:  
# ERROR: cannot download default sources list from:
#  https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
# Website may be down
```
sudo echo "199.232.28.133 raw.githubusercontent.com" >> /etc/hosts
sudo echo "151.101.228.133 raw.github.com" >> /etc/hosts
```


#Install ROS packages
```
sudo apt-get install ros-melodic-jackal-*
sudo apt-get install ros-melodic-turtlebot3*
```
Find a directory to build opencv package for ROS
```
sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-melodic-cv-bridge
mkdir -p $PWD/catkin_CVBridge/src
cd catkin_CVBridge/
catkin init
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3.6 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install
git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv

apt-cache show ros-melodic-cv-bridge | grep Version
cd src/vision_opencv/
# according to the shown version
git checkout 1.13.0
cd ../../
catkin config --install
catkin build cv_bridge
echo "source $PWD/install/setup.bash --extend" >> ~/.bashrc
```