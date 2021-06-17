cmake_minimum_required(VERSION 3.0.2)
project(coslam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
#set(CMAKE_CXX_FLAGS ${ADDITIONAL_CMAKE_CXX_FLAGS} "-O3 -msse2 -msse3 -Wall -std=c++11")

set(INTERFACE
        src/rosinterface/dataInterface.cpp
        src/rosinterface/dataInterface.h
        src/rosinterface/rosInterface.cpp
        src/rosinterface/rosInterface.h
        )

set(ROSELASTIC  src/rosElastic.cpp src/rosElastic.h)

file(GLOB tools src/Tools/*.cpp src/Tools/*h)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        roscpp
        rospy
        sensor_msgs
        std_msgs
        cv_bridge
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenNI2 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(efusion REQUIRED)

include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EFUSION_INCLUDE_DIR})
include_directories(${OPENNI2_INCLUDE_DIR})

add_definitions(-Dlinux=1)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coslam
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(ros_efusion src/rosEfusion.cpp
                        ${INTERFACE}
                        ${ROSELASTIC}
                        ${tools})
target_link_libraries(ros_efusion
                        ${catkin_LIBRARIES}
                        ${Boost_LIBRARIES}
                        ${EXTRA_WINDOWS_LIBS}
                        ${ZLIB_LIBRARY}
                        ${Pangolin_LIBRARIES}
                        ${CUDA_LIBRARIES}
                        ${EXTRA_LIBS}
                        ${EFUSION_LIBRARY}
                        ${OPENNI2_LIBRARY}
                        ${SUITESPARSE_LIBRARIES}
                        ${BLAS_LIBRARIES}
                        ${LAPACK_LIBRARIES}
                        )

#add_executable(test_logreader src/test/logreader.cpp ${tools} ${INTERFACE})
#target_link_libraries(test_logreader
#                        ${Pangolin_LIBRARIES}
#                        ${EXTRA_LIBS}
#                        ${EFUSION_LIBRARY}
#                        ${CUDA_LIBRARIES}
#                        ${OPENNI2_LIBRARY}
#                        ${SUITESPARSE_LIBRARIES}
#                        ${LAPACK_LIBRARIES}
#                        ${catkin_LIBRARIES}
#                        ${Boost_LIBRARIES}
#        )

add_executable(test_rosinterface src/test/rosinterface.cpp ${INTERFACE})
target_link_libraries(test_rosinterface ${catkin_LIBRARIES} ${Boost_LIBRARIES})