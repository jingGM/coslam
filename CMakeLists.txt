cmake_minimum_required(VERSION 3.19)
project(coslam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_CXX_FLAGS ${ADDITIONAL_CMAKE_CXX_FLAGS} "-O3 -msse2 -msse3 -Wall -std=c++11 -DSHADER_DIR=${CMAKE_CURRENT_SOURCE_DIR}/src/GLDisplay/Shaders")

set(CUDA_ARCH_BIN "30 35 50 52 61" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")


set(INTERFACE
        src/Tools/rosinterface/dataInterface.cpp
        src/Tools/rosinterface/dataInterface.h
        src/Tools/rosinterface/rosInterface.cpp
        src/Tools/rosinterface/rosInterface.h)

set(ROSELASTIC  src/rosElastic.cpp src/rosElastic.h)

set(tools
        src/Tools/LiveLogReader.h src/Tools/LiveLogReader.cpp
        src/Tools/LogReader.h src/Tools/JPEGLoader.h
        src/Tools/RawLogReader.h src/Tools/RawLogReader.cpp
        src/Tools/ROSLogReader.h src/Tools/ROSLogReader.cpp
        src/Tools/RealSenseInterface.h src/Tools/RealSenseInterface.cpp
        src/Tools/OpenNI2Interface.h src/Tools/OpenNI2Interface.cpp
        src/Tools/OdometryProvider.h src/Tools/ThreadMutexObject.h
        src/Tools/CameraInterface.h
        src/Tools/GroundTruthOdometry.h src/Tools/GroundTruthOdometry.cpp)
set(utils
        src/Utils/GlobalCamInfo.h src/Utils/GlobalCamInfo.cpp
        src/Utils/LocalCameraInfo.h src/Utils/LocalCameraInfo.cpp
        src/Utils/Img.h src/Utils/Stopwatch.h
        src/Utils/Parse.h src/Utils/Parse.cpp
        src/Utils/DeformationGraph.h src/Utils/DeformationGraph.cpp
        src/Utils/GraphNode.h
        src/Utils/Jacobian.h src/Utils/OrderedJacobianRow.h
        src/Utils/CholeskyDecomp.h src/Utils/CholeskyDecomp.cpp)
set(gldisplay
        src/GLDisplay/Shaders/Shaders.h src/GLDisplay/Shaders/Uniform.h src/GLDisplay/Shaders/Vertex.h
        src/GLDisplay/GUI.h src/GLDisplay/GUI.cpp
        src/GLDisplay/Resize.h src/GLDisplay/Resize.cpp
        src/GLDisplay/ComputePack.h src/GLDisplay/ComputePack.cpp
        src/GLDisplay/FeedbackBuffer.h src/GLDisplay/FeedbackBuffer.cpp
        src/GLDisplay/GlobalModel.h src/GLDisplay/GlobalModel.cpp
        src/GLDisplay/IndexMap.h src/GLDisplay/IndexMap.cpp
        src/GLDisplay/FillIn.h src/GLDisplay/FillIn.cpp
        src/GLDisplay/GlobalCamTexture.h)
set(cudas
        src/Core/cuda/convenience.cuh
        src/Core/cuda/operators.cuh
        src/Core/cuda/types.cuh
        src/Core/cuda/reduce.cu
        src/Core/cuda/cudafuncs.cuh src/Core/cuda/cudafuncs.cu)
set(cudacpp
        src/Core/cuda/GPUConfig.h
        src/Core/cuda/containers/device_array.hpp
        src/Core/cuda/containers/kernel_containers.hpp
        src/Core/cuda/containers/device_memory_impl.hpp
        src/Core/cuda/containers/device_array_impl.hpp
        src/Core/cuda/containers/device_memory.hpp src/Core/cuda/containers/device_memory.cpp)
set(cores
        ${cudacpp}
        src/Core/GPUTexture.h src/Core/GPUTexture.cpp
        src/Core/ElasticFusion.h src/Core/ElasticFusion.cpp
        src/Core/RGBDOdometry.h src/Core/RGBDOdometry.cpp
        src/Core/GlobalOdometry.h src/Core/GlobalOdometry.cpp
        src/Core/Ferns.h src/Core/Ferns.cpp
        src/Core/Deformation.h src/Core/Deformation.cpp
        src/Core/PoseMatch.h)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        roscpp
        rospy
        sensor_msgs
        std_msgs
        cv_bridge
        tf
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

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES coslam
        #  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
        #  DEPENDS system_lib
)

add_definitions(-Dlinux=1)

include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OPENNI2_INCLUDE_DIR})
include_directories(${SUITESPARSE_INCLUDE_DIRS})
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

######################################
## set up cuda
######################################
include(CudaComputeTargetFlags.cmake)
APPEND_TARGET_ARCH_FLAGS()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")

CUDA_COMPILE(cuda_objs ${cudas})

if(WIN32)
    set(ADDITIONAL_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
endif()
###########################

add_executable(ros_efusion main.cpp
        ${INTERFACE}
        ${ROSELASTIC}
        ${tools}
        ${utils}
        ${gldisplay}
        ${cores}
        ${cuda_objs}
        )
target_link_libraries(ros_efusion
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${EXTRA_WINDOWS_LIBS}
        ${ZLIB_LIBRARY}
        ${Pangolin_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${EXTRA_LIBS}
        ${OPENNI2_LIBRARY}
        ${SUITESPARSE_LIBRARIES}
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${OpenCV_LIBS}
        )

#set(rosutils
#        src/Utils/GlobalCamInfo.h src/Utils/GlobalCamInfo.cpp
#        src/Utils/LocalCameraInfo.h src/Utils/LocalCameraInfo.cpp)
#add_executable(collect_data src/Tools/rosinterface/collectdata.cpp ${INTERFACE} ${rosutils})
#target_link_libraries(collect_data ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(test_rosinterface src/test/rosinterface.cpp ${INTERFACE} ${tools} ${utils})
#target_link_libraries(test_rosinterface ${catkin_LIBRARIES}
#        ${Boost_LIBRARIES}
#        ${EXTRA_WINDOWS_LIBS}
#        ${ZLIB_LIBRARY}
#        ${Pangolin_LIBRARIES}
#        ${CUDA_LIBRARIES}
#        ${EXTRA_LIBS}
#        ${OPENNI2_LIBRARY}
#        ${SUITESPARSE_LIBRARIES}
#        ${BLAS_LIBRARIES}
#        ${LAPACK_LIBRARIES}
#        ${OpenCV_LIBS}
#        )

#add_executable(smalltest src/test/small.cpp)

#add_executable(realsenseData src/test/realsense/saveData.cpp
#        src/test/realsense/DataCollection.h
#        src/test/realsense/DataCollection.cpp)
#target_link_libraries(realsenseData
#        ${catkin_LIBRARIES}
#        ${Boost_LIBRARIES}
#        ${EXTRA_WINDOWS_LIBS}
#        ${ZLIB_LIBRARY}
#        ${Pangolin_LIBRARIES}
#        ${CUDA_LIBRARIES}
#        ${EXTRA_LIBS}
#        ${OPENNI2_LIBRARY}
#        ${SUITESPARSE_LIBRARIES}
#        ${BLAS_LIBRARIES}
#        ${LAPACK_LIBRARIES}
#        ${OpenCV_LIBS}
#        )
