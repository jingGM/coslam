cmake_minimum_required(VERSION 3.19)
project(test)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        Thirdparty/g2o
)

set(source
        src/System.h src/System.cpp
        src/Tracking.h src/Tracking.cpp
        src/ORBextractor.h src/ORBextractor.cpp
        src/Frame.h src/Frame.cpp
        src/Initializer.h src/Initializer.cpp
        src/ORBmatcher.h src/ORBmatcher.cpp
        src/ORBVocabulary.h
        )
set(CMAKE_CXX_STANDARD 14)

add_executable(test main.cpp ${source})
target_link_libraries(test
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        )