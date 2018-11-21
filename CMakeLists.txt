#おまじない　最低限必要なCmakeのバージョン指定
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#project name ${PROJECT_NAME}で参照可能
project(prism_FootPrint)

#出力情報を増やして cmake デバッグしやすくします。
set(CMAKE_VERBOSE_MAKEFILE 1)

#コンパイルで使用するc++のバージョン指定
set(CMAKE_CXX_STANDARD 11)

#.cmakeファイル = module へのパス
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# release か debugかを指定
set(CMAKE_BUILD_TYPE Debug)

# ちゃんとビルドして生成されたものは
# /build/ディレクトリにしまいましょうという指示
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR
            "CMake generation is not possible within the source directory!"
            "\n Remove the CMakeCache.txt file and try again from another folder, e.g.:"
            "\n "
            "\n rm CMakeCache.txt"
            "\n mkdir build"
            "\n cd build"
            "\n cmake .."
            )
endif()

find_package(GFlags)
find_package(Glog)
find_package(OpenPose REQUIRED)
include(OpenCV)
#include(OpenMVG)
include_directories(${OpenPose_INCLUDE_DIRS} ${GFLAGS_INCLUDE_DIR} ${GLOG_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} /usr/local/include/eigen3)

add_subdirectory(src/basicFunction)
add_subdirectory(src/camera)
add_subdirectory(src/image)
add_subdirectory(src/openPose)
add_subdirectory(src/pointCloud)
add_subdirectory(src/videoToImage)
add_subdirectory(src/reconstruction)
set(SOURCE_FILES src/main.cpp src/FootPrint.cpp src/FootPrint.h)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} basicFunc)
target_link_libraries(${PROJECT_NAME} Reconstruction)
target_link_libraries(${PROJECT_NAME} Camera)
target_link_libraries(${PROJECT_NAME} ImageInfo)
target_link_libraries(${PROJECT_NAME} Openpose)
target_link_libraries(${PROJECT_NAME} VideoToImage)
#target_link_libraries(${PROJECT_NAME} OpenMVG)
target_link_libraries(${PROJECT_NAME} PointCloud)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${OpenMVG_LIBS})
#message(STATUS "OpenMVG_DIR: ${OpenMVG_LIBS}")

target_link_libraries(${PROJECT_NAME} ${OpenPose_LIBS} ${GFLAGS_LIBRARY} ${GLOG_LIBRARY})

