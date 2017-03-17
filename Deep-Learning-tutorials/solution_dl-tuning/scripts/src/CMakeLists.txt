# Copyright: (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Pasquale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.8.8)

set(PROJECTNAME classify_image_list_vvv17)
project(${PROJECTNAME})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

option(CPU_ONLY "Use CPU only for Caffe" ON)
option(USE_CUDNN "Use CuDNN library for Caffe" OFF)

find_package(OpenCV REQUIRED)
find_package(Caffe REQUIRED)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost COMPONENTS filesystem)

include_directories(${Boost_INCLUDE_DIRS}) 
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Caffe_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR})

add_definitions(${Caffe_DEFINITIONS})    # ex. -DCPU_ONLY

SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g" )

set(folder_source ${PROJECTNAME}.cpp)
source_group("Source Files" FILES ${folder_source})

add_executable(${PROJECTNAME} ${folder_source})

target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECTNAME} ${Caffe_LIBRARIES})
target_link_libraries(${PROJECTNAME} ${Boost_LIBRARIES})
