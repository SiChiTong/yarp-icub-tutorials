# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Francesco Romano <francesco.romano@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.12)

# set the project name
project(TestAssignmentComputedTorque)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

# add the required cmake packages
find_package(RTF COMPONENTS DLL)
find_package(YARP REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)
find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

# add include directories
include_directories(${CMAKE_SOURCE_DIR})

include_directories(SYSTEM
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${YARP_HELPERS_INCLUDE_DIR}
                    ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

# add required libraries
link_libraries(${RTF_LIBRARIES} ${YARP_LIBRARIES} ${yarpWholeBodyInterface_LIBRARIES})

# add the source codes to build the plugin library
add_library(${PROJECT_NAME} MODULE test.cpp)
