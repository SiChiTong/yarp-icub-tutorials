cmake_minimum_required(VERSION 2.8.12)

project(assignment_computed-torque)

find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros (e.g. "yarp_install")
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(SYSTEM ${yarpWholeBodyInterface_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp Module.h Module.cpp)
target_link_libraries(${PROJECT_NAME} ${yarpWholeBodyInterface_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()
