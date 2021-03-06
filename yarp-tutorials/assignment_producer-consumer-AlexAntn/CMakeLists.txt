# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(tutorial_cartesian-interface)

# mandatory use of these packages
find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros (e.g. "yarp_install")
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# import math symbols from standard cmath
#add_definitions(-D_USE_MATH_DEFINES)




# building client 
include_directories(${YARP_INCLUDE_DIRS})
add_executable(producer ${CMAKE_SOURCE_DIR}/src/producer.cpp)
target_link_libraries(producer ${YARP_LIBRARIES})
install(TARGETS producer DESTINATION bin)

# building server
include_directories(${YARP_INCLUDE_DIRS})
add_executable(consumer ${CMAKE_SOURCE_DIR/}src/consumer.cpp)
target_link_libraries(consumer ${YARP_LIBRARIES})
install(TARGETS consumer DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

