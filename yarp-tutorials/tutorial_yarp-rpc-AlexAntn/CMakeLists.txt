# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(tutorial_yarp-rpc)

# mandatory use of these packages
find_package(YARP REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

# helpers defining certain macros (e.g. "yarp_install")
#include(YarpInstallationHelpers)

# building client 
include_directories(${YARP_INCLUDE_DIRS})
add_executable(tutorial_yarp-rpc-client ${CMAKE_SOURCE_DIR}/src/rpc-client.cpp)
target_link_libraries(tutorial_yarp-rpc-client ${YARP_LIBRARIES})
install(TARGETS tutorial_yarp-rpc-client
	RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# building server
include_directories(${YARP_INCLUDE_DIRS})
add_executable(tutorial_yarp-rpc-server ${CMAKE_SOURCE_DIR/}src/rpc-server.cpp)
target_link_libraries(tutorial_yarp-rpc-server ${YARP_LIBRARIES})
install(TARGETS tutorial_yarp-rpc-server
	RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)

