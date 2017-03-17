# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(tutorial_yarp-port)

# mandatory use of these packages
find_package(YARP REQUIRED)

# extend the current search path used by cmake to load helpers
# list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
# helpers defining certain macros (e.g. "yarp_install")
# include(YarpInstallationHelpers)

include_directories(${YARP_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/main.cpp)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

#install(TARGETS ${PROJECT_NAME} DESTINATION bin)
#file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
#yarp_install(FILES ${scripts} DESTINATION bin)
