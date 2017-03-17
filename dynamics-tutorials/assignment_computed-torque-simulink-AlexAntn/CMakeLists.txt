cmake_minimum_required(VERSION 2.8.12)

project(assignment_computed-torque-simulink)


set(SIMMODEL "${CMAKE_CURRENT_SOURCE_DIR}/computed_torque.mdl")

find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()

install(FILES ${SIMMODEL} DESTINATION simmodels)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()
