# CMakeList.txt : CMake project for exoskeleton, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)
project(exoskeleton CXX)

set(SOURCE_FILES
    exoskeleton/exoskeleton.cpp
)

add_library(exoskeleton SHARED STATIC ${SOURCE_FILES})
install(TARGETS exoskeleton DESTINATION ${EXOSKELETON_INSTALL_LIB_DIR})
