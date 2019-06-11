cmake_minimum_required(VERSION 3.2)
project(exoskeleton C CXX)

set(SOURCE_FILES
    exoskeleton.h
    exoskeleton.cpp
)

add_library(exoskeleton SHARED STATIC ${SOURCE_FILES})

install(TARGETS exoskeleton DESTINATION ${EXOSKELETON_INSTALL_LIB_DIR})
install(FILES exoskeleton.h DESTINATION ${EXOSKELETON_INSTALL_INCLUDE_DIR})
