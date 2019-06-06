cmake_minimum_required(VERSION 3.2)
project(exoskeleton)

add_subdirectory(exoskeleton)
set(SOURCE_FILES main.cpp)

add_executable(exo ${SOURCE_FILES})
target_link_libraries(exo exoskeleton)
install(TARGETS exo DESTINATION ${EXOSKELETON_INSTALL_BIN_DIR})
