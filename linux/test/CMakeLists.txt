cmake_minimum_required(VERSION 3.2)
project(exoskeleton_tests)

add_subdirectory(lib/googletest)

include_directories(${EXOSKELETON_HEADERS_DIR})
include_directories(lib/googletest/googletest/include)

set(SOURCE_FILES main.cpp src/exoskeleton_tests.cpp)

add_executable(exoskeleton_tests ${SOURCE_FILES})
target_link_libraries(exoskeleton_tests exoskeleton gtest)
install(TARGETS exoskeleton_tests DESTINATION bin)

