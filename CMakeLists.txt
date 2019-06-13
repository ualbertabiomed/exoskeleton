cmake_minimum_required (VERSION 3.8)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
set(EXOSKELETON_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/exoskeleton)
set(EXOSKELETON_INSTALL_BIN_DIR ${PROJECT_SOURCE_DIR}/bin)
set(EXOSKELETON_INSTALL_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)

include_directories(${EXOSKELETON_INSTALL_INCLUDE_DIR})

add_subdirectory(src)
# comment

add_executable(exo src/main.cpp)
target_link_libraries(exo exoskeleton)
install(TARGETS exo DESTINATION ${EXOSKELETON_INSTALL_BIN_DIR})

# TODO: Add tests and install targets if needed.
