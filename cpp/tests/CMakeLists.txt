cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(test_detectors)

include_directories(/usr/local/include)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../bin)

find_library(PTERANODON_EXT_LIB pteranodon_ext HINTS /usr/local/lib/)

add_executable(test_detectors src/test_detectors.cpp)

target_link_libraries(test_detectors PUBLIC ${PTERANODON_EXT_LIB})
