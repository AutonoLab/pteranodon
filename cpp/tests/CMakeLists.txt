cmake_minimum_required(VERSION 3.0.0)
project(test_pteranodon_ext VERSION 0.1.0)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

include_directories(/usr/local/include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_library(PTERANODON_EXT_LIB pteranodon_ext HINTS /usr/local/lib/)

add_executable(test_pteranodon_ext ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

target_link_libraries(test_pteranodon_ext PUBLIC ${PTERANODON_EXT_LIB} ${OpenCV_LIBS})
