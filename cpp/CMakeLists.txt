cmake_minimum_required(VERSION 3.4)
project(pteranodon_ext VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

add_compile_options(-O3 -Wall)

include_directories(/usr/local/include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/headers)

file(GLOB_RECURSE headers CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/headers/*.hpp)
file(GLOB_RECURSE sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${sources} ${headers})

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES include/pteranodon_ext DESTINATION include)
install(DIRECTORY include/headers DESTINATION include)
