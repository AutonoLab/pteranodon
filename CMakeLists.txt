cmake_minimum_required(VERSION 3.4)
project(pteranodon_ext VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/headers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/python)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/extern/pybind11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file (GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/*.cpp)
file (GLOB_RECURSE HEADER_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/headers/*.hpp)
file (GLOB_RECURSE PYTHON_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/python/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/python/*.hpp)

pybind11_add_module(pteranodon_ext 
	${SOURCE_FILES}
	${HEADER_FILES}
	${PYTHON_FILES}
)

target_link_libraries(pteranodon_ext PUBLIC ${OPENCV_LIBS} ${PTERANODON_EXT})

find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)

exec_program(${PYTHON_EXECUTABLE}
             ARGS "-c \"import sysconfig; print(sysconfig.get_paths()['purelib'])\""
             OUTPUT_VARIABLE PYTHON_LIBRARY_DIR
             RETURN_VALUE PYTHON_LIBRARY_DIR_NOT_FOUND
            )
if(PYTHON_LIBRARY_DIR_NOT_FOUND)
    message(FATAL_ERROR "Python library directory not found")
endif()

install(TARGETS pteranodon_ext
  COMPONENT python
  LIBRARY DESTINATION "${PYTHON_LIBRARY_DIR}"
)
