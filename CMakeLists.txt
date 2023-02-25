cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

project(pteranodon_ext)

find_package(PythonLibs)
execute_process(
    COMMAND "python3" "-c" "import os, numpy.distutils; print(os.pathsep.join(numpy.distutils.misc_util.get_numpy_include_dirs()))"
    RESULT_VARIABLE _numpy_process
    OUTPUT_VARIABLE _numpy_include_dirs
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if (NOT _numpy_process EQUAL 0)
    message(FATAL_ERROR "Can't locate numpy include dirs")
endif()
include_directories(${PYTHON_INCLUDE_DIRS} ${_numpy_include_dirs})
message("Python libs: ${PYTHON_LIBRARIES}")

include_directories("${CMAKE_SOURCE_DIR}/cpp/include/detectors")
include_directories("${CMAKE_SOURCE_DIR}/python")

file (GLOB SOURCE_FILES "cpp/src/*.cpp")
file (GLOB HEADER_FILES "cpp/include/detectors/*.hpp")
file (GLOB PYTHON_FILES "python/*.cpp" "python/*.hpp")

find_package(pybind11 REQUIRED)

include(pybind11.cmake)
pybind11_add_module(pteranodon_ext 
	${SOURCE_FILES}
	${HEADER_FILES}
	${PYTHON_FILES}
)

target_link_libraries(pteranodon_ext PUBLIC)

install(TARGETS pteranodon_ext
  COMPONENT python
  LIBRARY DESTINATION "${PYTHON_LIBRARY_DIR}"
  )
