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

file (GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/*.cpp)
file (GLOB_RECURSE HEADER_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/headers/*.hpp)
file (GLOB_RECURSE PYTHON_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/python/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/python/*.hpp)

pybind11_add_module(pteranodon_ext 
	${SOURCE_FILES}
	${HEADER_FILES}
	${PYTHON_FILES}
)

target_link_libraries(pteranodon_ext PUBLIC)

find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)

exec_program(${PYTHON_EXECUTABLE}
             ARGS "-c \"import numpy; print(numpy.get_include())\""
             OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
             RETURN_VALUE NUMPY_NOT_FOUND
            )
if(NUMPY_NOT_FOUND)
    message(FATAL_ERROR "NumPy headers not found")
endif()

install(TARGETS pteranodon_ext
  COMPONENT python
  LIBRARY DESTINATION "${PYTHON_LIBRARY_DIR}"
)
