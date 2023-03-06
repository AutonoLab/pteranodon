cmake_minimum_required(VERSION 3.4)
project(pteranodon_ext VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
enable_testing()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-O3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/headers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/python)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/extern/pybind11)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/extern/cvnp)

set(NUMPY_INCLUDE_DIR "" CACHE FILEPATH "Path to numpy header if cmake can't find them.")
if (NOT ${NUMPY_INCLUDE_DIR} STREQUAL "")
  message( " *** NUMPY_INCLUDE_DIR : ${NUMPY_INCLUDE_DIR}" )
  if(NOT EXISTS ${NUMPY_INCLUDE_DIR}/numpy/ndarrayobject.h)
    message(SEND_ERROR "Can't find numpy/ndarrayobject.h in ${NUMPY_INCLUDE_DIR}")
    endif()
  include_directories(${NUMPY_INCLUDE_DIR})
endif()

find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${pybind11_INCLUDE_DIR})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS}/opencv4)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file (GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/*.cpp)
file (GLOB_RECURSE HEADER_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/headers/*.hpp)
file (GLOB_RECURSE PYTHON_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/python/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/python/*.hpp)

LINK_DIRECTORIES(
  ${OpenCV_LIB_DIR}
)

pybind11_add_module(pteranodon_ext MODULE
	${SOURCE_FILES}
	${HEADER_FILES}
	${PYTHON_FILES}
)

target_link_libraries(pteranodon_ext PRIVATE cvnp pybind11::pybind11 ${OpenCV_LIBS})

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
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")

add_test(NAME pteranodon_ext_test
  COMMAND ${PYTHON_EXECUTABLE} -c "import pteranodon_ext"
)