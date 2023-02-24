cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(pteranodon_ext VERSION 0.1.0)

# find OpenCV
find_package(OpenCV REQUIRED)

# Include dir
include_directories(/usr/local/include)

# Src
AUX_SOURCE_DIRECTORY(src SRC_FILES)

# Headers
set(PROJECT_SOURCE_DIR "src")
set(PROJECT_INCLUDE_DIR "include/detectors")

# Source files
set(SOURCE_FILES
    ${PROJECT_INCLUDE_DIR}/blob_detector.hpp
    ${PROJECT_SOURCE_DIR}/blob_detector.cpp
)

# Set up such that XCode organizes the files correctly
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES})

# Add library
add_library(pteranodon_ext SHARED ${SOURCE_FILES})

# Include directories
target_include_directories(pteranodon_ext PRIVATE include/detectors)

# Install
install(TARGETS pteranodon_ext DESTINATION lib)

# Install the headers
install(FILES include/pteranodon_ext DESTINATION include)

# Create base directory
install(DIRECTORY include/detectors DESTINATION include)

# Link libraries
target_link_libraries(pteranodon_ext ${OpenCV_LIBS})
