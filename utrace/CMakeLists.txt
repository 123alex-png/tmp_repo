cmake_minimum_required(VERSION 3.10)
project(simpleAttach)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Enable debug symbols
set(CMAKE_BUILD_TYPE Debug)

# Add include directories
include_directories(include)
include_directories(third-party)

# Find Boost libraries
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Threads REQUIRED)

# Get all .cpp files in the project
set(SRC_FOLDERS 
    ${CMAKE_SOURCE_DIR}/initialize
    ${CMAKE_SOURCE_DIR}/pid-monitor
    ${CMAKE_SOURCE_DIR}/run-monitor
)
file(GLOB_RECURSE SOURCES "main.cpp")
foreach(SRC_FOLDER ${SRC_FOLDERS})
    file(GLOB_RECURSE FILES "${SRC_FOLDER}/*.cpp")
    set(SOURCES ${SOURCES} ${FILES})
endforeach()

# Create the executable
add_executable(simpleAttach ${SOURCES})

# Link Boost libraries
target_link_libraries(simpleAttach Boost::program_options Threads::Threads)

# Run clang-format on all .cpp and .hpp files in the project
file(GLOB_RECURSE ALL_SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/include/*.hpp
    ${SRC_FOLDERS}/*.cpp
    ${SRC_FOLDERS}/*.hpp
) 

add_custom_target(
    clang-format
    COMMAND clang-format
    -style=file
    -i
    ${ALL_SOURCE_FILES}
)
add_dependencies(simpleAttach clang-format)

# clean up
add_custom_target(
    clean-all
    COMMAND rm -rf ${CMAKE_SOURCE_DIR}/build
    COMMAND rm -rf ${CMAKE_SOURCE_DIR}/CMakeFiles
    COMMAND rm -rf ${CMAKE_SOURCE_DIR}/CMakeCache.txt
    COMMAND rm -rf ${CMAKE_SOURCE_DIR}/cmake_install.cmake
    COMMAND rm -rf ${CMAKE_SOURCE_DIR}/simpleAttach
    COMMAND rm -rf ${CMAKE_SOURCE_DIR}/Makefile
)
