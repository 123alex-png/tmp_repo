cmake_minimum_required(VERSION 3.10)
project(utrace)

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

# Get all .cpp files in the /src folder
file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE LIB_SOURCES "${CMAKE_SOURCE_DIR}/library/*.c")

# Create the executable
add_executable(utrace ${SOURCES})
add_library(utrace_lib SHARED ${LIB_SOURCES})

# Set the output name for the shared library
set_target_properties(utrace_lib PROPERTIES OUTPUT_NAME "utrace")

# Link Boost libraries
target_link_libraries(utrace Boost::program_options Threads::Threads)

# Run clang-format on all .cpp and .hpp files in the project
file(GLOB_RECURSE ALL_SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/include/*.hh
    ${SOURCES}
    ${LIB_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/*.hpp
) 

add_custom_target(
    clang-format
    COMMAND clang-format
    -style=file
    -i
    ${ALL_SOURCE_FILES}
)

add_dependencies(utrace clang-format)
add_dependencies(utrace_lib clang-format)

add_custom_target(
    clean-all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND rm -rf ${CMAKE_SOURCE_DIR}/build
)