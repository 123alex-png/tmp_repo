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

# Get all .cpp files in the /src folder

file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")

# Create the executable
add_executable(simpleAttach ${SOURCES})

# Link Boost libraries
target_link_libraries(simpleAttach Boost::program_options Threads::Threads)

# Run clang-format on all .cpp and .hpp files in the project
file(GLOB_RECURSE ALL_SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/include/*.hpp
    ${SOURCES}
    ${CMAKE_SOURCE_DIR}/src/*.hpp
) 

file(GLOB_RECURSE LIB_SOURCES
    ${CMAKE_SOURCE_DIR}/library/*.c
)

add_library(simpleAttachLib OBJECT ${LIB_SOURCES})

set_target_properties(simpleAttachLib PROPERTIES OUTPUT_NAME simpleAttachLib.o)

add_custom_target(
    clang-format
    COMMAND clang-format
    -style=file
    -i
    ${ALL_SOURCE_FILES}
)

add_custom_target(
    clang-format-lib
    COMMAND clang-format
    -style=file
    -i
    ${LIB_SOURCES}
)

add_dependencies(simpleAttach clang-format)
add_dependencies(simpleAttachLib clang-format-lib)
target_link_options(simpleAttachLib PRIVATE -static)

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
