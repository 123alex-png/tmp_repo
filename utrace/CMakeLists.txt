cmake_minimum_required(VERSION 3.10)
project(utrace)

# test whether CMake is run in the project root directory
file(TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" PATH_TO_CMAKELISTS_TXT)

message(STATUS ${TO_CMAKE_PATH})
message(STATUS "CMakeLists.txt path: ${PATH_TO_CMAKELISTS_TXT}")

if(EXISTS "${PATH_TO_CMAKELISTS_TXT}")
    message(FATAL_ERROR "Run CMake from a build subdirectory! \"mkdir build ; cd build ; cmake ..\" \
    Some junk files were created in this folder (CMakeCache.txt, CMakeFiles); you should delete those.")
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Enable debug symbols
set(CMAKE_BUILD_TYPE Debug)

# Add include directories
include_directories(src/include)
include_directories(src/third-party)

# Find Boost libraries
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Threads REQUIRED)

# Get all .cpp files in the /src folder
file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE LIB_SOURCES_DBG "${CMAKE_SOURCE_DIR}/library/with-debugger/*.c")
file(GLOB_RECURSE LIB_SOURCES_NO_DBG "${CMAKE_SOURCE_DIR}/library/without-debugger/*.c")
file(GLOB_RECURSE LIB_HEADERS "${CMAKE_SOURCE_DIR}/library/without-debugger/*.h")

# Create the executable
add_executable(utrace ${SOURCES})
add_library(utrace_lib_dbg SHARED ${LIB_SOURCES_DBG})
add_library(utrace_lib_no_dbg STATIC ${LIB_SOURCES_NO_DBG})

# Set the output name for the shared library
set_target_properties(utrace_lib_dbg PROPERTIES OUTPUT_NAME "utrace-dbg")
set_target_properties(utrace_lib_no_dbg PROPERTIES OUTPUT_NAME "utrace-nodbg")

# Link Boost libraries
target_link_libraries(utrace Boost::program_options Threads::Threads)

# Run clang-format on all .cpp and .hpp files in the project
file(GLOB_RECURSE ALL_SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/include/*.hh
    ${SOURCES}
    ${LIB_SOURCES_DBG}
    ${LIB_SOURCES_NO_DBG}
    ${LIB_HEADERS}
) 

## clang-format
find_program(CLANG_FORMAT "clang-format")
if(CLANG_FORMAT)
    add_custom_target(
        clang-format
        COMMAND clang-format
        -style=file
        -i
        ${ALL_SOURCE_FILES}
    )

    add_dependencies(utrace clang-format)
    add_dependencies(utrace_lib_dbg clang-format)
    add_dependencies(utrace_lib_no_dbg clang-format)
else()
    message(WARNING "clang-format not found. Skipping the clang-format target.")
endif()

add_custom_target(
    clean-all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND rm -rf ${CMAKE_SOURCE_DIR}/build
)
