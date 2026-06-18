cmake_minimum_required(VERSION 3.20)
project(shelf_solver CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to a fully optimized build when no type is requested.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(nlohmann_json 3.2.0 REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(shelf_solver main.cpp shelf_solver.cpp)
target_link_libraries(shelf_solver PRIVATE nlohmann_json::nlohmann_json Threads::Threads)

# --- Map data format -------------------------------------------------------
# Newer Scene/<n>/buildConstData.json files describe floor (type 101) and high
# platform (type 201) tiles as rectangular REGIONS: Params = [x, y, w, h].
# Legacy maps used a single cell ([x, y]) per command. Turn this OFF to parse
# the legacy single-cell format.
option(NEW_MAP_FORMAT "Interpret type-101/201 Params as [x,y,w,h] regions" ON)
if(NEW_MAP_FORMAT)
    target_compile_definitions(shelf_solver PRIVATE SHELF_RECT_REGIONS)
    message(STATUS "Map format: rectangular regions ([x,y,w,h])")
else()
    message(STATUS "Map format: legacy single-cell ([x,y])")
endif()

# --- Highest level of optimization -----------------------------------------
# -O3        aggressive optimization
# -march/-mtune=native  tune for the build machine's CPU
# -funroll-loops        unroll hot SA/DFS loops
# -fomit-frame-pointer  free a register on the hot path
# NDEBUG     drop assertions
target_compile_options(shelf_solver PRIVATE
    $<$<CONFIG:Release>:-O3 -march=native -mtune=native -funroll-loops -fomit-frame-pointer -DNDEBUG>)

# Link-time optimization (whole-program) when the toolchain supports it.
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported)
    set_property(TARGET shelf_solver PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    message(STATUS "IPO/LTO enabled for Release")
else()
    message(STATUS "IPO/LTO not supported: ${ipo_error}")
endif()