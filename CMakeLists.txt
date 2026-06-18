cmake_minimum_required(VERSION 3.20)

option(SHELF_STATIC_MSVC_RUNTIME "Link the MSVC runtime statically" ON)
if(SHELF_STATIC_MSVC_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

project(shelf_solver CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to a fully optimized build when no type is requested.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(SHELF_USE_BUNDLED_JSON "Fetch nlohmann_json during configure instead of using a system package" OFF)
option(SHELF_NATIVE_OPTIMIZATIONS "Tune Release builds for the build host CPU" OFF)

function(shelf_fetch_nlohmann_json)
    include(FetchContent)
    set(JSON_BuildTests OFF CACHE INTERNAL "")
    FetchContent_Declare(nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.12.0
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(nlohmann_json)
endfunction()

if(SHELF_USE_BUNDLED_JSON)
    shelf_fetch_nlohmann_json()
else()
    find_package(nlohmann_json 3.2.0 QUIET)
    if(NOT nlohmann_json_FOUND)
        message(STATUS "nlohmann_json package not found; fetching bundled copy")
        shelf_fetch_nlohmann_json()
    endif()
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

file(GLOB SHELF_SCENE_MAPS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Scene/*/buildConstData.json")
set(SHELF_EMBEDDED_MAPS_CPP "${CMAKE_CURRENT_BINARY_DIR}/generated/embedded_maps.cpp")
set(SHELF_EMBEDDED_MAPS_HPP "${CMAKE_CURRENT_BINARY_DIR}/generated/embedded_maps.hpp")
add_custom_command(
    OUTPUT "${SHELF_EMBEDDED_MAPS_CPP}" "${SHELF_EMBEDDED_MAPS_HPP}"
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/scripts/embed_maps.py"
            --scene-dir "${CMAKE_CURRENT_SOURCE_DIR}/Scene"
            --out-cpp "${SHELF_EMBEDDED_MAPS_CPP}"
            --out-hpp "${SHELF_EMBEDDED_MAPS_HPP}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/embed_maps.py" ${SHELF_SCENE_MAPS}
    VERBATIM)

add_executable(shelf_solver main.cpp shelf_solver.cpp "${SHELF_EMBEDDED_MAPS_CPP}")
target_include_directories(shelf_solver PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
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

# --- Release optimization --------------------------------------------------
# -O3        aggressive optimization
# -funroll-loops        unroll hot SA/DFS loops
# -fomit-frame-pointer  free a register on the hot path
# SHELF_NATIVE_OPTIMIZATIONS adds -march/-mtune=native for local builds where
# CPU portability is not required.
target_compile_options(shelf_solver PRIVATE
    $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:GNU,Clang,AppleClang>>:-O3 -funroll-loops -fomit-frame-pointer -DNDEBUG>
    $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:MSVC>>:/O2 /DNDEBUG>)
if(SHELF_NATIVE_OPTIMIZATIONS)
    target_compile_options(shelf_solver PRIVATE
        $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:GNU,Clang,AppleClang>>:-march=native -mtune=native>)
endif()

# Link-time optimization (whole-program) when the toolchain supports it.
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported)
    set_property(TARGET shelf_solver PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    message(STATUS "IPO/LTO enabled for Release")
else()
    message(STATUS "IPO/LTO not supported: ${ipo_error}")
endif()
