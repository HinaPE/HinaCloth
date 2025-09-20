cmake_minimum_required(VERSION 3.26)
project(HinaCloth LANGUAGES CXX)

# Global configuration
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ----------------------------------------------------------------------------
# Core library: libhinacloth (XPBD implementation)
# ----------------------------------------------------------------------------
file(GLOB_RECURSE HINACLOTH_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")
add_library(libhinacloth STATIC ${HINACLOTH_SRC})

# Public include for API consumption
target_include_directories(libhinacloth PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Enforce C++23 feature availability
target_compile_features(libhinacloth PUBLIC cxx_std_23)

# Warnings and compiler options
include(CheckCXXCompilerFlag)
if (MSVC)
    target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:/O2 /Ot /Oi /fp:fast>)
    target_compile_options(libhinacloth PRIVATE /W4 /permissive- /utf-8)
    # Report proper __cplusplus value to the preprocessor
    target_compile_options(libhinacloth PRIVATE /Zc:__cplusplus)
    target_compile_definitions(libhinacloth PUBLIC _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:-O3 -ffast-math -fomit-frame-pointer>)
    target_compile_options(libhinacloth PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------------------------------
# Examples (no external visualizer dependency)
# ----------------------------------------------------------------------------
set(EXAMPLES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples)

function(_hina_setup_example target)
    if (TARGET ${target})
        target_link_libraries(${target} PRIVATE libhinacloth)
        target_compile_features(${target} PRIVATE cxx_std_23)
        if (MSVC)
            target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus)
            target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS)
        else()
            target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
        endif()
    endif()
endfunction()

if (EXISTS ${EXAMPLES_DIR}/e_min.cpp)
    add_executable(example_xpbd_min ${EXAMPLES_DIR}/e_min.cpp)
    _hina_setup_example(example_xpbd_min)
endif()

if (EXISTS ${EXAMPLES_DIR}/e_max.cpp)
    add_executable(example_xpbd_max ${EXAMPLES_DIR}/e_max.cpp)
    _hina_setup_example(example_xpbd_max)
endif()

if (EXISTS ${EXAMPLES_DIR}/e_smoke.cpp)
    add_executable(example_xpbd_smoke ${EXAMPLES_DIR}/e_smoke.cpp)
    _hina_setup_example(example_xpbd_smoke)
endif()
