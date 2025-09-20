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
set(HINA_HAVE_AVX2 FALSE)
if (MSVC)
    target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:/O2 /Ot /Oi /fp:fast>)
    target_compile_options(libhinacloth PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(libhinacloth PRIVATE /Zc:__cplusplus)
    target_compile_definitions(libhinacloth PUBLIC _CRT_SECURE_NO_WARNINGS)
    check_cxx_compiler_flag("/arch:AVX2" HINA_MSVC_HAS_AVX2)
    if (HINA_MSVC_HAS_AVX2)
        target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:/arch:AVX2>)
        set(HINA_HAVE_AVX2 TRUE)
    endif()
else()
    target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:-O3 -ffast-math -fomit-frame-pointer>)
    target_compile_options(libhinacloth PRIVATE -Wall -Wextra -Wpedantic)
    check_cxx_compiler_flag("-mavx2" HINA_GCC_HAS_MAVX2)
    check_cxx_compiler_flag("-mfma"  HINA_GCC_HAS_MFMA)
    set(_AVX_FLAGS)
    if (HINA_GCC_HAS_MAVX2)
        list(APPEND _AVX_FLAGS -mavx2)
    endif()
    if (HINA_GCC_HAS_MFMA)
        list(APPEND _AVX_FLAGS -mfma)
    endif()
    if (_AVX_FLAGS)
        target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:${_AVX_FLAGS}>)
        set(HINA_HAVE_AVX2 TRUE)
    endif()
endif()

if (HINA_HAVE_AVX2)
    target_compile_definitions(libhinacloth PUBLIC HINACLOTH_HAVE_AVX2=1)
endif()

# ----------------------------------------------------------------------------
# oneTBB: enabled via helper (Stage 2)
# ----------------------------------------------------------------------------
include(cmake/setup_tbb.cmake)
use_tbb(libhinacloth)
# Enable TBB code paths in runtime
target_compile_definitions(libhinacloth PUBLIC HINACLOTH_HAVE_TBB=1)

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
        else()
            target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
        endif()
        use_tbb(${target})
        if (HINA_HAVE_AVX2)
            target_compile_definitions(${target} PRIVATE HINACLOTH_HAVE_AVX2=1)
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

if (EXISTS ${EXAMPLES_DIR}/e_flag.cpp)
    add_executable(example_xpbd_flag ${EXAMPLES_DIR}/e_flag.cpp)
    _hina_setup_example(example_xpbd_flag)
endif()

if (EXISTS ${EXAMPLES_DIR}/e_bench.cpp)
    add_executable(example_xpbd_bench ${EXAMPLES_DIR}/e_bench.cpp)
    _hina_setup_example(example_xpbd_bench)
endif()
