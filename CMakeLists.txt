cmake_minimum_required(VERSION 3.26)
project(HinaCloth LANGUAGES CXX)

# -----------------------------------------------------------------------------
# Global configuration
# -----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Options
option(HINACLOTH_WARN_AS_ERROR "Treat compiler warnings as errors" OFF)
option(HINACLOTH_WITH_TBB      "Enable oneTBB integration"         ON)

# -----------------------------------------------------------------------------
# Helper functions (warnings, optimization)
# -----------------------------------------------------------------------------
include(CheckCXXCompilerFlag)

function(hinacloth_apply_common_warnings target)
    if (MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus)
        if (HINACLOTH_WARN_AS_ERROR)
            target_compile_options(${target} PRIVATE /WX)
        endif()
        target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
        if (HINACLOTH_WARN_AS_ERROR)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

function(hinacloth_apply_release_opts target)
    if (MSVC)
        target_compile_options(${target} PRIVATE $<$<CONFIG:Release>:/O2 /Ot /Oi /fp:fast>)
    else()
        target_compile_options(${target} PRIVATE $<$<CONFIG:Release>:-O3 -ffast-math -fomit-frame-pointer>)
    endif()
endfunction()

# -----------------------------------------------------------------------------
# Core library
# -----------------------------------------------------------------------------
file(GLOB_RECURSE HINACLOTH_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")

add_library(libhinacloth STATIC ${HINACLOTH_SRC})

# Public include path for consumers
target_include_directories(libhinacloth PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
# Enforce C++23 for consumers and this target
set_target_properties(libhinacloth PROPERTIES CXX_STANDARD 23 CXX_EXTENSIONS OFF)

# Apply warnings and optimization options
hinacloth_apply_common_warnings(libhinacloth)
hinacloth_apply_release_opts(libhinacloth)

# -----------------------------------------------------------------------------
# CPU features: AVX2 (optional, Release-only flags)
# -----------------------------------------------------------------------------
set(HINA_HAVE_AVX2 FALSE)
if (MSVC)
    check_cxx_compiler_flag("/arch:AVX2" HINA_MSVC_HAS_AVX2)
    if (HINA_MSVC_HAS_AVX2)
        target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:/arch:AVX2>)
        set(HINA_HAVE_AVX2 TRUE)
    endif()
else()
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

# -----------------------------------------------------------------------------
# Optional: oneTBB integration
# -----------------------------------------------------------------------------
if (HINACLOTH_WITH_TBB)
    include(cmake/setup_tbb.cmake)
    use_tbb(libhinacloth)
    target_compile_definitions(libhinacloth PUBLIC HINACLOTH_HAVE_TBB=1)
endif()

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------
include(CTest)
if (BUILD_TESTING)
    add_subdirectory(test)
endif()
