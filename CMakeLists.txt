cmake_minimum_required(VERSION 3.26)
project(HinaCloth LANGUAGES CXX)

# =============================
# Global language/standard setup
# =============================
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# =============================
# Options
# =============================
# Turn warnings into errors (recommended for CI)
option(HINACLOTH_WARN_AS_ERROR "Treat compiler warnings as errors" ON)
# Build examples
option(HINACLOTH_BUILD_EXAMPLES "Build example executables" ON)
# Toggle TBB integration
option(HINACLOTH_WITH_TBB "Enable oneTBB integration" ON)

# =============================
# Helper: warnings/optimization per compiler
# =============================
include(CheckCXXCompilerFlag)
function(hinacloth_apply_common_warnings target)
    if (MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus)
        if (HINACLOTH_WARN_AS_ERROR)
            target_compile_options(${target} PRIVATE /WX)
        endif()
        target_compile_definitions(${target} PUBLIC _CRT_SECURE_NO_WARNINGS)
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

# =============================
# Core library
# =============================
file(GLOB_RECURSE HINACLOTH_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")
add_library(libhinacloth STATIC ${HINACLOTH_SRC})

target_include_directories(libhinacloth PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
# Enforce C++23 availability for consumers
target_compile_features(libhinacloth PUBLIC cxx_std_23)
# Apply warnings/opt flags
hinacloth_apply_common_warnings(libhinacloth)

# Optional: enable AVX2 optimizations in Release if available
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

# =============================
# oneTBB setup (optional)
# =============================
if (HINACLOTH_WITH_TBB)
    include(cmake/setup_tbb.cmake)
    use_tbb(libhinacloth)
    target_compile_definitions(libhinacloth PUBLIC HINACLOTH_HAVE_TBB=1)
endif()

# Apply release opts after potentially adding arch flags
hinacloth_apply_release_opts(libhinacloth)

# =============================
 # Progressive Examples (ex00..ex05)
# =============================
if (HINACLOTH_BUILD_EXAMPLES)
    set(EXAMPLES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples)

    function(_hina_setup_example target)
        if (TARGET ${target})
            target_link_libraries(${target} PRIVATE libhinacloth)
            target_compile_features(${target} PRIVATE cxx_std_23)
            hinacloth_apply_common_warnings(${target})
            hinacloth_apply_release_opts(${target})
            if (HINACLOTH_WITH_TBB)
                use_tbb(${target})
                target_compile_definitions(${target} PRIVATE HINACLOTH_HAVE_TBB=1)
            endif()
            if (HINA_HAVE_AVX2)
                target_compile_definitions(${target} PRIVATE HINACLOTH_HAVE_AVX2=1)
            endif()
        endif()
    endfunction()

    add_executable(example_ex00_minimal    ${EXAMPLES_DIR}/ex00_minimal.cpp)
    _hina_setup_example(example_ex00_minimal)

    if (EXISTS ${EXAMPLES_DIR}/ex01_policy_and_query.cpp)
        add_executable(example_ex01_policy ${EXAMPLES_DIR}/ex01_policy_and_query.cpp)
        _hina_setup_example(example_ex01_policy)
    endif()

    if (EXISTS ${EXAMPLES_DIR}/ex02_runtime_commands.cpp)
        add_executable(example_ex02_commands ${EXAMPLES_DIR}/ex02_runtime_commands.cpp)
        _hina_setup_example(example_ex02_commands)
    endif()

    if (EXISTS ${EXAMPLES_DIR}/ex03_attachment.cpp)
        add_executable(example_ex03_attachment ${EXAMPLES_DIR}/ex03_attachment.cpp)
        _hina_setup_example(example_ex03_attachment)
    endif()

    if (EXISTS ${EXAMPLES_DIR}/ex04_bending.cpp)
        add_executable(example_ex04_bending ${EXAMPLES_DIR}/ex04_bending.cpp)
        _hina_setup_example(example_ex04_bending)
    endif()

    if (EXISTS ${EXAMPLES_DIR}/ex05_blocked_avx2.cpp)
        add_executable(example_ex05_blocked_avx2 ${EXAMPLES_DIR}/ex05_blocked_avx2.cpp)
        _hina_setup_example(example_ex05_blocked_avx2)
    endif()

    if (EXISTS ${EXAMPLES_DIR}/ex06_bench_cli.cpp)
        add_executable(example_ex06_bench ${EXAMPLES_DIR}/ex06_bench_cli.cpp)
        _hina_setup_example(example_ex06_bench)
    endif()
endif()
