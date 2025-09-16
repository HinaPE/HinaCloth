cmake_minimum_required(VERSION 3.26)
project(HinaCloth LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(cmake/setup_pybind.cmake)
include(cmake/setup_tbb.cmake)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

add_library(xpbd_core MODULE
    src/xpbd.cpp
)
use_pybind11(xpbd_core)

target_compile_features(xpbd_core PUBLIC cxx_std_23)
target_link_libraries(xpbd_core PRIVATE Python3::Module)
use_tbb(xpbd_core)
set_target_properties(xpbd_core PROPERTIES
    PREFIX "${Python3_MODULE_PREFIX}"
    SUFFIX "${Python3_MODULE_EXTENSION}"
    OUTPUT_NAME "xpbd_core"
)

target_compile_options(xpbd_core PRIVATE
    $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:MSVC>>:/O2 /Ot /Oi /GL /fp:fast /arch:AVX2>
    $<$<AND:$<CONFIG:Release>,$<NOT:$<CXX_COMPILER_ID:MSVC>>>:-O3 -ffast-math -fomit-frame-pointer -march=native -mavx2>
)
