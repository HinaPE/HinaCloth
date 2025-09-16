cmake_minimum_required(VERSION 3.26)
project(HinaCloth LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(cmake/setup_pybind.cmake)
include(cmake/setup_tbb.cmake)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

add_library(hinacloth_python MODULE main.cpp)
use_pybind11(hinacloth_python)
use_tbb(hinacloth_python)
target_compile_features(hinacloth_python PUBLIC cxx_std_23)
target_link_libraries(hinacloth_python PRIVATE Python3::Module)
set_target_properties(hinacloth_python PROPERTIES
    PREFIX "${Python3_MODULE_PREFIX}"
    SUFFIX "${Python3_MODULE_EXTENSION}"
    OUTPUT_NAME "hinacloth"
)