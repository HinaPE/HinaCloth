cmake_minimum_required(VERSION 3.26)
project(HinaCloth LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(HinaCloth main.cpp)
target_compile_features(HinaCloth PUBLIC cxx_std_23)