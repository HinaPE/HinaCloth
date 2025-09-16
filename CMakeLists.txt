cmake_minimum_required(VERSION 3.26)
project(HinaCloth LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(cmake/setup_pybind.cmake)
include(cmake/setup_tbb.cmake)

# Hint CMake to prefer paths we explicitly provide.
set(Python3_FIND_STRATEGY LOCATION)

if(NOT DEFINED Python3_ROOT_DIR AND NOT Python3_EXECUTABLE)
    set(_HINACLOTH_BLENDER_PYTHON_DIRS)

    if(WIN32)
        set(_HINACLOTH_BLENDER_ROOTS
            "$ENV{ProgramFiles}/Blender Foundation"
            "$ENV{ProgramW6432}/Blender Foundation"
            "$ENV{SystemDrive}/Program Files (x86)/Blender Foundation"
        )
        foreach(_root IN LISTS _HINACLOTH_BLENDER_ROOTS)
            if(_root AND EXISTS "${_root}")
                file(GLOB _candidates LIST_DIRECTORIES TRUE
                    "${_root}/Blender*/[0-9.]*/python")
                list(APPEND _HINACLOTH_BLENDER_PYTHON_DIRS ${_candidates})
            endif()
        endforeach()
    elseif(APPLE)
        set(_HINACLOTH_BLENDER_ROOTS
            "/Applications"
            "$ENV{HOME}/Applications"
        )
        foreach(_root IN LISTS _HINACLOTH_BLENDER_ROOTS)
            if(_root AND EXISTS "${_root}")
                file(GLOB _candidates LIST_DIRECTORIES TRUE
                    "${_root}/Blender*.app/Contents/Resources/*/python")
                list(APPEND _HINACLOTH_BLENDER_PYTHON_DIRS ${_candidates})
            endif()
        endforeach()
    else()
        set(_HINACLOTH_BLENDER_ROOTS
            "/usr/share/blender"
            "/usr/lib/blender"
            "/usr/local/share/blender"
            "/usr/local/lib/blender"
            "/opt"
            "$ENV{HOME}/.local/share/blender"
            "$ENV{HOME}/blender"
        )
        foreach(_root IN LISTS _HINACLOTH_BLENDER_ROOTS)
            if(_root AND EXISTS "${_root}")
                file(GLOB _candidates LIST_DIRECTORIES TRUE
                    "${_root}/Blender*/[0-9.]*/python"
                    "${_root}/blender*/[0-9.]*/python"
                    "${_root}/[0-9.]*/python")
                list(APPEND _HINACLOTH_BLENDER_PYTHON_DIRS ${_candidates})
            endif()
        endforeach()
    endif()

    if(_HINACLOTH_BLENDER_PYTHON_DIRS)
        list(REMOVE_DUPLICATES _HINACLOTH_BLENDER_PYTHON_DIRS)
        list(SORT _HINACLOTH_BLENDER_PYTHON_DIRS)
        list(REVERSE _HINACLOTH_BLENDER_PYTHON_DIRS)
        list(GET _HINACLOTH_BLENDER_PYTHON_DIRS 0 _HINACLOTH_SELECTED_PYTHON_ROOT)
        if(EXISTS "${_HINACLOTH_SELECTED_PYTHON_ROOT}")
            message(STATUS "HinaCloth: auto-detected Blender Python at ${_HINACLOTH_SELECTED_PYTHON_ROOT}")
            set(Python3_ROOT_DIR "${_HINACLOTH_SELECTED_PYTHON_ROOT}" CACHE PATH "Blender Python root" FORCE)

            if(WIN32)
                set(_HINACLOTH_EXECUTABLE "${Python3_ROOT_DIR}/bin/python.exe")
            else()
                file(GLOB _exec_candidates
                    "${Python3_ROOT_DIR}/bin/python3*"
                    "${Python3_ROOT_DIR}/bin/python")
                if(_exec_candidates)
                    list(SORT _exec_candidates)
                    list(REVERSE _exec_candidates)
                    list(GET _exec_candidates 0 _HINACLOTH_EXECUTABLE)
                endif()
            endif()

            if(_HINACLOTH_EXECUTABLE AND EXISTS "${_HINACLOTH_EXECUTABLE}")
                set(Python3_EXECUTABLE "${_HINACLOTH_EXECUTABLE}" CACHE FILEPATH "Blender Python executable" FORCE)
            endif()

            if(WIN32)
                file(GLOB _lib_candidates
                    "${Python3_ROOT_DIR}/lib/python*.lib"
                    "${Python3_ROOT_DIR}/libs/python*.lib")
            elseif(APPLE)
                file(GLOB _lib_candidates "${Python3_ROOT_DIR}/lib/libpython*.dylib")
            else()
                file(GLOB _lib_candidates
                    "${Python3_ROOT_DIR}/lib/libpython*.so"
                    "${Python3_ROOT_DIR}/lib/libpython*.a")
            endif()

            if(_lib_candidates)
                list(SORT _lib_candidates)
                list(REVERSE _lib_candidates)
                list(GET _lib_candidates 0 _HINACLOTH_PYTHON_LIBRARY)
                if(EXISTS "${_HINACLOTH_PYTHON_LIBRARY}")
                    set(Python3_LIBRARY "${_HINACLOTH_PYTHON_LIBRARY}" CACHE FILEPATH "Blender Python library" FORCE)
                endif()
            endif()
        endif()
    endif()
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

set(_HINACLOTH_PY_EXT "")
if(DEFINED Python3_EXTENSION_SUFFIX AND Python3_EXTENSION_SUFFIX)
    set(_HINACLOTH_PY_EXT "${Python3_EXTENSION_SUFFIX}")
elseif(DEFINED Python3_MODULE_EXTENSION AND Python3_MODULE_EXTENSION)
    set(_HINACLOTH_PY_EXT "${Python3_MODULE_EXTENSION}")
elseif(WIN32)
    set(_HINACLOTH_PY_EXT ".pyd")
else()
    set(_HINACLOTH_PY_EXT ".so")
endif()

add_library(xpbd_core MODULE
    src/xpbd.cpp
    src/xpbd_pybind.cpp
)
use_pybind11(xpbd_core)

set(_HINACLOTH_EXTENSION_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/blender/modules")
file(MAKE_DIRECTORY "${_HINACLOTH_EXTENSION_MODULES_DIR}")

add_custom_command(TARGET xpbd_core POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_HINACLOTH_EXTENSION_MODULES_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:xpbd_core>"
        "${_HINACLOTH_EXTENSION_MODULES_DIR}/$<TARGET_FILE_NAME:xpbd_core>"
    COMMENT "Copying xpbd_core module into Blender extension modules directory"
)

target_compile_features(xpbd_core PUBLIC cxx_std_23)
target_link_libraries(xpbd_core PRIVATE Python3::Module)
use_tbb(xpbd_core)
set_target_properties(xpbd_core PROPERTIES
    PREFIX "${Python3_MODULE_PREFIX}"
    SUFFIX "${_HINACLOTH_PY_EXT}"
    OUTPUT_NAME "xpbd_core"
)

target_compile_options(xpbd_core PRIVATE
    $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:MSVC>>:/O2 /Ot /Oi /GL /fp:fast /arch:AVX2>
    $<$<AND:$<CONFIG:Release>,$<NOT:$<CXX_COMPILER_ID:MSVC>>>:-O3 -ffast-math -fomit-frame-pointer -march=native -mavx2>
)

enable_testing()
add_executable(test_xpbd
        tests/test_xpbd.cpp
        src/xpbd.cpp
)
target_include_directories(test_xpbd PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_options(test_xpbd PRIVATE
        $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:MSVC>>:/O2 /Ot /Oi /GL /fp:fast /arch:AVX2>
        $<$<AND:$<CONFIG:Release>,$<NOT:$<CXX_COMPILER_ID:MSVC>>>:-O3 -ffast-math -fomit-frame-pointer -march=native -mavx2>
)
use_tbb(test_xpbd)
add_test(NAME xpbd_benchmark COMMAND test_xpbd)
