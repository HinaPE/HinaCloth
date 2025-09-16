#=============================================================================
# setup_blender.cmake
# Detection utilities for Blender installations and helper functions for
# generating example .blend files.
#=============================================================================

if(DEFINED _HINACLOTH_SETUP_BLENDER_INCLUDED)
    return()
endif()
set(_HINACLOTH_SETUP_BLENDER_INCLUDED TRUE)

# Collect potential Blender installation directories.
set(_HINACLOTH_INSTALL_CANDIDATES)
if(WIN32)
    set(_hc_roots
        "$ENV{ProgramFiles}/Blender Foundation"
        "$ENV{ProgramW6432}/Blender Foundation"
        "$ENV{SystemDrive}/Program Files (x86)/Blender Foundation"
    )
    foreach(_root IN LISTS _hc_roots)
        if(_root AND EXISTS "${_root}")
            file(GLOB _installs LIST_DIRECTORIES TRUE "${_root}/Blender*")
            list(APPEND _HINACLOTH_INSTALL_CANDIDATES ${_installs})
        endif()
    endforeach()
elseif(APPLE)
    set(_hc_roots "/Applications" "$ENV{HOME}/Applications")
    foreach(_root IN LISTS _hc_roots)
        if(_root AND EXISTS "${_root}")
            file(GLOB _installs LIST_DIRECTORIES TRUE "${_root}/Blender*.app")
            list(APPEND _HINACLOTH_INSTALL_CANDIDATES ${_installs})
        endif()
    endforeach()
else()
    set(_hc_roots
        "/usr/share/blender"
        "/usr/lib/blender"
        "/usr/local/share/blender"
        "/usr/local/lib/blender"
        "/opt"
        "$ENV{HOME}/.local/share/blender"
        "$ENV{HOME}/blender"
    )
    foreach(_root IN LISTS _hc_roots)
        if(_root AND EXISTS "${_root}")
            list(APPEND _HINACLOTH_INSTALL_CANDIDATES "${_root}")
            file(GLOB _installs LIST_DIRECTORIES TRUE
                "${_root}/Blender*"
                "${_root}/blender*")
            list(APPEND _HINACLOTH_INSTALL_CANDIDATES ${_installs})
        endif()
    endforeach()
endif()

# Bring in any install derived from a preset Python root.
if(DEFINED Python3_ROOT_DIR)
    if(WIN32)
        get_filename_component(_hc_py_parent "${Python3_ROOT_DIR}" DIRECTORY)
        get_filename_component(_hc_py_install "${_hc_py_parent}" DIRECTORY)
    elseif(APPLE)
        get_filename_component(_hc_py_resources "${Python3_ROOT_DIR}" DIRECTORY)
        get_filename_component(_hc_py_contents "${_hc_py_resources}" DIRECTORY)
        get_filename_component(_hc_py_install "${_hc_py_contents}" DIRECTORY)
    else()
        get_filename_component(_hc_py_parent "${Python3_ROOT_DIR}" DIRECTORY)
        get_filename_component(_hc_py_install "${_hc_py_parent}" DIRECTORY)
    endif()
    if(_hc_py_install AND EXISTS "${_hc_py_install}")
        list(APPEND _HINACLOTH_INSTALL_CANDIDATES "${_hc_py_install}")
    endif()
endif()

if(_HINACLOTH_INSTALL_CANDIDATES)
    list(REMOVE_DUPLICATES _HINACLOTH_INSTALL_CANDIDATES)
    list(SORT _HINACLOTH_INSTALL_CANDIDATES)
    list(REVERSE _HINACLOTH_INSTALL_CANDIDATES)
endif()

# Auto-detect Blender Python if not explicitly set.
if(NOT DEFINED Python3_ROOT_DIR AND NOT Python3_EXECUTABLE)
    set(_HINACLOTH_BLENDER_PYTHON_DIRS)
    foreach(_install IN LISTS _HINACLOTH_INSTALL_CANDIDATES)
        if(WIN32)
            file(GLOB _py LIST_DIRECTORIES TRUE "${_install}/[0-9.]*/python")
        elseif(APPLE)
            file(GLOB _py LIST_DIRECTORIES TRUE "${_install}/Contents/Resources/*/python")
        else()
            file(GLOB _py LIST_DIRECTORIES TRUE
                "${_install}/[0-9.]*/python"
                "${_install}/python")
        endif()
        list(APPEND _HINACLOTH_BLENDER_PYTHON_DIRS ${_py})
    endforeach()

    if(_HINACLOTH_BLENDER_PYTHON_DIRS)
        list(REMOVE_DUPLICATES _HINACLOTH_BLENDER_PYTHON_DIRS)
        list(SORT _HINACLOTH_BLENDER_PYTHON_DIRS)
        list(REVERSE _HINACLOTH_BLENDER_PYTHON_DIRS)
        list(GET _HINACLOTH_BLENDER_PYTHON_DIRS 0 _HINACLOTH_SELECTED_PYTHON_ROOT)
        set(Python3_ROOT_DIR "${_HINACLOTH_SELECTED_PYTHON_ROOT}" CACHE PATH "Blender Python root" FORCE)
    endif()
endif()

# If we now have a Python root, ensure executable/library are configured.
if(Python3_ROOT_DIR)
    if(NOT Python3_EXECUTABLE)
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
    endif()

    if(NOT Python3_LIBRARY)
        if(WIN32)
            file(GLOB _lib_candidates
                "${Python3_ROOT_DIR}/libs/python*.lib"
                "${Python3_ROOT_DIR}/lib/python*.lib")
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

    if(WIN32)
        get_filename_component(_hc_py_parent "${Python3_ROOT_DIR}" DIRECTORY)
        get_filename_component(_hc_py_install "${_hc_py_parent}" DIRECTORY)
    elseif(APPLE)
        get_filename_component(_hc_py_resources "${Python3_ROOT_DIR}" DIRECTORY)
        get_filename_component(_hc_py_contents "${_hc_py_resources}" DIRECTORY)
        get_filename_component(_hc_py_install "${_hc_py_contents}" DIRECTORY)
    else()
        get_filename_component(_hc_py_parent "${Python3_ROOT_DIR}" DIRECTORY)
        get_filename_component(_hc_py_install "${_hc_py_parent}" DIRECTORY)
    endif()
    if(_hc_py_install AND EXISTS "${_hc_py_install}")
        list(APPEND _HINACLOTH_INSTALL_CANDIDATES "${_hc_py_install}")
        list(REMOVE_DUPLICATES _HINACLOTH_INSTALL_CANDIDATES)
    endif()
endif()

# Locate Blender executable when not set explicitly.
if(NOT HINACLOTH_BLENDER_EXECUTABLE)
    set(_HINACLOTH_BLENDER_EXE_CANDIDATES)
    foreach(_install IN LISTS _HINACLOTH_INSTALL_CANDIDATES)
        if(WIN32)
            list(APPEND _HINACLOTH_BLENDER_EXE_CANDIDATES "${_install}/blender.exe")
        elseif(APPLE)
            if(_install MATCHES "\\.app$")
                list(APPEND _HINACLOTH_BLENDER_EXE_CANDIDATES "${_install}/Contents/MacOS/Blender")
            else()
                list(APPEND _HINACLOTH_BLENDER_EXE_CANDIDATES "${_install}/MacOS/Blender")
            endif()
        else()
            list(APPEND _HINACLOTH_BLENDER_EXE_CANDIDATES "${_install}/blender")
        endif()
    endforeach()

    if(_HINACLOTH_BLENDER_EXE_CANDIDATES)
        list(REMOVE_DUPLICATES _HINACLOTH_BLENDER_EXE_CANDIDATES)
        list(SORT _HINACLOTH_BLENDER_EXE_CANDIDATES)
        list(REVERSE _HINACLOTH_BLENDER_EXE_CANDIDATES)
        foreach(_candidate IN LISTS _HINACLOTH_BLENDER_EXE_CANDIDATES)
            if(NOT HINACLOTH_BLENDER_EXECUTABLE AND _candidate AND EXISTS "${_candidate}")
                set(HINACLOTH_BLENDER_EXECUTABLE "${_candidate}" CACHE FILEPATH "Path to Blender executable" FORCE)
            endif()
        endforeach()
    endif()

    if(NOT HINACLOTH_BLENDER_EXECUTABLE)
        find_program(_hc_blender_prog NAMES blender blender.exe)
        if(_hc_blender_prog)
            set(HINACLOTH_BLENDER_EXECUTABLE "${_hc_blender_prog}" CACHE FILEPATH "Path to Blender executable" FORCE)
        endif()
    endif()

    if(HINACLOTH_BLENDER_EXECUTABLE)
        message(STATUS "HinaCloth: using Blender executable at ${HINACLOTH_BLENDER_EXECUTABLE}")
    endif()
endif()

# Helper function to register blend generation on a target.
function(hinacloth_add_blend_generation TARGET)
    if(ARGC LESS 2)
        return()
    endif()
    if(NOT HINACLOTH_BLENDER_EXECUTABLE)
        message(WARNING "HinaCloth: Blender executable not found; skipping example blend generation.")
        return()
    endif()
    foreach(_script IN LISTS ARGN)
        get_filename_component(_script_name "${_script}" NAME)
        add_custom_command(TARGET ${TARGET} POST_BUILD
            COMMAND "${HINACLOTH_BLENDER_EXECUTABLE}" --background --python "${_script}"
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            COMMENT "Generating blend file from ${_script_name}"
        )
    endforeach()
endfunction()
