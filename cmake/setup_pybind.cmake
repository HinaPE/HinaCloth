# ============================================================================
# setup_pybind.cmake
# Helper module to download and expose pybind11 without using FetchContent.
# ============================================================================

if(DEFINED _HINACLOTH_SETUP_PYBIND_INCLUDED)
    return()
endif()
set(_HINACLOTH_SETUP_PYBIND_INCLUDED TRUE)

set(PYBIND11_VERSION "3.0.1" CACHE STRING "pybind11 release to download")
set(PYBIND11_BASE_URL "https://github.com/pybind/pybind11/archive/refs/tags" CACHE STRING "Base URL for pybind11 archives")

set(_pybind_archive "v${PYBIND11_VERSION}.tar.gz")
set(_pybind_deps_dir "${CMAKE_BINARY_DIR}/_deps")
set(_pybind_archive_path "${_pybind_deps_dir}/${_pybind_archive}")
set(_pybind_source_dir "${_pybind_deps_dir}/pybind11-${PYBIND11_VERSION}")

if(NOT EXISTS "${_pybind_source_dir}/include/pybind11/pybind11.h")
    file(MAKE_DIRECTORY "${_pybind_deps_dir}")

    set(_pybind_url "${PYBIND11_BASE_URL}/v${PYBIND11_VERSION}.tar.gz")
    message(STATUS "Downloading pybind11 ${PYBIND11_VERSION} from ${_pybind_url}")

    file(DOWNLOAD
        "${_pybind_url}"
        "${_pybind_archive_path}"
        SHOW_PROGRESS
        STATUS _pybind_download_status
        TLS_VERIFY ON
    )

    list(GET _pybind_download_status 0 _pybind_status_code)
    if(NOT _pybind_status_code EQUAL 0)
        list(GET _pybind_download_status 1 _pybind_status_msg)
        message(FATAL_ERROR "Failed to download pybind11: ${_pybind_status_msg}")
    endif()

    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E tar xzf "${_pybind_archive_path}"
        WORKING_DIRECTORY "${_pybind_deps_dir}"
        RESULT_VARIABLE _pybind_tar_result
    )

    if(NOT _pybind_tar_result EQUAL 0)
        message(FATAL_ERROR "Failed to extract pybind11 archive (exit code ${_pybind_tar_result})")
    endif()
endif()

set(PYBIND11_SOURCE_DIR "${_pybind_source_dir}" CACHE PATH "Absolute path to the pybind11 source directory" FORCE)
set(PYBIND11_INCLUDE_DIR "${PYBIND11_SOURCE_DIR}/include" CACHE PATH "Path to pybind11 headers" FORCE)

if(EXISTS "${PYBIND11_SOURCE_DIR}/tools/pybind11Tools.cmake")
    list(APPEND CMAKE_MODULE_PATH "${PYBIND11_SOURCE_DIR}/tools")
endif()

function(use_pybind11 TARGET_NAME)
    if(NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR "use_pybind11 called with unknown target `${TARGET_NAME}`")
    endif()
    target_include_directories(${TARGET_NAME} PUBLIC "${PYBIND11_INCLUDE_DIR}")
endfunction()
