# ============================================================================
# setup_tbb.cmake
# Helper module to download and expose oneTBB without using FetchContent.
# ============================================================================

if(DEFINED _HINACLOTH_SETUP_TBB_INCLUDED)
    return()
endif()
set(_HINACLOTH_SETUP_TBB_INCLUDED TRUE)

set(TBB_VERSION "2022.2.0" CACHE STRING "oneTBB release to download")
set(TBB_BASE_URL "https://github.com/uxlfoundation/oneTBB/archive/refs/tags" CACHE STRING "Base URL for oneTBB archives")

set(_tbb_archive "v${TBB_VERSION}.tar.gz")
set(_tbb_deps_dir "${CMAKE_BINARY_DIR}/_deps")
set(_tbb_archive_path "${_tbb_deps_dir}/${_tbb_archive}")
set(_tbb_source_dir "${_tbb_deps_dir}/oneTBB-${TBB_VERSION}")
set(_tbb_build_dir "${_tbb_deps_dir}/oneTBB-${TBB_VERSION}-build")
set(_tbb_install_dir "${_tbb_deps_dir}/oneTBB-${TBB_VERSION}-install")

if(NOT EXISTS "${_tbb_source_dir}/CMakeLists.txt")
    file(MAKE_DIRECTORY "${_tbb_deps_dir}")

    set(_tbb_url "${TBB_BASE_URL}/v${TBB_VERSION}.tar.gz")
    message(STATUS "Downloading oneTBB ${TBB_VERSION} from ${_tbb_url}")

    file(DOWNLOAD
        "${_tbb_url}"
        "${_tbb_archive_path}"
        SHOW_PROGRESS
        STATUS _tbb_download_status
        TLS_VERIFY ON
    )

    list(GET _tbb_download_status 0 _tbb_status_code)
    if(NOT _tbb_status_code EQUAL 0)
        list(GET _tbb_download_status 1 _tbb_status_msg)
        message(FATAL_ERROR "Failed to download oneTBB: ${_tbb_status_msg}")
    endif()

    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E tar xzf "${_tbb_archive_path}"
        WORKING_DIRECTORY "${_tbb_deps_dir}"
        RESULT_VARIABLE _tbb_tar_result
    )

    if(NOT _tbb_tar_result EQUAL 0)
        message(FATAL_ERROR "Failed to extract oneTBB archive (exit code ${_tbb_tar_result})")
    endif()
endif()

if(NOT EXISTS "${_tbb_install_dir}/include/tbb/tbb.h")
    set(_tbb_configure_args
        "-S" "${_tbb_source_dir}"
        "-B" "${_tbb_build_dir}"
        "-DTBB_TEST=OFF"
        "-DTBB_STRICT=OFF"
        "-DCMAKE_INSTALL_PREFIX=${_tbb_install_dir}"
    )

    if(NOT CMAKE_CONFIGURATION_TYPES)
        if(CMAKE_BUILD_TYPE)
            list(APPEND _tbb_configure_args "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
            set(_tbb_single_config ${CMAKE_BUILD_TYPE})
        else()
            list(APPEND _tbb_configure_args "-DCMAKE_BUILD_TYPE=Release")
            set(_tbb_single_config Release)
        endif()
    endif()

    execute_process(
        COMMAND "${CMAKE_COMMAND}" ${_tbb_configure_args}
        RESULT_VARIABLE _tbb_configure_result
    )

    if(NOT _tbb_configure_result EQUAL 0)
        message(FATAL_ERROR "Failed to configure oneTBB (exit code ${_tbb_configure_result})")
    endif()

    set(_tbb_build_command "${CMAKE_COMMAND}" "--build" "${_tbb_build_dir}" "--target" "install")
    if(CMAKE_CONFIGURATION_TYPES)
        list(APPEND _tbb_build_command "--config" "Release")
    elseif(DEFINED _tbb_single_config)
        list(APPEND _tbb_build_command "--config" "${_tbb_single_config}")
    endif()

    execute_process(
        COMMAND ${_tbb_build_command}
        RESULT_VARIABLE _tbb_build_result
    )

    if(NOT _tbb_build_result EQUAL 0)
        message(FATAL_ERROR "Failed to build/install oneTBB (exit code ${_tbb_build_result})")
    endif()
endif()

set(TBB_SOURCE_DIR "${_tbb_source_dir}" CACHE PATH "Absolute path to the oneTBB source directory" FORCE)
set(TBB_INCLUDE_DIR "${_tbb_install_dir}/include" CACHE PATH "Path to oneTBB headers" FORCE)
set(TBB_ROOT "${_tbb_install_dir}" CACHE PATH "oneTBB install prefix" FORCE)

set(TBB_DIR "${_tbb_install_dir}/lib/cmake/tbb" CACHE PATH "Directory containing TBBConfig.cmake" FORCE)

if(NOT TARGET TBB::tbb)
    find_package(TBB CONFIG REQUIRED PATHS "${TBB_DIR}" NO_DEFAULT_PATH)
endif()

get_target_property(_hina_tbb_type TBB::tbb TYPE)
set(_HINACLOTH_TBB_RUNTIME_COPY FALSE)
if(_hina_tbb_type STREQUAL "SHARED_LIBRARY" OR _hina_tbb_type STREQUAL "MODULE_LIBRARY" OR _hina_tbb_type STREQUAL "UNKNOWN_LIBRARY")
    set(_HINACLOTH_TBB_RUNTIME_COPY TRUE)
endif()

if(NOT _HINACLOTH_TBB_RUNTIME_COPY)
    get_target_property(_hina_tbb_location TBB::tbb IMPORTED_LOCATION)
    if(_hina_tbb_location MATCHES "\\.(dll|so(\\.[0-9]+)*|dylib)$")
        set(_HINACLOTH_TBB_RUNTIME_COPY TRUE)
    endif()
endif()

if(NOT _HINACLOTH_TBB_RUNTIME_COPY)
    get_target_property(_hina_tbb_configs TBB::tbb IMPORTED_CONFIGURATIONS)
    if(_hina_tbb_configs)
        foreach(_hina_tbb_cfg IN LISTS _hina_tbb_configs)
            string(TOUPPER "${_hina_tbb_cfg}" _hina_tbb_cfg_upper)
            get_target_property(_hina_tbb_loc_cfg TBB::tbb IMPORTED_LOCATION_${_hina_tbb_cfg_upper})
            if(_hina_tbb_loc_cfg MATCHES "\\.(dll|so(\\.[0-9]+)*|dylib)$")
                set(_HINACLOTH_TBB_RUNTIME_COPY TRUE)
                break()
            endif()
        endforeach()
    endif()
endif()

function(use_tbb TARGET_NAME)
    if(NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR "use_tbb called with unknown target `${TARGET_NAME}`")
    endif()

    if(NOT TARGET TBB::tbb)
        find_package(TBB CONFIG REQUIRED PATHS "${TBB_DIR}" NO_DEFAULT_PATH)
    endif()

    target_link_libraries(${TARGET_NAME} PUBLIC TBB::tbb)

    if(_HINACLOTH_TBB_RUNTIME_COPY)
        get_property(_hina_tbb_copied TARGET ${TARGET_NAME} PROPERTY HINACLOTH_TBB_RUNTIME_COPIED SET)
        if(NOT _hina_tbb_copied)
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:TBB::tbb>
                    $<TARGET_FILE_DIR:${TARGET_NAME}>
                COMMAND_EXPAND_LISTS
            )
            if(WIN32)
                add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        $<TARGET_RUNTIME_DLLS:${TARGET_NAME}>
                        $<TARGET_FILE_DIR:${TARGET_NAME}>
                    COMMAND_EXPAND_LISTS
                )
            endif()
            set_property(TARGET ${TARGET_NAME} PROPERTY HINACLOTH_TBB_RUNTIME_COPIED TRUE)
        endif()
    endif()
endfunction()
