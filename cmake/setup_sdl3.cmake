# ============================================================================
# setup_sdl3.cmake
# Helper module to download, build, and expose SDL3 without FetchContent.
# ============================================================================

if(DEFINED _HINACLOTH_SETUP_SDL3_INCLUDED)
    return()
endif()
set(_HINACLOTH_SETUP_SDL3_INCLUDED TRUE)

set(SDL3_VERSION "3.2.2" CACHE STRING "SDL3 release to download")
set(SDL3_BASE_URL "https://github.com/libsdl-org/SDL/archive/refs/tags" CACHE STRING "Base URL for SDL3 archives")

set(_sdl_archive "release-${SDL3_VERSION}.tar.gz")
set(_sdl_deps_dir "${CMAKE_BINARY_DIR}/_deps")
set(_sdl_archive_path "${_sdl_deps_dir}/${_sdl_archive}")
set(_sdl_source_dir "${_sdl_deps_dir}/SDL-release-${SDL3_VERSION}")
set(_sdl_build_dir "${_sdl_deps_dir}/SDL-release-${SDL3_VERSION}-build")
set(_sdl_install_dir "${_sdl_deps_dir}/SDL-release-${SDL3_VERSION}-install")

if(NOT EXISTS "${_sdl_source_dir}/CMakeLists.txt")
    file(MAKE_DIRECTORY "${_sdl_deps_dir}")

    set(_sdl_url "${SDL3_BASE_URL}/release-${SDL3_VERSION}.tar.gz")
    message(STATUS "Downloading SDL3 ${SDL3_VERSION} from ${_sdl_url}")

    file(DOWNLOAD
        "${_sdl_url}"
        "${_sdl_archive_path}"
        SHOW_PROGRESS
        STATUS _sdl_download_status
        TLS_VERIFY ON
    )

    list(GET _sdl_download_status 0 _sdl_status_code)
    if(NOT _sdl_status_code EQUAL 0)
        list(GET _sdl_download_status 1 _sdl_status_msg)
        message(FATAL_ERROR "Failed to download SDL3: ${_sdl_status_msg}")
    endif()

    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E tar xzf "${_sdl_archive_path}"
        WORKING_DIRECTORY "${_sdl_deps_dir}"
        RESULT_VARIABLE _sdl_tar_result
    )

    if(NOT _sdl_tar_result EQUAL 0)
        message(FATAL_ERROR "Failed to extract SDL3 archive (exit code ${_sdl_tar_result})")
    endif()
endif()

if(NOT EXISTS "${_sdl_install_dir}/lib/cmake/SDL3/SDL3Config.cmake")
    set(_sdl_configure_args
        "-S" "${_sdl_source_dir}"
        "-B" "${_sdl_build_dir}"
        "-DSDL_TESTS=OFF"
        "-DSDL_EXAMPLES=OFF"
        "-DSDL_INSTALL_TESTS=OFF"
        "-DCMAKE_INSTALL_PREFIX=${_sdl_install_dir}"
    )

    if(NOT CMAKE_CONFIGURATION_TYPES)
        if(CMAKE_BUILD_TYPE)
            list(APPEND _sdl_configure_args "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
            set(_sdl_single_config ${CMAKE_BUILD_TYPE})
        else()
            list(APPEND _sdl_configure_args "-DCMAKE_BUILD_TYPE=Release")
            set(_sdl_single_config Release)
        endif()
    endif()

    execute_process(
        COMMAND "${CMAKE_COMMAND}" ${_sdl_configure_args}
        RESULT_VARIABLE _sdl_configure_result
    )

    if(NOT _sdl_configure_result EQUAL 0)
        message(FATAL_ERROR "Failed to configure SDL3 (exit code ${_sdl_configure_result})")
    endif()

    set(_sdl_build_command "${CMAKE_COMMAND}" "--build" "${_sdl_build_dir}" "--target" "install")
    if(CMAKE_CONFIGURATION_TYPES)
        list(APPEND _sdl_build_command "--config" "Release")
    elseif(DEFINED _sdl_single_config)
        list(APPEND _sdl_build_command "--config" "${_sdl_single_config}")
    endif()

    execute_process(
        COMMAND ${_sdl_build_command}
        RESULT_VARIABLE _sdl_build_result
    )

    if(NOT _sdl_build_result EQUAL 0)
        message(FATAL_ERROR "Failed to build/install SDL3 (exit code ${_sdl_build_result})")
    endif()
endif()

set(SDL3_SOURCE_DIR "${_sdl_source_dir}" CACHE PATH "Absolute path to the SDL3 source directory" FORCE)
set(SDL3_INCLUDE_DIR "${_sdl_install_dir}/include" CACHE PATH "Path to SDL3 headers" FORCE)
set(SDL3_ROOT "${_sdl_install_dir}" CACHE PATH "SDL3 install prefix" FORCE)

set(SDL3_DIR "${_sdl_install_dir}/cmake" CACHE PATH "Directory containing SDL3Config.cmake" FORCE)

if(NOT TARGET SDL3::SDL3)
    find_package(SDL3 CONFIG REQUIRED PATHS "${SDL3_DIR}" NO_DEFAULT_PATH)
endif()

function(use_sdl3 TARGET_NAME)
    if(NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR "use_sdl3 called with unknown target `${TARGET_NAME}`")
    endif()

    if(NOT TARGET SDL3::SDL3)
        find_package(SDL3 CONFIG REQUIRED PATHS "${SDL3_DIR}" NO_DEFAULT_PATH)
    endif()

    target_link_libraries(${TARGET_NAME} PUBLIC SDL3::SDL3)

    if(WIN32)
        get_property(_hina_sdl3_copied TARGET ${TARGET_NAME} PROPERTY HINACLOTH_SDL3_RUNTIME_COPIED SET)
        if(NOT _hina_sdl3_copied)
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_RUNTIME_DLLS:${TARGET_NAME}>
                    $<TARGET_FILE_DIR:${TARGET_NAME}>
                COMMAND_EXPAND_LISTS
            )
            set_property(TARGET ${TARGET_NAME} PROPERTY HINACLOTH_SDL3_RUNTIME_COPIED TRUE)
        endif()
    endif()
endfunction()
