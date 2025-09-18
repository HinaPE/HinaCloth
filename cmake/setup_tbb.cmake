if(DEFINED _HINACLOTH_SETUP_TBB_INCLUDED)
    return()
endif()
set(_HINACLOTH_SETUP_TBB_INCLUDED TRUE)

# Minimal, cross-platform oneTBB setup: download, build+install Release only, expose TBB::tbb,
# and provide a single helper `use_tbb(target)` to link + ensure Release mapping.

set(TBB_VERSION "2022.2.0" CACHE STRING "oneTBB release to download")
set(TBB_BASE_URL "https://github.com/uxlfoundation/oneTBB/archive/refs/tags" CACHE STRING "Base URL for oneTBB archives")

set(_tbb_archive v${TBB_VERSION}.tar.gz)
set(_tbb_deps_dir "${CMAKE_BINARY_DIR}/_deps")
set(_tbb_archive_path "${_tbb_deps_dir}/${_tbb_archive}")
set(_tbb_source_dir "${_tbb_deps_dir}/oneTBB-${TBB_VERSION}")
set(_tbb_build_dir "${_tbb_deps_dir}/oneTBB-${TBB_VERSION}-build")
set(_tbb_install_dir "${_tbb_deps_dir}/oneTBB-${TBB_VERSION}-install")

if(NOT EXISTS "${_tbb_source_dir}/CMakeLists.txt")
    file(MAKE_DIRECTORY "${_tbb_deps_dir}")
    set(_tbb_url "${TBB_BASE_URL}/v${TBB_VERSION}.tar.gz")
    message(STATUS "Downloading oneTBB ${TBB_VERSION} from ${_tbb_url}")
    file(DOWNLOAD "${_tbb_url}" "${_tbb_archive_path}" SHOW_PROGRESS STATUS _dl TLS_VERIFY ON)
    list(GET _dl 0 _code)
    if(NOT _code EQUAL 0)
        list(GET _dl 1 _msg)
        message(FATAL_ERROR "Failed to download oneTBB: ${_msg}")
    endif()
    execute_process(COMMAND "${CMAKE_COMMAND}" -E tar xzf "${_tbb_archive_path}" WORKING_DIRECTORY "${_tbb_deps_dir}" RESULT_VARIABLE _tar)
    if(NOT _tar EQUAL 0)
        message(FATAL_ERROR "Failed to extract oneTBB archive (exit code ${_tar})")
    endif()
endif()

if(NOT EXISTS "${_tbb_install_dir}/lib/cmake/TBB/TBBConfig.cmake")
    execute_process(COMMAND "${CMAKE_COMMAND}" -S "${_tbb_source_dir}" -B "${_tbb_build_dir}"
        -DTBB_TEST=OFF -DTBB_STRICT=OFF -DCMAKE_INSTALL_PREFIX=${_tbb_install_dir}
        RESULT_VARIABLE _cfg)
    if(NOT _cfg EQUAL 0)
        message(FATAL_ERROR "Failed to configure oneTBB (exit code ${_cfg})")
    endif()
    # Always build/install Release to unify across platforms and configurations
    execute_process(COMMAND "${CMAKE_COMMAND}" --build "${_tbb_build_dir}" --target install --config Release RESULT_VARIABLE _bld)
    if(NOT _bld EQUAL 0)
        message(FATAL_ERROR "Failed to build/install oneTBB Release (exit code ${_bld})")
    endif()
endif()

set(TBB_DIR "${_tbb_install_dir}/lib/cmake/TBB" CACHE PATH "TBB CMake package dir" FORCE)
find_package(TBB CONFIG REQUIRED PATHS "${TBB_DIR}" NO_DEFAULT_PATH)

function(use_tbb target)
    if(NOT TARGET ${target})
        message(FATAL_ERROR "use_tbb called with unknown target `${target}`")
    endif()
    if(NOT TARGET TBB::tbb)
        find_package(TBB CONFIG REQUIRED PATHS "${TBB_DIR}" NO_DEFAULT_PATH)
    endif()
    target_link_libraries(${target} PUBLIC TBB::tbb)
    # Force all non-Release configurations to reuse Release artifacts
    if(CMAKE_CONFIGURATION_TYPES)
        set_target_properties(TBB::tbb PROPERTIES
            MAP_IMPORTED_CONFIG_DEBUG Release
            MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release
            MAP_IMPORTED_CONFIG_MINSIZEREL Release)
        if (WIN32)
            # Point Debug import library to Release import lib to avoid *_debug naming
            if (EXISTS "${_tbb_install_dir}/lib/tbb12.lib")
                set_target_properties(TBB::tbb PROPERTIES IMPORTED_IMPLIB_DEBUG "${_tbb_install_dir}/lib/tbb12.lib")
            endif()
        endif()
    endif()
    # For executable/shared targets, copy runtime next to the binary for convenience
    get_target_property(_type ${target} TYPE)
    if(_type STREQUAL "EXECUTABLE" OR _type STREQUAL "SHARED_LIBRARY" OR _type STREQUAL "MODULE_LIBRARY")
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:TBB::tbb> $<TARGET_FILE_DIR:${target}>
            VERBATIM)
    endif()
endfunction()
