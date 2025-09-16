# ============================================================================
# setup_imgui.cmake
# Helper module to download and expose Dear ImGui (docking branch).
# ============================================================================

if(DEFINED _HINACLOTH_SETUP_IMGUI_INCLUDED)
    return()
endif()
set(_HINACLOTH_SETUP_IMGUI_INCLUDED TRUE)

set(IMGUI_VERSION "1.92.2b-docking" CACHE STRING "Dear ImGui docking tag to download")
set(IMGUI_BASE_URL "https://github.com/ocornut/imgui/archive/refs/tags" CACHE STRING "Base URL for Dear ImGui archives")

set(_imgui_archive "v${IMGUI_VERSION}.tar.gz")
set(_imgui_deps_dir "${CMAKE_BINARY_DIR}/_deps")
set(_imgui_archive_path "${_imgui_deps_dir}/${_imgui_archive}")
set(_imgui_source_dir "${_imgui_deps_dir}/imgui-${IMGUI_VERSION}")

if(NOT EXISTS "${_imgui_source_dir}/imgui.cpp")
    file(MAKE_DIRECTORY "${_imgui_deps_dir}")

    set(_imgui_url "${IMGUI_BASE_URL}/v${IMGUI_VERSION}.tar.gz")
    message(STATUS "Downloading Dear ImGui ${IMGUI_VERSION} from ${_imgui_url}")

    file(DOWNLOAD
        "${_imgui_url}"
        "${_imgui_archive_path}"
        SHOW_PROGRESS
        STATUS _imgui_download_status
        TLS_VERIFY ON
    )

    list(GET _imgui_download_status 0 _imgui_status_code)
    if(NOT _imgui_status_code EQUAL 0)
        list(GET _imgui_download_status 1 _imgui_status_msg)
        message(FATAL_ERROR "Failed to download Dear ImGui: ${_imgui_status_msg}")
    endif()

    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E tar xzf "${_imgui_archive_path}"
        WORKING_DIRECTORY "${_imgui_deps_dir}"
        RESULT_VARIABLE _imgui_tar_result
    )

    if(NOT _imgui_tar_result EQUAL 0)
        message(FATAL_ERROR "Failed to extract Dear ImGui archive (exit code ${_imgui_tar_result})")
    endif()
endif()

set(IMGUI_SOURCE_DIR "${_imgui_source_dir}" CACHE PATH "Absolute path to the Dear ImGui source directory" FORCE)
set(IMGUI_INCLUDE_DIR "${_imgui_source_dir}" CACHE PATH "Path to Dear ImGui headers" FORCE)

set(_IMGUI_CORE_SOURCES
    "${IMGUI_SOURCE_DIR}/imgui.cpp"
    "${IMGUI_SOURCE_DIR}/imgui_demo.cpp"
    "${IMGUI_SOURCE_DIR}/imgui_draw.cpp"
    "${IMGUI_SOURCE_DIR}/imgui_tables.cpp"
    "${IMGUI_SOURCE_DIR}/imgui_widgets.cpp"
)

function(_hinacloth_define_imgui_target)
    if(TARGET imgui::imgui)
        return()
    endif()

    add_library(imgui STATIC ${_IMGUI_CORE_SOURCES})
    add_library(imgui::imgui ALIAS imgui)

    target_include_directories(imgui
        PUBLIC
            "${IMGUI_SOURCE_DIR}"
            "${IMGUI_SOURCE_DIR}/backends"
    )
    target_compile_features(imgui PUBLIC cxx_std_17)
    set_target_properties(imgui PROPERTIES POSITION_INDEPENDENT_CODE ON)
endfunction()

function(use_imgui TARGET_NAME)
    if(NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR "use_imgui called with unknown target `${TARGET_NAME}`")
    endif()

    _hinacloth_define_imgui_target()
    target_link_libraries(${TARGET_NAME} PUBLIC imgui::imgui)
endfunction()

function(imgui_enable_backends TARGET_NAME)
    if(NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR "imgui_enable_backends called with unknown target `${TARGET_NAME}`")
    endif()

    if(NOT IMGUI_SOURCE_DIR)
        message(FATAL_ERROR "IMGUI_SOURCE_DIR is not defined. Include setup_imgui.cmake before enabling backends.")
    endif()

    cmake_parse_arguments(_IMGUI_BACKEND "" "" "BACKENDS" ${ARGN})
    if(NOT _IMGUI_BACKEND_BACKENDS)
        message(FATAL_ERROR "imgui_enable_backends requires BACKENDS to be specified")
    endif()

    _hinacloth_define_imgui_target()

    foreach(_backend IN LISTS _IMGUI_BACKEND_BACKENDS)
        string(TOLOWER "${_backend}" _backend_lower)
        if(_backend_lower STREQUAL "vulkan")
            target_sources(${TARGET_NAME} PRIVATE "${IMGUI_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp")
            if(NOT TARGET Vulkan::Vulkan)
                find_package(Vulkan REQUIRED)
            endif()
            target_link_libraries(${TARGET_NAME} PRIVATE Vulkan::Vulkan)
        elseif(_backend_lower STREQUAL "sdl3")
            target_sources(${TARGET_NAME} PRIVATE "${IMGUI_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp")
            if(COMMAND use_sdl3)
                use_sdl3(${TARGET_NAME})
            elseif(TARGET SDL3::SDL3)
                target_link_libraries(${TARGET_NAME} PRIVATE SDL3::SDL3)
            else()
                message(FATAL_ERROR "SDL3 backend requested but SDL3 is not configured. Include setup_sdl3.cmake and/or create SDL3::SDL3 target.")
            endif()
        else()
            message(FATAL_ERROR "Unknown Dear ImGui backend '${_backend}' requested")
        endif()
    endforeach()
endfunction()
