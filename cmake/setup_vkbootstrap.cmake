# ============================================================================
# setup_vkbootstrap.cmake
# Helper module to download and expose vk-bootstrap without FetchContent.
# ============================================================================

if(DEFINED _HINACLOTH_SETUP_VKBOOTSTRAP_INCLUDED)
    return()
endif()
set(_HINACLOTH_SETUP_VKBOOTSTRAP_INCLUDED TRUE)

set(VK_BOOTSTRAP_VERSION "1.4.325" CACHE STRING "vk-bootstrap release to download")
set(VK_BOOTSTRAP_BASE_URL "https://github.com/charles-lunarg/vk-bootstrap/archive/refs/tags" CACHE STRING "Base URL for vk-bootstrap archives")

set(_vkbootstrap_archive "v${VK_BOOTSTRAP_VERSION}.tar.gz")
set(_vkbootstrap_deps_dir "${CMAKE_BINARY_DIR}/_deps")
set(_vkbootstrap_archive_path "${_vkbootstrap_deps_dir}/${_vkbootstrap_archive}")
set(_vkbootstrap_source_dir "${_vkbootstrap_deps_dir}/vk-bootstrap-${VK_BOOTSTRAP_VERSION}")

if(NOT EXISTS "${_vkbootstrap_source_dir}/src/VkBootstrap.cpp")
    file(MAKE_DIRECTORY "${_vkbootstrap_deps_dir}")

    set(_vkbootstrap_url "${VK_BOOTSTRAP_BASE_URL}/v${VK_BOOTSTRAP_VERSION}.tar.gz")
    message(STATUS "Downloading vk-bootstrap ${VK_BOOTSTRAP_VERSION} from ${_vkbootstrap_url}")

    file(DOWNLOAD
        "${_vkbootstrap_url}"
        "${_vkbootstrap_archive_path}"
        SHOW_PROGRESS
        STATUS _vkbootstrap_download_status
        TLS_VERIFY ON
    )

    list(GET _vkbootstrap_download_status 0 _vkbootstrap_status_code)
    if(NOT _vkbootstrap_status_code EQUAL 0)
        list(GET _vkbootstrap_download_status 1 _vkbootstrap_status_msg)
        message(FATAL_ERROR "Failed to download vk-bootstrap: ${_vkbootstrap_status_msg}")
    endif()

    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E tar xzf "${_vkbootstrap_archive_path}"
        WORKING_DIRECTORY "${_vkbootstrap_deps_dir}"
        RESULT_VARIABLE _vkbootstrap_tar_result
    )

    if(NOT _vkbootstrap_tar_result EQUAL 0)
        message(FATAL_ERROR "Failed to extract vk-bootstrap archive (exit code ${_vkbootstrap_tar_result})")
    endif()
endif()

set(VK_BOOTSTRAP_SOURCE_DIR "${_vkbootstrap_source_dir}" CACHE PATH "Absolute path to the vk-bootstrap source directory" FORCE)
set(VK_BOOTSTRAP_INCLUDE_DIR "${_vkbootstrap_source_dir}/src" CACHE PATH "Path to vk-bootstrap headers" FORCE)

function(_hinacloth_define_vkbootstrap_target)
    if(TARGET vk-bootstrap::vk-bootstrap)
        return()
    endif()

    add_library(vk-bootstrap STATIC
        "${VK_BOOTSTRAP_SOURCE_DIR}/src/VkBootstrap.cpp"
    )
    add_library(vk-bootstrap::vk-bootstrap ALIAS vk-bootstrap)

    target_include_directories(vk-bootstrap
        PUBLIC
            "${VK_BOOTSTRAP_SOURCE_DIR}/src"
    )
    if(EXISTS "${VK_BOOTSTRAP_SOURCE_DIR}/include")
        target_include_directories(vk-bootstrap PUBLIC "${VK_BOOTSTRAP_SOURCE_DIR}/include")
    endif()
    target_compile_features(vk-bootstrap PUBLIC cxx_std_17)
    set_target_properties(vk-bootstrap PROPERTIES POSITION_INDEPENDENT_CODE ON)

    if(TARGET Vulkan::Headers)
        target_link_libraries(vk-bootstrap PUBLIC Vulkan::Headers)
    elseif(TARGET Vulkan::Vulkan)
        target_link_libraries(vk-bootstrap PUBLIC Vulkan::Vulkan)
    else()
        message(FATAL_ERROR "vk-bootstrap requires Vulkan::Headers or Vulkan::Vulkan to be available. Call find_package(Vulkan) before use_vkbootstrap().")
    endif()
endfunction()

function(use_vkbootstrap TARGET_NAME)
    if(NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR "use_vkbootstrap called with unknown target `${TARGET_NAME}`")
    endif()

    _hinacloth_define_vkbootstrap_target()
    target_link_libraries(${TARGET_NAME} PUBLIC vk-bootstrap::vk-bootstrap)
endfunction()
