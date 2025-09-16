# ============================================================================
# setup_vma.cmake
# Helper module to download and expose the Vulkan Memory Allocator headers.
# ============================================================================

if(DEFINED _HINACLOTH_SETUP_VMA_INCLUDED)
    return()
endif()
set(_HINACLOTH_SETUP_VMA_INCLUDED TRUE)

set(VMA_VERSION "3.3.0" CACHE STRING "VulkanMemoryAllocator release to download")
set(VMA_BASE_URL "https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator/archive/refs/tags" CACHE STRING "Base URL for VMA archives")

set(_vma_archive "v${VMA_VERSION}.tar.gz")
set(_vma_deps_dir "${CMAKE_BINARY_DIR}/_deps")
set(_vma_archive_path "${_vma_deps_dir}/${_vma_archive}")
set(_vma_source_dir "${_vma_deps_dir}/VulkanMemoryAllocator-${VMA_VERSION}")

if(NOT EXISTS "${_vma_source_dir}/include/vk_mem_alloc.h")
    file(MAKE_DIRECTORY "${_vma_deps_dir}")

    set(_vma_url "${VMA_BASE_URL}/v${VMA_VERSION}.tar.gz")
    message(STATUS "Downloading VulkanMemoryAllocator ${VMA_VERSION} from ${_vma_url}")

    file(DOWNLOAD
        "${_vma_url}"
        "${_vma_archive_path}"
        SHOW_PROGRESS
        STATUS _vma_download_status
        TLS_VERIFY ON
    )

    list(GET _vma_download_status 0 _vma_status_code)
    if(NOT _vma_status_code EQUAL 0)
        list(GET _vma_download_status 1 _vma_status_msg)
        message(FATAL_ERROR "Failed to download VulkanMemoryAllocator: ${_vma_status_msg}")
    endif()

    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E tar xzf "${_vma_archive_path}"
        WORKING_DIRECTORY "${_vma_deps_dir}"
        RESULT_VARIABLE _vma_tar_result
    )

    if(NOT _vma_tar_result EQUAL 0)
        message(FATAL_ERROR "Failed to extract VulkanMemoryAllocator archive (exit code ${_vma_tar_result})")
    endif()
endif()

set(VMA_SOURCE_DIR "${_vma_source_dir}" CACHE PATH "Absolute path to the VulkanMemoryAllocator source directory" FORCE)
set(VMA_INCLUDE_DIR "${_vma_source_dir}/include" CACHE PATH "Path to VulkanMemoryAllocator headers" FORCE)

function(_hinacloth_define_vma_target)
    if(TARGET VMA::VMA)
        return()
    endif()

    add_library(VMA::VMA INTERFACE IMPORTED)
    set_target_properties(VMA::VMA PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${VMA_INCLUDE_DIR}"
    )
endfunction()

_hinacloth_define_vma_target()

function(use_vma TARGET_NAME)
    if(NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR "use_vma called with unknown target `${TARGET_NAME}`")
    endif()

    _hinacloth_define_vma_target()
    target_link_libraries(${TARGET_NAME} PUBLIC VMA::VMA)
endfunction()
