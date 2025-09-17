cmake_minimum_required(VERSION 3.26)
project(HinaCloth LANGUAGES CXX)

option(HINACLOTH_BUILD_VULKAN_VISUALIZER "Build the Vulkan visualizer application" OFF)

# ==== Global configuration ===================================================
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

function(hinacloth_enable_release_tuning target)
    target_compile_options(${target} PRIVATE
        $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:MSVC>>:/O2 /Ot /Oi /GL /fp:fast /arch:AVX2>
        $<$<AND:$<CONFIG:Release>,$<NOT:$<CXX_COMPILER_ID:MSVC>>>:-O3 -ffast-math -fomit-frame-pointer -march=native -mavx2>
    )
endfunction()

# ==== Dependency setup =======================================================
include(cmake/setup_pybind.cmake)
include(cmake/setup_tbb.cmake)

set(Python3_FIND_STRATEGY LOCATION)
include(cmake/setup_blender.cmake)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

# ==== Python extension metadata ==============================================
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

# ==== xpbd_core module =======================================================
add_library(xpbd_core MODULE
    src/xpbd.cpp
    src/xpbd_pybind.cpp
)
use_pybind11(xpbd_core)
use_tbb(xpbd_core)

target_compile_features(xpbd_core PUBLIC cxx_std_23)
target_link_libraries(xpbd_core PRIVATE Python3::Module)
set_target_properties(xpbd_core PROPERTIES
    PREFIX "${Python3_MODULE_PREFIX}"
    SUFFIX "${_HINACLOTH_PY_EXT}"
    OUTPUT_NAME "xpbd_core"
)
hinacloth_enable_release_tuning(xpbd_core)

set(_HINACLOTH_EXTENSION_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/blender/modules")
file(MAKE_DIRECTORY "${_HINACLOTH_EXTENSION_MODULES_DIR}")

add_custom_command(TARGET xpbd_core POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_HINACLOTH_EXTENSION_MODULES_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:xpbd_core>"
        "${_HINACLOTH_EXTENSION_MODULES_DIR}/$<TARGET_FILE_NAME:xpbd_core>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_RUNTIME_DLLS:xpbd_core>
        "${_HINACLOTH_EXTENSION_MODULES_DIR}"
    COMMAND_EXPAND_LISTS
    COMMENT "Copying xpbd_core module and runtime DLLs into Blender extension modules directory"
)

file(GLOB _HINACLOTH_EXAMPLE_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/example/*.py")
hinacloth_add_blend_generation(xpbd_core ${_HINACLOTH_EXAMPLE_SCRIPTS})

# ==== Tests =================================================================
enable_testing()

add_executable(test_clothdata
    tests/test_clothdata.cpp
)
target_include_directories(test_clothdata PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
hinacloth_enable_release_tuning(test_clothdata)

add_test(NAME clothdata_verification COMMAND test_clothdata)
add_executable(test_xpbd
    tests/test_xpbd.cpp
    src/xpbd.cpp
)
target_include_directories(test_xpbd PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
use_tbb(test_xpbd)
hinacloth_enable_release_tuning(test_xpbd)

add_test(NAME xpbd_benchmark COMMAND test_xpbd)


# ==== Vulkan Visualizer ==============================================================
if (HINACLOTH_BUILD_VULKAN_VISUALIZER)
    include(cmake/setup_imgui.cmake)
    include(cmake/setup_sdl3.cmake)
    include(cmake/setup_vkbootstrap.cmake)
    include(cmake/setup_vma.cmake)
    find_package(Vulkan REQUIRED)
    set(VulkanAppName "visualizer")
    add_executable(${VulkanAppName}
            main.cpp
            vulkan/vk_engine.cpp
            vulkan/vk_engine.h
            vulkan/cloth_renderer.cpp
            vulkan/cloth_renderer.h
            vulkan/shaders/cloth_vert_spv.inc
            vulkan/shaders/cloth_frag_spv.inc
            src/xpbd.cpp
    )
    target_link_libraries(${VulkanAppName} PRIVATE Vulkan::Vulkan)
    target_include_directories(${VulkanAppName} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/vulkan)
    use_imgui(${VulkanAppName})
    use_sdl3(${VulkanAppName})
    use_vkbootstrap(${VulkanAppName})
    use_vma(${VulkanAppName})
    use_tbb(${VulkanAppName})
    imgui_enable_backends(${VulkanAppName} BACKENDS sdl3 vulkan)
    target_compile_features(${VulkanAppName} PRIVATE cxx_std_23)
    hinacloth_enable_release_tuning(${VulkanAppName})
endif()

