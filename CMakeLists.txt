cmake_minimum_required(VERSION 3.26)
project(HinaCloth LANGUAGES CXX)

# Global configuration
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ----------------------------------------------------------------------------
# Core library: libhinacloth (XPBD implementation)
# ----------------------------------------------------------------------------
file(GLOB HINACLOTH_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library(libhinacloth STATIC ${HINACLOTH_SRC})
target_include_directories(libhinacloth PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
if (MSVC)
    target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:/O2 /Ot /Oi /fp:fast>)
else()
    target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:-O3 -ffast-math -fomit-frame-pointer>)
endif()

# Backward-compatible alias (if existing code refers to xpbd_core)
add_library(xpbd_core ALIAS libhinacloth)

# ----------------------------------------------------------------------------
# Optionally build Vulkan visualizer and examples
# ----------------------------------------------------------------------------
option(HINACLOTH_BUILD_VULKAN_VISUALIZER "Build Vulkan visualizer and examples" OFF)
if (HINACLOTH_BUILD_VULKAN_VISUALIZER)
    add_subdirectory(vulkan-visualizer)

    # XPBD visualization example (examples/example_xpbd.cpp)
    set(EXAMPLES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples)
    set(SHADER_SRC_DIR ${EXAMPLES_DIR}/shader)
    set(SHADER_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/examples/shader)
    file(MAKE_DIRECTORY ${SHADER_BIN_DIR})

    find_program(GLSLC glslc HINTS ENV VULKAN_SDK PATH_SUFFIXES Bin bin)
    if (NOT GLSLC)
        message(WARNING "glslc not found. Expect precompiled SPIR-V in ${SHADER_BIN_DIR}.")
    endif ()

    set(XPBD_SHADERS xpbd.vert xpbd.frag)
    set(XPBD_SPV)
    foreach (SH ${XPBD_SHADERS})
        set(SRC ${SHADER_SRC_DIR}/${SH})
        set(SPV ${SHADER_BIN_DIR}/${SH}.spv)
        if (GLSLC)
            add_custom_command(
                    OUTPUT ${SPV}
                    COMMAND ${GLSLC} -O -c ${SRC} -o ${SPV}
                    DEPENDS ${SRC}
                    COMMENT "[glslc] ${SH} -> ${SPV}"
                    VERBATIM
            )
            list(APPEND XPBD_SPV ${SPV})
        endif ()
    endforeach ()

    if (XPBD_SPV)
        add_custom_target(example_xpbd_shaders DEPENDS ${XPBD_SPV})
    endif ()

    add_executable(example_xpbd ${EXAMPLES_DIR}/example_xpbd.cpp)
    if (TARGET example_xpbd_shaders)
        add_dependencies(example_xpbd example_xpbd_shaders)
    endif ()

    target_compile_definitions(example_xpbd PRIVATE
            SHADER_OUTPUT_DIR="${SHADER_BIN_DIR}"
    )

    target_link_libraries(example_xpbd PRIVATE vulkan_visualizer libhinacloth)

    if (WIN32)
        add_custom_command(TARGET example_xpbd POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:example_xpbd>
                $<TARGET_FILE_DIR:example_xpbd>
            COMMAND_EXPAND_LISTS
        )
    endif ()
endif ()
