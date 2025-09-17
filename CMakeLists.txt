cmake_minimum_required(VERSION 3.26)
project(HinaCloth LANGUAGES CXX)

# Global configuration
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build a static library with current src contents only
file(GLOB HINACLOTH_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(xpbd_core STATIC ${HINACLOTH_SRC})
target_include_directories(xpbd_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Reasonable optimization flags for Release (no external deps)
if (MSVC)
    target_compile_options(xpbd_core PRIVATE
        $<$<CONFIG:Release>:/O2 /Ot /Oi /fp:fast>
    )
else()
    target_compile_options(xpbd_core PRIVATE
        $<$<CONFIG:Release>:-O3 -ffast-math -fomit-frame-pointer>
    )
endif()

# ---- Vulkan visualizer example (no changes inside subdir) ----
add_subdirectory(vulkan-visualizer)

add_executable(xpbd_vulkan
    examples/xpbd_vulkan.cpp
)
target_link_libraries(xpbd_vulkan PRIVATE vulkan_visualizer xpbd_core)
target_include_directories(xpbd_vulkan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Compile shaders for the example if glslc is available
set(XPBD_SHADER_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples/shader)
set(XPBD_SHADER_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/examples/shader)
file(MAKE_DIRECTORY ${XPBD_SHADER_BIN_DIR})

find_program(GLSLC glslc HINTS ENV VULKAN_SDK PATH_SUFFIXES Bin bin)
set(XPBD_SHADERS xpbd.vert xpbd.frag)
set(XPBD_SPV_FILES)
foreach (SH ${XPBD_SHADERS})
    set(SRC ${XPBD_SHADER_SRC_DIR}/${SH})
    set(SPV ${XPBD_SHADER_BIN_DIR}/${SH}.spv)
    if (GLSLC)
        add_custom_command(
            OUTPUT ${SPV}
            COMMAND ${GLSLC} -O -c ${SRC} -o ${SPV}
            DEPENDS ${SRC}
            COMMENT "[glslc] Compiling ${SH} -> ${SPV}"
            VERBATIM
        )
        list(APPEND XPBD_SPV_FILES ${SPV})
    endif ()
endforeach ()
if (XPBD_SPV_FILES)
    add_custom_target(xpbd_shaders DEPENDS ${XPBD_SPV_FILES})
    add_dependencies(xpbd_vulkan xpbd_shaders)
endif ()

target_compile_definitions(xpbd_vulkan PRIVATE
    SHADER_OUTPUT_DIR="${XPBD_SHADER_BIN_DIR}"
    SHADER_SOURCE_DIR="${XPBD_SHADER_SRC_DIR}"
)

if (WIN32)
    add_custom_command(TARGET xpbd_vulkan POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:xpbd_vulkan>
            $<TARGET_FILE_DIR:xpbd_vulkan>
        COMMAND_EXPAND_LISTS
    )
endif()
