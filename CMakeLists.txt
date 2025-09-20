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
file(GLOB_RECURSE HINACLOTH_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")
add_library(libhinacloth STATIC ${HINACLOTH_SRC})
target_include_directories(libhinacloth PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
include(CheckCXXCompilerFlag)
if (MSVC)
    target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:/O2 /Ot /Oi /fp:fast>)
    check_cxx_compiler_flag("/arch:AVX2" HINACLOTH_CXX_HAS_ARCH_AVX2)
    if (HINACLOTH_CXX_HAS_ARCH_AVX2)
        target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:/arch:AVX2>)
    endif()
else()
    target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:-O3 -ffast-math -fomit-frame-pointer>)
    check_cxx_compiler_flag("-mavx2" HINACLOTH_CXX_HAS_MAVX2)
    check_cxx_compiler_flag("-mfma"  HINACLOTH_CXX_HAS_MFMA)
    set(_HINA_AVX_FLAGS)
    if (HINACLOTH_CXX_HAS_MAVX2)
        list(APPEND _HINA_AVX_FLAGS -mavx2)
    endif()
    if (HINACLOTH_CXX_HAS_MFMA)
        list(APPEND _HINA_AVX_FLAGS -mfma)
    endif()
    if (_HINA_AVX_FLAGS)
        target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:${_HINA_AVX_FLAGS}>)
    endif()
endif()

# ----------------------------------------------------------------------------
# Optionally build Vulkan visualizer and examples
# ----------------------------------------------------------------------------
option(HINACLOTH_BUILD_VULKAN_VISUALIZER "Build Vulkan visualizer and examples" ON)
option(HINACLOTH_BUILD_TESTS "Build XPBD tests/benchmarks" OFF)

# Setup oneTBB via helper script
include(cmake/setup_tbb.cmake)
# Attach oneTBB to libhinacloth and expose availability to dependents
use_tbb(libhinacloth)
target_compile_definitions(libhinacloth PUBLIC HINACLOTH_HAVE_TBB=1)

if (HINACLOTH_BUILD_VULKAN_VISUALIZER)
    add_subdirectory(vulkan-visualizer)

    # XPBD visualization example (examples/example_xpbd.cpp)
    set(EXAMPLES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples)
#    set(SHADER_SRC_DIR ${EXAMPLES_DIR}/shader)
#    set(SHADER_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/examples/shader)
#    file(MAKE_DIRECTORY ${SHADER_BIN_DIR})
#
#    find_program(GLSLC glslc HINTS ENV VULKAN_SDK PATH_SUFFIXES Bin bin)
#    if (NOT GLSLC)
#        message(WARNING "glslc not found. Expect precompiled SPIR-V in ${SHADER_BIN_DIR}.")
#    endif ()
#
#    set(XPBD_SHADERS xpbd.vert xpbd.frag)
#    set(XPBD_SPV)
#    foreach (SH ${XPBD_SHADERS})
#        set(SRC ${SHADER_SRC_DIR}/${SH})
#        set(SPV ${SHADER_BIN_DIR}/${SH}.spv)
#        if (GLSLC)
#            add_custom_command(
#                    OUTPUT ${SPV}
#                    COMMAND ${GLSLC} -O -c ${SRC} -o ${SPV}
#                    DEPENDS ${SRC}
#                    COMMENT "[glslc] ${SH} -> ${SPV}"
#                    VERBATIM
#            )
#            list(APPEND XPBD_SPV ${SPV})
#        endif ()
#    endforeach ()
#
#    if (XPBD_SPV)
#        add_custom_target(example_xpbd_shaders DEPENDS ${XPBD_SPV})
#    endif ()

    add_executable(example_xpbd ${EXAMPLES_DIR}/e_min.cpp)
    if (TARGET example_xpbd_shaders)
        add_dependencies(example_xpbd example_xpbd_shaders)
    endif ()

    target_compile_definitions(example_xpbd PRIVATE
            SHADER_OUTPUT_DIR="${SHADER_BIN_DIR}"
    )

    target_link_libraries(example_xpbd PRIVATE vulkan_visualizer libhinacloth)
    # Use helper to attach TBB to executable (handles runtime copy)
    use_tbb(example_xpbd)

    if (WIN32)
        add_custom_command(TARGET example_xpbd POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:example_xpbd>
                $<TARGET_FILE_DIR:example_xpbd>
            COMMAND_EXPAND_LISTS
        )
    endif ()

    # Additional comprehensive example covering more features
    add_executable(example_xpbd_max ${EXAMPLES_DIR}/e_max.cpp)
    target_link_libraries(example_xpbd_max PRIVATE libhinacloth)
    # Attach TBB as well for parity
    use_tbb(example_xpbd_max)
    if (WIN32)
        add_custom_command(TARGET example_xpbd_max POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:example_xpbd_max>
                $<TARGET_FILE_DIR:example_xpbd_max>
            COMMAND_EXPAND_LISTS
        )
    endif ()
endif ()

## ----------------------------------------------------------------------------
## Tests / Benchmarks
## ----------------------------------------------------------------------------
#if (HINACLOTH_BUILD_TESTS)
#    add_executable(test_bench_xpbd test/test_bench_xpbd.cpp)
#    target_link_libraries(test_bench_xpbd PRIVATE libhinacloth)
#    use_tbb(test_bench_xpbd)
#
#    add_executable(test_validation_xpbd test/test_validation_xpbd.cpp)
#    target_link_libraries(test_validation_xpbd PRIVATE libhinacloth)
#    use_tbb(test_validation_xpbd)
#
#    enable_testing()
#    add_test(NAME validation_xpbd COMMAND test_validation_xpbd)
#    add_test(NAME bench_xpbd COMMAND test_bench_xpbd)
#endif ()
