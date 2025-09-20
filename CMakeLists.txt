cmake_minimum_required(VERSION 3.26)
project(HinaCloth LANGUAGES CXX)

# =============================
# Global language/standard setup
# =============================
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# =============================
# Options
# =============================
# Turn warnings into errors (recommended for CI)
option(HINACLOTH_WARN_AS_ERROR "Treat compiler warnings as errors" ON)
# Build examples
option(HINACLOTH_BUILD_EXAMPLES "Build example executables" ON)
# Toggle TBB integration
option(HINACLOTH_WITH_TBB "Enable oneTBB integration" ON)

# =============================
# Helper: warnings/optimization per compiler
# =============================
include(CheckCXXCompilerFlag)
function(hinacloth_apply_common_warnings target)
    if (MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus)
        if (HINACLOTH_WARN_AS_ERROR)
            target_compile_options(${target} PRIVATE /WX)
        endif()
        target_compile_definitions(${target} PUBLIC _CRT_SECURE_NO_WARNINGS)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
        if (HINACLOTH_WARN_AS_ERROR)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

function(hinacloth_apply_release_opts target)
    if (MSVC)
        target_compile_options(${target} PRIVATE $<$<CONFIG:Release>:/O2 /Ot /Oi /fp:fast>)
    else()
        target_compile_options(${target} PRIVATE $<$<CONFIG:Release>:-O3 -ffast-math -fomit-frame-pointer>)
    endif()
endfunction()

# =============================
# Core library
# =============================
file(GLOB_RECURSE HINACLOTH_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")
add_library(libhinacloth STATIC ${HINACLOTH_SRC})

target_include_directories(libhinacloth PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
# Enforce C++23 availability for consumers
target_compile_features(libhinacloth PUBLIC cxx_std_23)
# Apply warnings/opt flags
hinacloth_apply_common_warnings(libhinacloth)

# Optional: enable AVX2 optimizations in Release if available
set(HINA_HAVE_AVX2 FALSE)
if (MSVC)
    check_cxx_compiler_flag("/arch:AVX2" HINA_MSVC_HAS_AVX2)
    if (HINA_MSVC_HAS_AVX2)
        target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:/arch:AVX2>)
        set(HINA_HAVE_AVX2 TRUE)
    endif()
else()
    check_cxx_compiler_flag("-mavx2" HINA_GCC_HAS_MAVX2)
    check_cxx_compiler_flag("-mfma"  HINA_GCC_HAS_MFMA)
    set(_AVX_FLAGS)
    if (HINA_GCC_HAS_MAVX2)
        list(APPEND _AVX_FLAGS -mavx2)
    endif()
    if (HINA_GCC_HAS_MFMA)
        list(APPEND _AVX_FLAGS -mfma)
    endif()
    if (_AVX_FLAGS)
        target_compile_options(libhinacloth PRIVATE $<$<CONFIG:Release>:${_AVX_FLAGS}>)
        set(HINA_HAVE_AVX2 TRUE)
    endif()
endif()
if (HINA_HAVE_AVX2)
    target_compile_definitions(libhinacloth PUBLIC HINACLOTH_HAVE_AVX2=1)
endif()

# =============================
# oneTBB setup (optional)
# =============================
if (HINACLOTH_WITH_TBB)
    include(cmake/setup_tbb.cmake)
    use_tbb(libhinacloth)
    target_compile_definitions(libhinacloth PUBLIC HINACLOTH_HAVE_TBB=1)
endif()

# Apply release opts after potentially adding arch flags
hinacloth_apply_release_opts(libhinacloth)

# =============================
# Vulkan visualizer (optional library)
# =============================
option(HINACLOTH_WITH_VULKAN_VISUALIZER "Enable building the Vulkan visualizer library and visual examples" ON)
if (HINACLOTH_WITH_VULKAN_VISUALIZER)
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/vulkan-visualizer/CMakeLists.txt)
        add_subdirectory(vulkan-visualizer)
        set(HINACLOTH_HAVE_VULKAN_VISUALIZER ON)
    else()
        message(WARNING "vulkan-visualizer subdirectory not found; visual examples will be skipped.")
        set(HINACLOTH_HAVE_VULKAN_VISUALIZER OFF)
    endif()
else()
    set(HINACLOTH_HAVE_VULKAN_VISUALIZER OFF)
endif()

# =============================
# Progressive Examples (ex00..ex05)
# =============================
set(HINA_SHADER_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples/shader)
set(HINA_SHADER_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/examples/shader)
if (HINACLOTH_BUILD_EXAMPLES)
    set(EXAMPLES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples)

    function(_hina_setup_example target)
        if (TARGET ${target})
            target_link_libraries(${target} PRIVATE libhinacloth)
            target_compile_features(${target} PRIVATE cxx_std_23)
            hinacloth_apply_common_warnings(${target})
            hinacloth_apply_release_opts(${target})
            if (HINACLOTH_WITH_TBB)
                use_tbb(${target})
                target_compile_definitions(${target} PRIVATE HINACLOTH_HAVE_TBB=1)
            endif()
            if (HINA_HAVE_AVX2)
                target_compile_definitions(${target} PRIVATE HINACLOTH_HAVE_AVX2=1)
            endif()
        endif()
    endfunction()

    add_executable(example_ex00_minimal    ${EXAMPLES_DIR}/ex00_minimal.cpp)
    _hina_setup_example(example_ex00_minimal)

    if (EXISTS ${EXAMPLES_DIR}/ex01_policy_and_query.cpp)
        add_executable(example_ex01_policy ${EXAMPLES_DIR}/ex01_policy_and_query.cpp)
        _hina_setup_example(example_ex01_policy)
    endif()

    if (EXISTS ${EXAMPLES_DIR}/ex02_runtime_commands.cpp)
        add_executable(example_ex02_commands ${EXAMPLES_DIR}/ex02_runtime_commands.cpp)
        _hina_setup_example(example_ex02_commands)
    endif()

    if (EXISTS ${EXAMPLES_DIR}/ex03_attachment.cpp)
        add_executable(example_ex03_attachment ${EXAMPLES_DIR}/ex03_attachment.cpp)
        _hina_setup_example(example_ex03_attachment)
    endif()

    if (EXISTS ${EXAMPLES_DIR}/ex04_bending.cpp)
        add_executable(example_ex04_bending ${EXAMPLES_DIR}/ex04_bending.cpp)
        _hina_setup_example(example_ex04_bending)
    endif()

    if (EXISTS ${EXAMPLES_DIR}/ex05_blocked_avx2.cpp)
        add_executable(example_ex05_blocked_avx2 ${EXAMPLES_DIR}/ex05_blocked_avx2.cpp)
        _hina_setup_example(example_ex05_blocked_avx2)
    endif()

    if (EXISTS ${EXAMPLES_DIR}/ex06_bench_cli.cpp)
        add_executable(example_ex06_bench ${EXAMPLES_DIR}/ex06_bench_cli.cpp)
        _hina_setup_example(example_ex06_bench)
    endif()

    # =============================
    # Visualizer-based examples (vx00..)
    # =============================
    if (HINACLOTH_HAVE_VULKAN_VISUALIZER)
        if (EXISTS ${EXAMPLES_DIR}/vx00_window.cpp)
            add_executable(visual_vx00_window ${EXAMPLES_DIR}/vx00_window.cpp)
            target_link_libraries(visual_vx00_window PRIVATE vulkan_visualizer libhinacloth)
            target_compile_features(visual_vx00_window PRIVATE cxx_std_23)
            hinacloth_apply_common_warnings(visual_vx00_window)
            hinacloth_apply_release_opts(visual_vx00_window)
            if (WIN32)
                add_custom_command(TARGET visual_vx00_window POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        $<TARGET_RUNTIME_DLLS:visual_vx00_window>
                        $<TARGET_FILE_DIR:visual_vx00_window>
                    COMMAND_EXPAND_LISTS)
            endif()
        endif()
        if (EXISTS ${EXAMPLES_DIR}/vx01_sim_overview.cpp)
            add_executable(visual_vx01_sim_overview ${EXAMPLES_DIR}/vx01_sim_overview.cpp)
            target_link_libraries(visual_vx01_sim_overview PRIVATE vulkan_visualizer libhinacloth)
            target_compile_features(visual_vx01_sim_overview PRIVATE cxx_std_23)
            hinacloth_apply_common_warnings(visual_vx01_sim_overview)
            hinacloth_apply_release_opts(visual_vx01_sim_overview)
            if (TARGET hinacloth_example_shaders)
                add_dependencies(visual_vx01_sim_overview hinacloth_example_shaders)
            endif()
            target_compile_definitions(visual_vx01_sim_overview PRIVATE SHADER_OUTPUT_DIR="${HINA_SHADER_BIN_DIR}")
            if (WIN32)
                add_custom_command(TARGET visual_vx01_sim_overview POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        $<TARGET_RUNTIME_DLLS:visual_vx01_sim_overview>
                        $<TARGET_FILE_DIR:visual_vx01_sim_overview>
                    COMMAND_EXPAND_LISTS)
            endif()
        endif()
        if (EXISTS ${EXAMPLES_DIR}/vx02_sim_controls.cpp)
            add_executable(visual_vx02_sim_controls ${EXAMPLES_DIR}/vx02_sim_controls.cpp)
            target_link_libraries(visual_vx02_sim_controls PRIVATE vulkan_visualizer libhinacloth)
            target_compile_features(visual_vx02_sim_controls PRIVATE cxx_std_23)
            hinacloth_apply_common_warnings(visual_vx02_sim_controls)
            hinacloth_apply_release_opts(visual_vx02_sim_controls)
            if (TARGET hinacloth_example_shaders)
                add_dependencies(visual_vx02_sim_controls hinacloth_example_shaders)
            endif()
            target_compile_definitions(visual_vx02_sim_controls PRIVATE SHADER_OUTPUT_DIR="${HINA_SHADER_BIN_DIR}")
            if (WIN32)
                add_custom_command(TARGET visual_vx02_sim_controls POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        $<TARGET_RUNTIME_DLLS:visual_vx02_sim_controls>
                        $<TARGET_FILE_DIR:visual_vx02_sim_controls>
                    COMMAND_EXPAND_LISTS)
            endif()
        endif()
    endif()
endif()

# =============================
# Visual example shaders (GLSL -> SPIR-V) for top-level examples
# =============================
file(MAKE_DIRECTORY ${HINA_SHADER_BIN_DIR})
find_program(HINA_GLSLC glslc HINTS ENV VULKAN_SDK PATH_SUFFIXES Bin bin)
set(HINA_SHADER_LIST cloth_lines.vert cloth_lines.frag)
set(HINA_SPV_FILES)
foreach(SH ${HINA_SHADER_LIST})
    set(SRC ${HINA_SHADER_SRC_DIR}/${SH})
    set(SPV ${HINA_SHADER_BIN_DIR}/${SH}.spv)
    if (HINA_GLSLC AND EXISTS ${SRC})
        add_custom_command(OUTPUT ${SPV}
            COMMAND ${HINA_GLSLC} -O -c ${SRC} -o ${SPV}
            DEPENDS ${SRC}
            COMMENT "[glslc] Compiling ${SH} -> ${SPV}"
            VERBATIM)
        list(APPEND HINA_SPV_FILES ${SPV})
    endif()
endforeach()
if (HINA_SPV_FILES)
    add_custom_target(hinacloth_example_shaders ALL DEPENDS ${HINA_SPV_FILES})
endif()

# =============================
# Unit tests6
# =============================
enable_testing()
file(GLOB_RECURSE HINACLOTH_TESTS_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/*.h")
if (HINACLOTH_TESTS_SRC)
    add_executable(hinacloth_tests ${HINACLOTH_TESTS_SRC})
    target_link_libraries(hinacloth_tests PRIVATE libhinacloth)
    target_include_directories(hinacloth_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    hinacloth_apply_common_warnings(hinacloth_tests)
    hinacloth_apply_release_opts(hinacloth_tests)
    if (HINACLOTH_WITH_TBB)
        use_tbb(hinacloth_tests)
        target_compile_definitions(hinacloth_tests PRIVATE HINACLOTH_HAVE_TBB=1)
    endif()
    if (HINA_HAVE_AVX2)
        target_compile_definitions(hinacloth_tests PRIVATE HINACLOTH_HAVE_AVX2=1)
    endif()
    add_test(NAME hinacloth_tests COMMAND hinacloth_tests)
endif()
