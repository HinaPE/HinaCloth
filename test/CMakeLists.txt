# HinaCloth tests

cmake_minimum_required(VERSION 3.26)

add_executable(hinacloth_tests
    test_main.cpp
)

target_link_libraries(hinacloth_tests PRIVATE libhinacloth)

target_include_directories(hinacloth_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)

set_target_properties(hinacloth_tests PROPERTIES CXX_STANDARD 23 CXX_EXTENSIONS OFF)

if (HINACLOTH_WITH_TBB)
    include(../cmake/setup_tbb.cmake)
    use_tbb(hinacloth_tests)
    target_compile_definitions(hinacloth_tests PUBLIC HINACLOTH_HAVE_TBB=1)
endif()

# Reuse project warning flags
hinacloth_apply_common_warnings(hinacloth_tests)

# Define individual CTest tests using a filter flag so they are reported separately
add_test(NAME test_distance_convergence COMMAND $<TARGET_FILE:hinacloth_tests> --filter distance)
add_test(NAME test_attachment_operator COMMAND $<TARGET_FILE:hinacloth_tests> --filter attachment)
add_test(NAME test_bending_convergence COMMAND $<TARGET_FILE:hinacloth_tests> --filter bending)
add_test(NAME test_pinned_node COMMAND $<TARGET_FILE:hinacloth_tests> --filter pinned)
add_test(NAME test_per_edge_compliance COMMAND $<TARGET_FILE:hinacloth_tests> --filter per_edge)
add_test(NAME test_copy_positions COMMAND $<TARGET_FILE:hinacloth_tests> --filter copypos)
