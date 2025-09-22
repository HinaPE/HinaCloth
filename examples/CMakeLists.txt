add_executable(example_ex01_all_features ex_01_all_features.cpp)

target_link_libraries(example_ex01_all_features PRIVATE libhinacloth)

hinacloth_apply_common_warnings(example_ex01_all_features)
hinacloth_apply_release_opts(example_ex01_all_features)

set_target_properties(example_ex01_all_features PROPERTIES CXX_STANDARD 23 CXX_EXTENSIONS OFF)

if (HINACLOTH_WITH_TBB)
    include(../cmake/setup_tbb.cmake)
    use_tbb(example_ex01_all_features)
endif()
