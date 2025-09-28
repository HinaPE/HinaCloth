cmake_minimum_required(VERSION 3.26)

add_executable(example_minimal main.cpp)
set_target_properties(example_minimal PROPERTIES CXX_STANDARD 23 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)

target_link_libraries(example_minimal PRIVATE HinaPE)

target_include_directories(example_minimal PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

