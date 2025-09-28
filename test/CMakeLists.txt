cmake_minimum_required(VERSION 3.26)

include(FetchContent)

# Fetch Catch2 (pinned version)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.5.2
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(Catch2)

add_executable(test_world_basic test_world_basic.cpp)
set_target_properties(test_world_basic PROPERTIES CXX_STANDARD 23 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)

target_link_libraries(test_world_basic PRIVATE HinaPE Catch2::Catch2WithMain)

target_include_directories(test_world_basic PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_test(NAME world_basic COMMAND test_world_basic)

