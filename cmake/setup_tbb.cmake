include_guard(GLOBAL)

set(HINACLOTH_TBB_VERSION "2022.2.0" CACHE STRING "Preferred oneTBB release tag")
option(HINACLOTH_TBB_FETCH "Allow FetchContent fallback to build oneTBB locally" ON)

if (NOT TARGET TBB::tbb)
    find_package(TBB CONFIG QUIET)
endif ()

if (NOT TARGET TBB::tbb AND HINACLOTH_TBB_FETCH)
    include(FetchContent)

    set(TBB_TEST OFF CACHE BOOL "" FORCE)
    set(TBB_STRICT OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(hinacloth_oneTBB
        URL https://github.com/uxlfoundation/oneTBB/archive/refs/tags/v${HINACLOTH_TBB_VERSION}.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP OFF
    )
    FetchContent_MakeAvailable(hinacloth_oneTBB)

    if (NOT TARGET TBB::tbb AND TARGET tbb)
        add_library(TBB::tbb ALIAS tbb)
    endif ()
endif ()

if (NOT TARGET TBB::tbb)
    message(FATAL_ERROR "oneTBB could not be located. Install it or enable HINACLOTH_TBB_FETCH to build a local copy.")
endif ()

function(use_tbb target)
    if (NOT TARGET ${target})
        message(FATAL_ERROR "use_tbb called with unknown target `${target}`")
    endif ()

    target_link_libraries(${target} PUBLIC TBB::tbb)

    if (WIN32)
        get_target_property(_hina_target_type ${target} TYPE)
        if (_hina_target_type STREQUAL "EXECUTABLE" OR _hina_target_type STREQUAL "SHARED_LIBRARY" OR _hina_target_type STREQUAL "MODULE_LIBRARY")
            add_custom_command(TARGET ${target} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:${target}> $<TARGET_FILE_DIR:${target}>
                COMMAND_EXPAND_LISTS
            )
        endif ()
    endif ()
endfunction()
