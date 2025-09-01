add_executable(unittests
    main.cpp
)

target_compile_options(unittests PUBLIC "-O3" "-Wall")

target_include_directories(unittests
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>/src
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

target_link_libraries(unittests
    PRIVATE
        gafro
)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(unittests PUBLIC "-Wno-class-memaccess")
endif()

target_compile_options(unittests PUBLIC "-Wno-unused-variable")

if (APPLE)
    target_compile_options(unittests PUBLIC "-Wno-unused-command-line-argument")
endif()

add_subdirectory(algebra)
add_subdirectory(optimization)
add_subdirectory(physics)
add_subdirectory(robots)

# Run the unit tests
add_custom_target(run-unittests ALL unittests
                  DEPENDS unittests
                  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                  COMMENT "Unit testing..." VERBATIM
)
