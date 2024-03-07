cmake_minimum_required(VERSION 3.11)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

project(gafro)

set(${PROJECT_NAME}_VERSION_MAJOR 1)
set(${PROJECT_NAME}_VERSION_MINOR 0)
set(${PROJECT_NAME}_VERSION_PATCH 0)

set(${PROJECT_NAME}_VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(GAFRO_PREFIX_PACKAGE share/${CMAKE_PROJECT_NAME}/cmake)
set(GAFRO_PREFIX_INCLUDE include/gafro)

# set(DEBUG_MODE FALSE)
# set(GAFRO_ASSETS_DIRECTORY "${CMAKE_INSTALL_PREFIX}/share/${CMAKE_PROJECT_NAME}/assets/")

# file(COPY "assets/" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/assets/")

find_package(Eigen3 3.4 REQUIRED)


add_subdirectory(src)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    add_subdirectory(tests)

    export(EXPORT ${CMAKE_PROJECT_NAME}-config-targets
        FILE "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config-targets.cmake"
        NAMESPACE gafro::
    )

    install(EXPORT ${CMAKE_PROJECT_NAME}-config-targets
        FILE ${CMAKE_PROJECT_NAME}-config-targets.cmake
        NAMESPACE gafro::
        DESTINATION ${GAFRO_PREFIX_PACKAGE}
    )

    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config-version.cmake"
        VERSION ${${PROJECT_NAME}_VERSION}
        COMPATIBILITY AnyNewerVersion
    )

    configure_file(cmake/${CMAKE_PROJECT_NAME}-config.cmake "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake" COPYONLY)
    configure_file(cmake/${CMAKE_PROJECT_NAME}-packages.cmake "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-packages.cmake")

    install(
        FILES
            "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config-version.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-packages.cmake"
        DESTINATION
            ${GAFRO_PREFIX_PACKAGE}
        COMPONENT
            Devel
    )

    # install(
    #     DIRECTORY
    #         "${CMAKE_CURRENT_BINARY_DIR}/assets/"
    #     DESTINATION
    #         "share/${CMAKE_PROJECT_NAME}/assets/"
    #     COMPONENT
    #         Devel
    # )
endif()
