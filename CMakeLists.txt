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

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(GAFRO_PREFIX_PACKAGE share/${PROJECT_NAME}/cmake)
set(GAFRO_PREFIX_INCLUDE include/gafro)

find_package(Eigen3 3.4 REQUIRED)

add_subdirectory(src/${PROJECT_NAME})
add_subdirectory(src/${PROJECT_NAME}_control)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    add_subdirectory(tests)
endif()

include(CMakePackageConfigHelpers)

configure_package_config_file(
    cmake/gafro-config.cmake
    "${PROJECT_BINARY_DIR}/gafro-config.cmake"
    INSTALL_DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/"
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    VERSION ${${PROJECT_NAME}_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_file(cmake/${PROJECT_NAME}-packages.cmake "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-packages.cmake")

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-config-targets
    LIBRARY DESTINATION lib/${PROJECT_NAME}/
    ARCHIVE DESTINATION lib/${PROJECT_NAME}/
    RUNTIME DESTINATION lib/${PROJECT_NAME}/
    INCLUDES DESTINATION ${GAFRO_PREFIX_INCLUDE}/
)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-packages.cmake"
    DESTINATION
        ${GAFRO_PREFIX_PACKAGE}
    COMPONENT
        Devel
)

install(EXPORT ${PROJECT_NAME}-config-targets
    FILE ${PROJECT_NAME}-config-targets.cmake
    NAMESPACE gafro::
    DESTINATION ${GAFRO_PREFIX_PACKAGE}
)

export(EXPORT ${PROJECT_NAME}-config-targets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-targets.cmake"
    NAMESPACE gafro::
)

export(PACKAGE gafro)