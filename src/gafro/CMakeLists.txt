configure_file(${CMAKE_CURRENT_LIST_DIR}/gafro_package_config.h.in "${CMAKE_CURRENT_BINARY_DIR}/gafro_package_config.hpp")

add_library(${PROJECT_NAME})


include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-march=native" MARCH_NATIVE_SUPPORTED)
check_cxx_compiler_flag("-msse4" MSSE2_SUPPORTED)

if (MARCH_NATIVE_SUPPORTED)
    target_compile_options(${PROJECT_NAME} PUBLIC "-march=native")
endif()

if (MSSE2_SUPPORTED)
    target_compile_options(${PROJECT_NAME} PUBLIC "-msse4")
endif()


target_compile_options(${PROJECT_NAME} PUBLIC "-O3")
target_compile_options(${PROJECT_NAME} PUBLIC "-Wall")
target_compile_options(${PROJECT_NAME} PUBLIC "-Wpedantic")
# target_compile_options(${PROJECT_NAME} PUBLIC "-Werror")

add_subdirectory(algebra)
add_subdirectory(physics)
add_subdirectory(robot)
add_subdirectory(optimization)

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${${PROJECT_NAME}_VERSION}
  SOVERSION 1
  INTERFACE_GAFRO_MAJOR_VERSION 1
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${${PROJECT_NAME}_PREFIX_INCLUDE}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>/..
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>

        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
        $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIRS}>

    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>/..
)

target_include_directories(${PROJECT_NAME}
    SYSTEM PRIVATE
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${EIGEN3_LIBRARIES}>
        $<INSTALL_INTERFACE:${EIGEN3_LIBRARIES}>
    PRIVATE
)

# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(TARGETS ${PROJECT_NAME} EXPORT ${CMAKE_PROJECT_NAME}-config-targets
        LIBRARY DESTINATION lib/${PROJECT_NAME}/
        ARCHIVE DESTINATION lib/${PROJECT_NAME}/
        RUNTIME DESTINATION lib/${PROJECT_NAME}/
        INCLUDES DESTINATION ${GAFRO_PREFIX_INCLUDE}/
    )

    install(
        FILES
            "${CMAKE_CURRENT_BINARY_DIR}/gafro_package_config.hpp"
            gafro.hpp
            algebra.hpp
            robot.hpp
            physics.hpp
            optimization.hpp
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro
        COMPONENT
            Devel
    )
endif()
