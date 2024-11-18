configure_file(${CMAKE_CURRENT_LIST_DIR}/gafro_package_config.h.in "${CMAKE_CURRENT_BINARY_DIR}/gafro_package_config.hpp")

add_library(gafro INTERFACE)
add_library(gafro::gafro ALIAS gafro)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-march=native" MARCH_NATIVE_SUPPORTED)
check_cxx_compiler_flag("-msse4" MSSE2_SUPPORTED)

if (MARCH_NATIVE_SUPPORTED)
    target_compile_options(${PROJECT_NAME} INTERFACE "-march=native")
endif()

if (MSSE2_SUPPORTED)
    target_compile_options(${PROJECT_NAME} INTERFACE "-msse4")
endif()


target_compile_options(${PROJECT_NAME} INTERFACE "-O3")
target_compile_options(${PROJECT_NAME} INTERFACE "-Wall")
target_compile_options(${PROJECT_NAME} INTERFACE "-Wpedantic")
# target_compile_options(${PROJECT_NAME} INTERFACE "-Werror")
target_compile_options(${PROJECT_NAME} INTERFACE "-fno-builtin")

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)

add_subdirectory(algebra)
add_subdirectory(physics)
add_subdirectory(robot)
add_subdirectory(optimization)
add_subdirectory(groups)

# include(GenerateExportHeader)
# generate_export_header(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES
  INTERFACE_VERSION ${${PROJECT_NAME}_VERSION}
)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<INSTALL_INTERFACE:${${PROJECT_NAME}_PREFIX_INCLUDE}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>/..
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>

        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
        $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIRS}>

        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>/..
)

target_include_directories(${PROJECT_NAME}
    SYSTEM PRIVATE
)

target_link_libraries(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${EIGEN3_LIBRARIES}>
        $<INSTALL_INTERFACE:${EIGEN3_LIBRARIES}>
)

# Only do that when using the gafro source code directly, not from a submodule
# if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    install(
        FILES
            "${CMAKE_CURRENT_BINARY_DIR}/gafro_package_config.hpp"
            gafro.hpp
            algebra.hpp
            robot.hpp
            physics.hpp
            optimization.hpp
            groups.hpp
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro
        COMPONENT
            Devel
    )
# endif()
