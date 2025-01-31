@PACKAGE_INIT@

get_filename_component(gafro_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

if(NOT TARGET gafro::gafro)
    include("${gafro_CMAKE_DIR}/gafro-config-targets.cmake")
    include("${gafro_CMAKE_DIR}/gafro-packages.cmake")
endif()

get_target_property(gafro_INCLUDE_DIRS gafro::gafro INTERFACE_INCLUDE_DIRECTORIES)

check_required_components(gafro)