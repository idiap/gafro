target_sources(${PROJECT_NAME}
    PRIVATE 
        ${CMAKE_CURRENT_LIST_DIR}/physics.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Inertia.hpp
)

install(
    FILES 
        Inertia.hpp
        Inertia.hxx
    DESTINATION 
        ${GAFRO_PREFIX_INCLUDE}/gafro/physics
    COMPONENT
        Devel
)