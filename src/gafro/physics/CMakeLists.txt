target_sources(${PROJECT_NAME}
    PRIVATE 
        ${CMAKE_CURRENT_LIST_DIR}/physics.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Inertia.hpp
)

# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES 
            Inertia.hpp
            Inertia.hxx
            Twist.hpp
            Twist.hxx
            Wrench.hpp
            Wrench.hxx
        DESTINATION 
            ${GAFRO_PREFIX_INCLUDE}/gafro/physics
        COMPONENT
            Devel
    )
endif()
