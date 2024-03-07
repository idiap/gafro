target_sources(${PROJECT_NAME}
    PRIVATE 
        ${CMAKE_CURRENT_LIST_DIR}/algorithm.cpp
)

# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES 
            InverseDynamics.hpp
            InverseDynamics.hxx
        DESTINATION 
            ${GAFRO_PREFIX_INCLUDE}/gafro/robot/algorithm
        COMPONENT
            Devel
    )
endif()