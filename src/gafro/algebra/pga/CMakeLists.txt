target_sources(${PROJECT_NAME}
    PRIVATE
)

# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES
            Motor.hpp
            Point.hpp
            Plane.hpp
            Line.hpp
            ProjectiveGeometricAlgebra.hpp
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro/algebra/pga
        COMPONENT
            Devel
    )
endif()