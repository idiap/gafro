# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES
            DualQuaternionAlgebra.hpp
            DualQuaternion.hpp
            DualQuaternion.hxx
            Point.hpp
            Line.hpp
            Plane.hpp
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro/algebra/dqa
        COMPONENT
            Devel
    )
endif()