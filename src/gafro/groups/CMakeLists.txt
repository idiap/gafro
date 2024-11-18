# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES
            Group.hpp
            Subgroup.hpp
            CoveringGroup.hpp
            LieGroup.hpp
            MatrixLieGroup.hpp
            MatrixLieGroup.hxx
            SpecialEuclideanGroup.hpp
            OrthogonalGroup.hpp
            SpecialOrthogonalGroup.hpp
            CliffordGroup.hpp
            PinGroup.hpp
            SpinGroup.hpp
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro/groups
        COMPONENT
            Devel
    )
endif()