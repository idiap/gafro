# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES 
            MultivectorGaussian.hpp
            MultivectorGaussian.hxx
            CovariancePropagation.hpp
            Point.hpp
            Point.hxx
        DESTINATION 
            ${GAFRO_PREFIX_INCLUDE}/gafro/probabilistic
        COMPONENT
            Devel
    )
endif()
