# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES 
            Pointcloud.hpp
            Pointcloud.hxx
            EuclideanPointcloud.hpp
            EuclideanPointcloud.hxx
            ConformalPointcloud.hpp
            ConformalPointcloud.hxx
            Orbit.hpp
            Orbit.hxx
        DESTINATION 
            ${GAFRO_PREFIX_INCLUDE}/gafro/geometry
        COMPONENT
            Devel
    )
endif()
