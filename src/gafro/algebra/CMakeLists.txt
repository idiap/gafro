target_sources(${PROJECT_NAME}
    PRIVATE
)

add_subdirectory(expressions)
add_subdirectory(util)

# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES
            Vector.hpp
            Vector.hxx

            DirectionVector.hpp
            DirectionVector.hxx

            TangentVector.hpp
            TangentVector.hxx

            Point.hpp
            Point.hxx

            PointPair.hpp
            PointPair.hxx

            Line.hpp
            Line.hxx

            Circle.hpp
            Circle.hxx

            Plane.hpp
            Plane.hxx

            Sphere.hpp
            Sphere.hxx

            Versor.hpp
            Versor.hxx

            Motor.hpp
            Motor.hxx

            MotorGenerator.hpp
            MotorGenerator.hxx

            Rotor.hpp
            Rotor.hxx

            RotorGenerator.hpp
            RotorGenerator.hxx

            Translator.hpp
            Translator.hxx

            TranslatorGenerator.hpp
            TranslatorGenerator.hxx

            Dilator.hpp
            Dilator.hxx

            AbstractMultivector.hpp
            AbstractMultivector.hxx
            Multivector.hpp
            Multivector.hxx

            Blades.hpp

            Operators.hxx

            MultivectorMatrix.hpp
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro/algebra
        COMPONENT
            Devel
    )
endif()