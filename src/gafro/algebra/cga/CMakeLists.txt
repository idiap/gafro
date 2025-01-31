# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES
            ConformalGeometricAlgebra.hpp
            
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

            MotorLogarithm.hpp
            MotorExponential.hpp

            Rotor.hpp
            Rotor.hxx

            RotorGenerator.hpp
            RotorGenerator.hxx

            RotorExponential.hpp

            Translator.hpp
            Translator.hxx

            TranslatorGenerator.hpp
            TranslatorGenerator.hxx

            Dilator.hpp
            Dilator.hxx

            Transversion.hpp
            Transversion.hxx

            SimilarityTransformation.hpp
            SimilarityTransformation.hxx

            ConformalTransformation.hpp
            ConformalTransformation.hxx

            Blades.hpp
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro/algebra/cga
        COMPONENT
            Devel
    )
endif()