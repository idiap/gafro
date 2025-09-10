# Only do that when using the gafro source code directly, not from a submodule
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES
            Joint.hpp
            Joint.hxx
            FixedJoint.hpp
            FixedJoint.hxx
            RevoluteJoint.hpp
            RevoluteJoint.hxx
            PrismaticJoint.hpp
            PrismaticJoint.hxx
            FreeJoint.hpp
            FreeJoint.hxx

            Link.hpp
            Link.hxx
            Body.hpp
            Body.hxx

            Manipulator.hpp
            Manipulator.hxx

            DualManipulator.hpp
            DualManipulator.hxx

            Hand.hpp
            Hand.hxx

            Quadruped.hpp
            Quadruped.hxx

            Humanoid.hpp
            Humanoid.hxx

            System.hpp
            System.hxx

            KinematicChain.hpp
            KinematicChain.hxx

            TaskSpace.hpp
            TaskSpace.hxx
            CooperativeTaskSpace.hpp
            CooperativeTaskSpace.hxx
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro/robot
        COMPONENT
            Devel
    )
endif()

add_subdirectory(algorithm)