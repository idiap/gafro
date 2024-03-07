target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/robot.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Joint.hpp
        ${CMAKE_CURRENT_LIST_DIR}/PrismaticJoint.hpp
        ${CMAKE_CURRENT_LIST_DIR}/Link.hpp
        ${CMAKE_CURRENT_LIST_DIR}/Manipulator.hpp
)

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

            Link.hpp
            Link.hxx
            Body.hpp
            Body.hxx

            Manipulator.hpp
            Manipulator.hxx

            # FrankaEmikaRobot.hpp
            # UR5.hpp

            System.hpp
            System.hxx

            KinematicChain.hpp
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro/robot
        COMPONENT
            Devel
    )
endif()

add_subdirectory(algorithm)