if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES
            TaskSpace.hpp
            TaskSpace.hxx
            SingleArmTaskSpace.hpp
            SingleArmTaskSpace.hxx
            CooperativeTaskSpace.hpp
            CooperativeTaskSpace.hxx
            CooperativeDualTaskSpace.hpp
            CooperativeDualTaskSpace.hxx
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro/control/task_space
        COMPONENT
            Devel
    )
endif()
