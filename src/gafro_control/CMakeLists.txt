find_package(orwell QUIET)

if (${orwell_FOUND})

    add_library(gafro_control INTERFACE
        RobotModel.hpp
        LineImpedanceController.hpp
        AdmittanceController.hpp
        MotorAdmittanceController.hpp
    )

    target_include_directories(gafro_control
        INTERFACE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>/..
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>/..
    )
    target_link_libraries(gafro_control
        INTERFACE
            gafro
            orwell::orwell
    )

    if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
        install(TARGETS gafro_control EXPORT ${CMAKE_PROJECT_NAME}-config-targets
            LIBRARY DESTINATION lib/${PROJECT_NAME}/
            ARCHIVE DESTINATION lib/${PROJECT_NAME}/
            RUNTIME DESTINATION lib/${PROJECT_NAME}/
            INCLUDES DESTINATION ${GAFRO_PREFIX_INCLUDE}/
        )

        install(
            FILES
                RobotModel.hpp
                RobotModel.hxx
                LineImpedanceController.hpp
                LineImpedanceController.hxx
                MotorAdmittanceController.hpp
                MotorAdmittanceController.hxx
                AdmittanceController.hpp
                AdmittanceController.hxx
            DESTINATION
                ${GAFRO_PREFIX_INCLUDE}/gafro_control
            COMPONENT
                Devel
        )
    endif()
    
endif()