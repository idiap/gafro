add_subdirectory(task_space)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES
            LineImpedanceController.hpp
            LineImpedanceController.hxx
            AdmittanceController.hpp
            AdmittanceController.hxx
            MotorAdmittanceController.hpp
            MotorAdmittanceController.hxx
            LineAdmittanceController.hpp
            LineAdmittanceController.hxx
            PlaneAdmittanceController.hpp
            PlaneAdmittanceController.hxx

            DualArmAdmittanceController.hpp
            DualArmAdmittanceController.hxx
            DualArmMotorAdmittanceController.hpp
            DualArmMotorAdmittanceController.hxx

            SingleArmMotorObjective.hpp
            SingleArmMotorObjective.hxx
            DualArmMotorObjective.hpp
            DualArmMotorObjective.hxx
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro/control
        COMPONENT
            Devel
    )
endif()
