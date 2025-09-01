if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    install(
        FILES
            RobotModel.hpp
            RobotModel.hxx

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

            RobotModelDualManipulator.hpp
            RobotModelDualManipulator.hxx

            DualArmAdmittanceController.hpp
            DualArmAdmittanceController.hxx
            DualArmMotorAdmittanceController.hpp
            DualArmMotorAdmittanceController.hxx
        DESTINATION
            ${GAFRO_PREFIX_INCLUDE}/gafro/control
        COMPONENT
            Devel
    )
endif()
