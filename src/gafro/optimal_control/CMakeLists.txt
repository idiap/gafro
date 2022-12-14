target_sources(${PROJECT_NAME}
    PRIVATE 
        ${CMAKE_CURRENT_LIST_DIR}/optimal_control.cpp
)

install(
    FILES 
        SingleManipulatorTarget.hpp
        SingleManipulatorMotorCost.hpp
        # PointToPoint.hpp
        # PointToLine.hpp
        # PointToPlane.hpp
        # PointToSphere.hpp
        # # 
        # RotorGeneratorToLine.hpp
        # RotorGeneratorToMotor.hpp
        # RotorGeneratorToPlane.hpp
        # RotorGeneratorToPoint.hpp
        # RotorGeneratorToSphere.hpp
        # # 
        # ArmSingleIntegrator.hpp
        # ArmToLine.hpp
        # ArmToMotor.hpp
        # ArmToPoint.hpp
        # # 
        # DualArmMeetingSphere.hpp
        # DualArmMeetingLines.hpp
        # DualArmMeetingCircle.hpp
        # DualArmLineToPlane.hpp
        # DualArmPlaneNormal.hpp
    DESTINATION 
        ${GAFRO_PREFIX_INCLUDE}/${PROJECT_NAME}/optimal_control
    COMPONENT
        Devel
)