#pragma once

#include <gafro_control/AdmittanceController.hpp>
#include <gafro_control/RobotModel.hpp>

namespace gafro_control
{
    template <int dof, class Reference>
    AdmittanceController<dof, Reference>::AdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name)
      : orwell::AdmittanceController<dof, Reference>(interface, name)
    {
        auto loadTensor = [&](const std::string &tensor) {
            double mass;
            std::array<double, 6> values;
            interface->getConfigurations()->loadParameter(name + "/" + tensor + "/mass", &mass, false);
            interface->getConfigurations()->loadParameter<6>(name + "/" + tensor + "/tensor", &values);

            return gafro::Inertia<double>(mass, values[0], values[1], values[2], values[3], values[4], values[5]);
        };

        inertia_ = loadTensor("inertia");
        damping_ = loadTensor("damping");
        stiffness_ = loadTensor("stiffness");
    }

    template <int dof, class Reference>
    AdmittanceController<dof, Reference>::~AdmittanceController() = default;

    template <int dof, class Reference>
    typename orwell::RobotState<dof>::Vector AdmittanceController<dof, Reference>::computeDesiredJointAcceleration()
    {
        auto robot = std::dynamic_pointer_cast<gafro_control::RobotModel<dof>>(this->getRobotModel())->getManipulator();

        const Eigen::Vector<double, dof> &position = this->getRobotState().getPosition();
        const Eigen::Vector<double, dof> &velocity = this->getRobotState().getVelocity();

        computeResiduals();

        gafro::Twist<double> desired_ee_acceleration = inertia_(external_wrench_ - stiffness_(residual_bivector_) - damping_(residual_twist_));

        Eigen::Matrix<double, 6, dof> jacobian = robot->getGeometricJacobian(position, getReferenceFrame()).embed();

        Eigen::Matrix<double, dof, 6> inverse_jacobian =
          (jacobian.transpose() * jacobian + 1e-5 * Eigen::Matrix<double, dof, dof>::Identity()).inverse() * jacobian.transpose();

        return inverse_jacobian * (desired_ee_acceleration.vector() -
                                   robot->getGeometricJacobianTimeDerivative(position, velocity, getReferenceFrame()).embed() * velocity);
    }

    template <int dof, class Reference>
    void AdmittanceController<dof, Reference>::setResidualBivector(const gafro::Motor<double>::Generator &residual_bivector)
    {
        residual_bivector_ = residual_bivector;
    }

    template <int dof, class Reference>
    void AdmittanceController<dof, Reference>::setResidualTwist(const gafro::Twist<double> &residual_twist)
    {
        residual_twist_ = residual_twist;
    }

    template <int dof, class Reference>
    void AdmittanceController<dof, Reference>::setExternalWrench(const gafro::Wrench<double> &external_wrench)
    {
        external_wrench_ = external_wrench;
    }

}  // namespace gafro_control