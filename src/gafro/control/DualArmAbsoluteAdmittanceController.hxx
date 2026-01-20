// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <sackmesser/Callbacks.hpp>
//
#include <gafro/control/DualArmAbsoluteAdmittanceController.hpp>
#include <gafro/control/RobotModelDualManipulator.hpp>

namespace gafro_control
{

    template <int dof>
    DualArmAbsoluteAdmittanceController<dof>::DualArmAbsoluteAdmittanceController(
      const sackmesser::Interface::Ptr                               &interface,
      const std::string                                              &name,
      const std::shared_ptr<gafro::DualManipulator<double, 2 * dof>> &manipulator)
    // : orwell::MultiRobotController<2, dof, orwell::TorqueController>(
    //     interface, name, std::make_shared<gafro_control::RobotModelDualManipulator<2 * dof>>(manipulator))
    {
        auto loadTensor = [&](const std::string &tensor) {
            double                mass;
            std::array<double, 6> values;
            interface->getConfigurations()->loadParameter(name + "/" + tensor + "/mass", &mass, false);
            interface->getConfigurations()->loadParameter<6>(name + "/" + tensor + "/tensor", &values);

            return gafro::Inertia<double>(mass, values[0], values[1], values[2], values[3], values[4], values[5]);
        };

        inertia_   = loadTensor("inertia");
        damping_   = loadTensor("damping");
        stiffness_ = loadTensor("stiffness");
    }

    template <int dof>
    DualArmAbsoluteAdmittanceController<dof>::~DualArmAbsoluteAdmittanceController() = default;

    template <int dof>
    void DualArmAbsoluteAdmittanceController<dof>::setAbsoluteTarget(const gafro::Motor<double> &absolute_target)
    {
        absolute_target_ = absolute_target;
    }

    template <int dof>
    Eigen::Matrix<double, 2 * dof, 1> DualArmAbsoluteAdmittanceController<dof>::computeCommand(const orwell::RobotState<2 * dof> &state)
    {
        // auto dual_robot = std::dynamic_pointer_cast<gafro_control::RobotModelDualManipulator<2 * dof>>(this->getRobotModel())->getManipulator();

        // Eigen::Vector<double, 2 * dof> q  = state.getPosition();
        // Eigen::Vector<double, 2 * dof> dq = state.getVelocity();

        // gafro::Motor<double> absolute_motor    = dual_robot->getAbsoluteMotor(q);
        // gafro::Motor<double> absolute_motor_dt = gafro::Motor<double>::Parameters(dual_robot->getAbsoluteAnalyticJacobian(q).embed() * dq);

        // gafro::Motor<double> absolute_residual_motor = absolute_target_.reverse() * absolute_motor;

        // gafro::Twist<double> absolute_residual_bivector = absolute_residual_motor.log();
        // gafro::Twist<double> absolute_residual_bivector_dt =
        //   -2.0 * gafro::Twist<double>(absolute_residual_motor.reverse() * absolute_target_.reverse() * absolute_motor_dt);

        // Eigen::Matrix<double, 6, 2 * dof> absolute_jacobian = dual_robot->getAbsoluteGeometricJacobian(q, absolute_motor).embed();
        // gafro::Twist<double>              absolute_twist    = gafro::Twist<double>(absolute_jacobian * dq);

        // gafro::Twist<double> desired_ee_acceleration =
        //   inertia_(gafro::Wrench<double>(-stiffness_(absolute_residual_bivector) - damping_(absolute_residual_bivector_dt)));

        // Eigen::Matrix<double, 2 * dof, 6> inverse_jacobian =
        //   (absolute_jacobian.transpose() * absolute_jacobian + 1e-7 * Eigen::Matrix<double, 2 * dof, 2 * dof>::Identity()).inverse() *
        //   absolute_jacobian.transpose();

        // Eigen::Matrix<double, 2 * dof, 1> acceleration =
        //   inverse_jacobian *
        //   (desired_ee_acceleration.vector() - dual_robot->getAbsoluteGeometricJacobianTimeDerivative(q, dq, absolute_motor).embed() * dq);

        // return dual_robot->getJointTorques(q, dq, acceleration, 0.0);
    }

}  // namespace gafro_control
