// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/control/DualArmMotorObjective.hpp>
#include <gafro/control/task_space/CooperativeDualTaskSpace.hpp>

namespace gafro
{
    DualArmMotorReference::DualArmMotorReference(const gafro::Motor<double> &absolute_target, const gafro::Motor<double> &relative_target)
    {
        setAbsoluteTarget(absolute_target);
        setRelativeTarget(relative_target);
    }

    void DualArmMotorReference::setAbsoluteTarget(const gafro::Motor<double> &absolute_target)
    {
        absolute_target_ = absolute_target;
    }

    void DualArmMotorReference::setRelativeTarget(const gafro::Motor<double> &relative_target)
    {
        relative_target_ = relative_target;
    }

    const gafro::Motor<double> &DualArmMotorReference::getAbsoluteTarget() const
    {
        return absolute_target_;
    }

    const gafro::Motor<double> &DualArmMotorReference::getRelativeTarget() const
    {
        return relative_target_;
    }

    template <int dof>
    DualArmMotorObjective<dof>::DualArmMotorObjective() = default;

    template <int dof>
    typename DualArmMotorObjective<dof>::State DualArmMotorObjective<dof>::computeState(const orwell::TaskSpace<dof>::Ptr &task_space,
                                                                                        const orwell::RobotState<dof>     &robot_state) const
    {
        auto robot = std::dynamic_pointer_cast<gafro::CooperativeDualTaskSpace<double, dof>>(task_space);

        const Eigen::Vector<double, dof> &position = robot_state.getPosition();
        const Eigen::Vector<double, dof> &velocity = robot_state.getVelocity();

        const gafro::Motor<double> &absolute_target = this->getReference().getAbsoluteTarget();
        const gafro::Motor<double> &relative_target = this->getReference().getRelativeTarget();

        gafro::Motor<double> absolute_motor = robot->getAbsoluteMotor(position);
        gafro::Motor<double> relative_motor = robot->getRelativeMotor(position);

        gafro::Motor<double> absolute_motor_dt = gafro::Motor<double>::Parameters(robot->getAbsoluteAnalyticJacobian(position).embed() * velocity);
        gafro::Motor<double> relative_motor_dt = gafro::Motor<double>::Parameters(robot->getRelativeAnalyticJacobian(position).embed() * velocity);

        gafro::Motor<double> absolute_residual_motor = absolute_motor.reverse() * absolute_target;
        gafro::Motor<double> relative_residual_motor = relative_motor.reverse() * relative_target;

        Eigen::Vector<double, 12> error            = Eigen::Vector<double, 12>::Zero();
        Eigen::Vector<double, 12> error_derivative = Eigen::Vector<double, 12>::Zero();

        error.topRows(6)    = -absolute_residual_motor.log().vector();
        error.bottomRows(6) = -relative_residual_motor.log().vector();

        error_derivative.topRows(6)    = (-2.0 * gafro::Twist<double>(absolute_motor.reverse() * absolute_motor_dt)).vector();
        error_derivative.bottomRows(6) = (-2.0 * gafro::Twist<double>(relative_motor.reverse() * relative_motor_dt)).vector();

        Eigen::Matrix<double, 12, dof> jacobian = Eigen::Matrix<double, 12, dof>::Zero();

        jacobian.topRows(6)    = robot->getAbsoluteGeometricJacobian(position, absolute_motor).embed();
        jacobian.bottomRows(6) = robot->getRelativeGeometricJacobian(position, relative_motor).embed();

        Eigen::Matrix<double, 12, dof> jacobian_derivative = Eigen::Matrix<double, 12, dof>::Zero();

        jacobian_derivative.topRows(6)    = robot->getAbsoluteGeometricJacobianTimeDerivative(position, velocity, absolute_motor).embed();
        jacobian_derivative.bottomRows(6) = robot->getRelativeGeometricJacobianTimeDerivative(position, velocity, relative_motor).embed();

        State state;

        state.setError(error);
        state.setErrorDerivative(error_derivative);

        state.setJacobian(jacobian);
        state.setJacobianDerivative(jacobian_derivative);

        return state;
    }

}  // namespace gafro