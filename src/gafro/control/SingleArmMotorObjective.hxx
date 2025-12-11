// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/control/SingleArmMotorObjective.hpp>
#include <gafro/control/task_space/SingleArmTaskSpace.hpp>

namespace gafro_control
{

    template <int dof>
    SingleArmMotorObjective<dof>::SingleArmMotorObjective() = default;

    template <int dof>
    typename SingleArmMotorObjective<dof>::State SingleArmMotorObjective<dof>::computeState(const orwell::TaskSpace<dof>::Ptr &task_space,
                                                                                            const orwell::RobotState<dof>     &robot_state) const
    {
        auto robot = std::dynamic_pointer_cast<gafro::SingleArmTaskSpace<double, dof>>(task_space)->getSystem();

        const Eigen::Vector<double, dof> &q  = robot_state.getPosition();
        const Eigen::Vector<double, dof> &dq = robot_state.getVelocity();

        const gafro::Motor<double> &target_motor = this->getReference();

        gafro::Motor<double> ee_motor    = robot->getEEMotor(q);
        gafro::Motor<double> ee_motor_dt = gafro::Motor<double>::Parameters(robot->getEEAnalyticJacobian(q).embed() * dq);

        gafro::Motor<double> residual_motor = ee_motor.reverse() * target_motor;

        State state;

        state.setError(-residual_motor.log().vector());
        state.setErrorDerivative(-2.0 * gafro::Twist<double>(ee_motor.reverse() * ee_motor_dt).vector());

        state.setJacobian(robot->getGeometricJacobian(q, ee_motor).embed());
        state.setJacobianDerivative(robot->getGeometricJacobianTimeDerivative(q, dq, ee_motor).embed());

        return state;
    }

}  // namespace gafro_control
