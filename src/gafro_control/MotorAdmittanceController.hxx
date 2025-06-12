// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro_control/MotorAdmittanceController.hpp>

namespace gafro_control
{

    template <int dof, orwell::AdmittanceControllerType type>
    MotorAdmittanceController<dof, type>::MotorAdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name)
      : AdmittanceController<dof, gafro::Motor<double>, type>(interface, name)
    {}

    template <int dof, orwell::AdmittanceControllerType type>
    void MotorAdmittanceController<dof, type>::computeResiduals()
    {
        auto robot = std::dynamic_pointer_cast<gafro_control::RobotModel<dof>>(this->getRobotModel())->getManipulator();

        const Eigen::Vector<double, dof> &q = this->getRobotState().getPosition();
        const Eigen::Vector<double, dof> &dq = this->getRobotState().getVelocity();

        const gafro::Motor<double> &target_motor = this->getReference();

        gafro::Motor<double> ee_motor = robot->getEEMotor(q);
        gafro::Motor<double> ee_motor_dt = gafro::Motor<double>::Parameters(robot->getEEAnalyticJacobian(q).embed() * dq);

        gafro::Motor<double> residual_motor = ee_motor.reverse() * target_motor;

        this->setResidualBivector(-residual_motor.log());
        this->setResidualTwist(-2.0 * gafro::Twist<double>(ee_motor.reverse() * ee_motor_dt));
    }

    template <int dof, orwell::AdmittanceControllerType type>
    gafro::Motor<double> MotorAdmittanceController<dof, type>::getReferenceFrame()
    {
        return std::dynamic_pointer_cast<gafro_control::RobotModel<dof>>(this->getRobotModel())->getEEMotor();
    }

}  // namespace gafro_control