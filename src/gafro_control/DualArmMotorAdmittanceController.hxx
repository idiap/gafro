/*
    Copyright (c) 2022 Idiap Research Institute, http://www.idiap.ch/
    Written by Tobias Löw <https://tobiloew.ch>

    This file is part of gafro.

    gafro is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3 as
    published by the Free Software Foundation.

    gafro is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with gafro. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <gafro_control/DualArmMotorAdmittanceController.hpp>

namespace gafro_control
{

    template <int dof, orwell::AdmittanceControllerType type>
    DualArmMotorAdmittanceController<dof, type>::DualArmMotorAdmittanceController(const sackmesser::Interface::Ptr &interface,
                                                                                  const std::string &name)
      : DualArmAdmittanceController<dof, DualArmMotorReference, type>(interface, name)
    {}

    template <int dof, orwell::AdmittanceControllerType type>
    void DualArmMotorAdmittanceController<dof, type>::computeResiduals()
    {
        auto dual_robot = std::dynamic_pointer_cast<gafro_control::RobotModelDualManipulator<2 * dof>>(this->getRobotModel())->getManipulator();

        const Eigen::Vector<double, 2 * dof> &position = this->getRobotState().getPosition();
        const Eigen::Vector<double, 2 * dof> &velocity = this->getRobotState().getVelocity();

        const gafro::Motor<double> &absolute_target = this->getReference().getAbsoluteTarget();
        const gafro::Motor<double> &relative_target = this->getReference().getRelativeTarget();

        gafro::Motor<double> absolute_motor = dual_robot->getAbsoluteMotor(position);
        gafro::Motor<double> relative_motor = dual_robot->getRelativeMotor(position);

        gafro::Motor<double> absolute_motor_dt =
          gafro::Motor<double>::Parameters(dual_robot->getAbsoluteAnalyticJacobian(position).embed() * velocity);
        gafro::Motor<double> relative_motor_dt =
          gafro::Motor<double>::Parameters(dual_robot->getRelativeAnalyticJacobian(position).embed() * velocity);

        gafro::Motor<double> absolute_residual_motor = absolute_motor.reverse() * absolute_target;
        gafro::Motor<double> relative_residual_motor = relative_motor.reverse() * relative_target;

        this->setAbsoluteResidual(-absolute_residual_motor.log());
        this->setRelativeResidual(-relative_residual_motor.log());

        this->setAbsoluteResidualDt(-2.0 * gafro::Twist<double>(absolute_motor.reverse() * absolute_motor_dt));
        this->setRelativeResidualDt(-2.0 * gafro::Twist<double>(relative_motor.reverse() * relative_motor_dt));
    }

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

}  // namespace gafro_control