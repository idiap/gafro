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

#include <gafro_control/DualArmAdmittanceController.hpp>
#include <gafro_control/RobotModelDualManipulator.hpp>

namespace gafro_control
{
    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    DualArmAdmittanceController<dof, Reference, type>::DualArmAdmittanceController(const sackmesser::Interface::Ptr &interface,
                                                                                   const std::string &name)
      : orwell::AdmittanceController<2 * dof, Reference, type>(interface, name)
    {
        auto loadTensor = [&](const std::string &tensor) {
            double mass;
            std::array<double, 6> values;
            interface->getConfigurations()->loadParameter(name + "/" + tensor + "/mass", &mass, false);
            interface->getConfigurations()->loadParameter<6>(name + "/" + tensor + "/tensor", &values);

            return gafro::Inertia<double>(mass, values[0], values[1], values[2], values[3], values[4], values[5]);
        };

        absolute_inertia_ = loadTensor("absolute_inertia");
        absolute_damping_ = loadTensor("absolute_damping");
        absolute_stiffness_ = loadTensor("absolute_stiffness");

        relative_inertia_ = loadTensor("relative_inertia");
        relative_damping_ = loadTensor("relative_damping");
        relative_stiffness_ = loadTensor("relative_stiffness");

        desired_absolute_wrench_ = gafro::Wrench<double>::Zero();
        external_absolute_wrench_ = gafro::Wrench<double>::Zero();

        desired_relative_wrench_ = gafro::Wrench<double>::Zero();
        external_relative_wrench_ = gafro::Wrench<double>::Zero();

        absolute_residual_ = gafro::Twist<double>::Zero();
        absolute_residual_dt_ = gafro::Twist<double>::Zero();
        relative_residual_ = gafro::Twist<double>::Zero();
        relative_residual_dt_ = gafro::Twist<double>::Zero();
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    DualArmAdmittanceController<dof, Reference, type>::~DualArmAdmittanceController() = default;

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    typename orwell::RobotState<2 * dof>::Vector DualArmAdmittanceController<dof, Reference, type>::computeDesiredJointAcceleration()
    {
        auto dual_robot = std::dynamic_pointer_cast<gafro_control::RobotModelDualManipulator<2 * dof>>(this->getRobotModel())->getManipulator();

        Eigen::Vector<double, 2 * dof> position = this->getRobotState().getPosition();
        Eigen::Vector<double, 2 * dof> velocity = this->getRobotState().getVelocity();

        gafro::Motor<double> absolute_motor = dual_robot->getAbsoluteMotor(position);
        gafro::Motor<double> relative_motor = dual_robot->getRelativeMotor(position);

        computeResiduals();

        Eigen::Matrix<double, 12, 2 * dof> jacobian;

        jacobian.topRows(6) = dual_robot->getAbsoluteGeometricJacobian(position, absolute_motor).embed();
        jacobian.bottomRows(6) = dual_robot->getRelativeGeometricJacobian(position, relative_motor).embed();

        Eigen::Matrix<double, 12, 2 * dof> jacobian_dt;

        jacobian_dt.topRows(6) = dual_robot->getAbsoluteGeometricJacobianTimeDerivative(position, velocity, absolute_motor).embed();
        jacobian_dt.bottomRows(6) = dual_robot->getRelativeGeometricJacobianTimeDerivative(position, velocity, absolute_motor).embed();

        Eigen::Matrix<double, 2 * dof, 12> inverse_jacobian =
          (jacobian.transpose() * jacobian + 1e-5 * Eigen::Matrix<double, 2 * dof, 2 * dof>::Identity()).inverse() * jacobian.transpose();

        Eigen::Vector<double, 12> desired_ee_acceleeration;

        desired_ee_acceleeration.topRows(6) = computeDesiredAbsoluteEEAcceleration().vector();
        desired_ee_acceleeration.bottomRows(6) = computeDesiredRelativeEEAcceleration().vector();

        return inverse_jacobian * (desired_ee_acceleeration - jacobian_dt * velocity);
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void DualArmAdmittanceController<dof, Reference, type>::setDesiredWrench(const gafro::Wrench<double> &w1, const gafro::Wrench<double> &w2)
    {}

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void DualArmAdmittanceController<dof, Reference, type>::setExternalWrench(const gafro::Wrench<double> &w1, const gafro::Wrench<double> &w2)
    {}

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void DualArmAdmittanceController<dof, Reference, type>::setAbsoluteResidual(const gafro::Motor<double>::Generator &absolute_residual)
    {
        absolute_residual_ = absolute_residual;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void DualArmAdmittanceController<dof, Reference, type>::setAbsoluteResidualDt(const gafro::Twist<double> &absolute_residual_dt)
    {
        absolute_residual_dt_ = absolute_residual_dt;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void DualArmAdmittanceController<dof, Reference, type>::setRelativeResidual(const gafro::Motor<double>::Generator &relative_residual)
    {
        relative_residual_ = relative_residual;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void DualArmAdmittanceController<dof, Reference, type>::setRelativeResidualDt(const gafro::Twist<double> &relative_residual_dt)
    {
        relative_residual_dt_ = relative_residual_dt;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    gafro::Twist<double> DualArmAdmittanceController<dof, Reference, type>::computeDesiredAbsoluteEEAcceleration() const
    {
        return absolute_inertia_(desired_absolute_wrench_ - external_absolute_wrench_ - absolute_stiffness_(absolute_residual_) -
                                 absolute_damping_(absolute_residual_dt_));
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    gafro::Twist<double> DualArmAdmittanceController<dof, Reference, type>::computeDesiredRelativeEEAcceleration() const
    {
        return relative_inertia_(desired_relative_wrench_ - external_relative_wrench_ - relative_stiffness_(relative_residual_) -
                                 relative_damping_(relative_residual_dt_));
    }

}  // namespace gafro_control