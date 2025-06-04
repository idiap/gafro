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

#include <gafro_control/AdmittanceController.hpp>
#include <gafro_control/RobotModel.hpp>

namespace gafro_control
{
    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    AdmittanceController<dof, Reference, type>::AdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name)
      : orwell::AdmittanceController<dof, Reference, type>(interface, name)
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

        residual_bivector_ = gafro::Motor<double>::Generator::Zero();
        residual_twist_ = gafro::Twist<double>::Zero();
        external_wrench_ = gafro::Wrench<double>::Zero();
        desired_wrench_ = gafro::Wrench<double>::Zero();
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    AdmittanceController<dof, Reference, type>::~AdmittanceController() = default;

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    typename orwell::RobotState<dof>::Vector AdmittanceController<dof, Reference, type>::computeDesiredJointAcceleration()
    {
        auto robot = std::dynamic_pointer_cast<gafro_control::RobotModel<dof>>(this->getRobotModel())->getManipulator();

        const Eigen::Vector<double, dof> &position = this->getRobotState().getPosition();
        const Eigen::Vector<double, dof> &velocity = this->getRobotState().getVelocity();

        computeResiduals();

        if (residual_bivector_.vector().norm() < 1e-5)
        {
            residual_bivector_ = gafro::Twist<double>::Zero();
        }

        Eigen::Matrix<double, 6, dof> jacobian = robot->getGeometricJacobian(position, getReferenceFrame()).embed();

        auto svd = jacobian.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

        gafro::Wrench<double> singularity = gafro::Wrench<double>::Zero();

        if (svd.singularValues()[dof-1]  < 5e-1)
        {
            singularity = gafro::Wrench<double>((1.0 - svd.singularValues()[dof-1] / 5e-1) * 0.1 * svd.matrixU().col(dof - 1));
        }

        gafro::Wrench<double> wrench = desired_wrench_ - external_wrench_ - stiffness_(residual_bivector_) - damping_(residual_twist_) - singularity;

        gafro::Twist<double> desired_ee_acceleration = inertia_(wrench);


        Eigen::Matrix<double, dof, 6> inverse_jacobian =
          (jacobian.transpose() * jacobian + 1e-5 * Eigen::Matrix<double, dof, dof>::Identity()).inverse() * jacobian.transpose();

        return inverse_jacobian * (desired_ee_acceleration.vector() -
                                   robot->getGeometricJacobianTimeDerivative(position, velocity, getReferenceFrame()).embed() * velocity);
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void AdmittanceController<dof, Reference, type>::setResidualBivector(const gafro::Motor<double>::Generator &residual_bivector)
    {
        residual_bivector_ = residual_bivector;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void AdmittanceController<dof, Reference, type>::setResidualTwist(const gafro::Twist<double> &residual_twist)
    {
        residual_twist_ = residual_twist;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void AdmittanceController<dof, Reference, type>::setDesiredWrench(const gafro::Wrench<double> &desired_wrench)
    {
        desired_wrench_ = desired_wrench;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void AdmittanceController<dof, Reference, type>::setExternalWrench(const gafro::Wrench<double> &external_wrench)
    {
        external_wrench_ = external_wrench;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void AdmittanceController<dof, Reference, type>::setInertia(const gafro::Inertia<double> &inertia)
    {
        inertia_ = inertia;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void AdmittanceController<dof, Reference, type>::setDamping(const gafro::Inertia<double> &damping)
    {
        damping_ = damping;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    void AdmittanceController<dof, Reference, type>::setStiffness(const gafro::Inertia<double> &stiffness)
    {
        stiffness_ = stiffness;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    const gafro::Inertia<double> &AdmittanceController<dof, Reference, type>::getInertia() const
    {
        return inertia_;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    const gafro::Inertia<double> &AdmittanceController<dof, Reference, type>::getDamping() const
    {
        return damping_;
    }

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    const gafro::Inertia<double> &AdmittanceController<dof, Reference, type>::getStiffness() const
    {
        return stiffness_;
    }

}  // namespace gafro_control