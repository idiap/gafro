// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro_control/RobotModel.hpp>

namespace gafro_control
{

    template <int dof>
    RobotModel<dof>::RobotModel(const std::shared_ptr<gafro::Manipulator<double, dof>> &manipulator)
      : orwell::RobotModel<dof>(manipulator->getJointLimitsMin(), manipulator->getJointLimitsMax()), manipulator_(manipulator)
    {}

    template <int dof>
    RobotModel<dof>::~RobotModel() = default;

    template <int dof>
    void RobotModel<dof>::computeCartesianState(const orwell::RobotState<dof> &robot_state)
    {
        ee_motor_ = manipulator_->getEEMotor(robot_state.getPosition());

        jacobian_ = manipulator_->getEEFrameJacobian(robot_state.getPosition()).embed();

        ee_twist_ = gafro::Twist<double>(jacobian_ * robot_state.getVelocity());
    }

    template <int dof>
    const Eigen::Matrix<double, 6, dof> &RobotModel<dof>::getJacobian() const
    {
        return jacobian_;
    }

    template <int dof>
    typename RobotModel<dof>::Vector RobotModel<dof>::computeForwardDynamics(const Vector &position, const Vector &velocity,
                                                                             const Vector &acceleration, const double &gravity)
    {
        return manipulator_->getJointTorques(position, velocity, acceleration, gravity);
    }

    template <int dof>
    std::shared_ptr<gafro::Manipulator<double, dof>> RobotModel<dof>::getManipulator()
    {
        return manipulator_;
    }

    template <int dof>
    const gafro::Motor<double> &RobotModel<dof>::getEEMotor() const
    {
        return ee_motor_;
    }

    template <int dof>
    const gafro::Twist<double> &RobotModel<dof>::getEETwist() const
    {
        return ee_twist_;
    }

    template <int dof>
    template <template <class> class Manipulator, class... Args>
    typename RobotModel<dof>::Ptr RobotModel<dof>::create(Args... args)
    {
        return std::make_shared<RobotModel<dof>>(std::make_shared<Manipulator<double>>(std::forward<Args>(args)...));
    }

}  // namespace gafro_control