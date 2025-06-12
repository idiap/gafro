// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro_control/RobotModelDualManipulator.hpp>

namespace gafro_control
{

    template <int dof>
    RobotModelDualManipulator<dof>::RobotModelDualManipulator(const std::shared_ptr<gafro::DualManipulator<double, dof>> &manipulator)
      : orwell::RobotModel<dof>(manipulator->getJointLimitsMin(), manipulator->getJointLimitsMax()), manipulator_(manipulator)
    {}

    template <int dof>
    RobotModelDualManipulator<dof>::RobotModelDualManipulator(const gafro::Manipulator<double, dof / 2> &manipulator,
                                                              const gafro::Motor<double> &first_base_motor,
                                                              const gafro::Motor<double> &second_base_motor)
      : RobotModelDualManipulator(std::make_shared<gafro::DualManipulator<double, dof>>(manipulator, first_base_motor, second_base_motor))
    {}

    template <int dof>
    RobotModelDualManipulator<dof>::~RobotModelDualManipulator() = default;

    template <int dof>
    void RobotModelDualManipulator<dof>::computeCartesianState(const orwell::RobotState<dof> &robot_state)
    {}

    template <int dof>
    const Eigen::Matrix<double, 6, dof> &RobotModelDualManipulator<dof>::getJacobian() const
    {
        return jacobian_;
    }

    template <int dof>
    typename RobotModelDualManipulator<dof>::Vector RobotModelDualManipulator<dof>::computeForwardDynamics(const Vector &position,
                                                                                                           const Vector &velocity,
                                                                                                           const Vector &acceleration,
                                                                                                           const double &gravity)
    {
        return RobotModelDualManipulator<dof>::Vector::Zero();
        // return manipulator_->getJointTorques(position, velocity, acceleration, gravity);
    }

    template <int dof>
    std::shared_ptr<gafro::DualManipulator<double, dof>> RobotModelDualManipulator<dof>::getManipulator()
    {
        return manipulator_;
    }

    template <int dof>
    const gafro::Motor<double> &RobotModelDualManipulator<dof>::getEEMotor() const
    {
        return ee_motor_;
    }

    template <int dof>
    const gafro::Twist<double> &RobotModelDualManipulator<dof>::getEETwist() const
    {
        return ee_twist_;
    }

    template <int dof>
    template <class... Args>
    typename RobotModelDualManipulator<dof>::Ptr RobotModelDualManipulator<dof>::create(Args... args)
    {
        return std::make_shared<RobotModelDualManipulator<dof>>(std::make_shared<gafro::DualManipulator<double, dof>>(std::forward<Args>(args)...));
    }

}  // namespace gafro_control