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