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

#include <gafro/gafro.hpp>
#include <gafro/robot/DualManipulator.hpp>
//
#include <orwell/MultiRobotController.hpp>
#include <orwell/torque/TorqueController.hpp>

namespace gafro_control
{

    template <int dof>
    class DualArmAbsoluteAdmittanceController : public orwell::MultiRobotController<2, dof, orwell::TorqueController>
    {
      public:
        DualArmAbsoluteAdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name,
                                            const std::shared_ptr<gafro::DualManipulator<double, 2 * dof>> &manipulator);

        virtual ~DualArmAbsoluteAdmittanceController();

        void setAbsoluteTarget(const gafro::Motor<double> &absolute_target);

        Eigen::Matrix<double, 2 * dof, 1> computeCommand(const orwell::RobotState<2 * dof> &state);

      private:
        gafro::Inertia<double> inertia_;
        gafro::Inertia<double> damping_;
        gafro::Inertia<double> stiffness_;

        gafro::Motor<double> absolute_target_;
    };

}  // namespace gafro_control

#include <gafro_control/DualArmAbsoluteAdmittanceController.hxx>