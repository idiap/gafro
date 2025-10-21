// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

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

#include <gafro/control/DualArmAbsoluteAdmittanceController.hxx>