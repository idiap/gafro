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
#include <gafro_control/AdmittanceController.hpp>

namespace gafro_control
{

    template <int dof, orwell::AdmittanceControllerType type>
    class DualArmMotorAdmittanceController : public DualArmAdmittanceController<dof, gafro::Sphere<double>, type>
    {
      public:
        DualArmMotorAdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

      private:
        void computeResiduals();
    };

    template <int dof>
    using DualArmMotorAdmittanceControllerTorque = DualArmMotorAdmittanceController<dof, orwell::AdmittanceControllerType::TORQUE>;

    template <int dof>
    using DualArmMotorAdmittanceControllerPosition = DualArmMotorAdmittanceController<dof, orwell::AdmittanceControllerType::POSITION>;

}  // namespace gafro_control

#include <gafro_control/DualArmMotorAdmittanceController.hxx>

REGISTER_CLASS(orwell::TorqueController<14>, gafro_control::DualArmMotorAdmittanceControllerTorque<7>, "dual_arm_motor_admittance_controller")
REGISTER_CLASS(orwell::PositionController<14>, gafro_control::DualArmMotorAdmittanceControllerPosition<7>, "dual_arm_motor_admittance_controller")