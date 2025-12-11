// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

// #include <gafro/control/DualArmAdmittanceController.hpp>
#include <gafro/gafro.hpp>
//
#include <gafro/control/task_space/CooperativeDualTaskSpace.hpp>
#include <orwell/torque/AdmittanceController.hpp>

namespace gafro
{

    class DualArmMotorReference
    {
      public:
        DualArmMotorReference() = default;

        DualArmMotorReference(const gafro::Motor<double> &absolute_target, const gafro::Motor<double> &relative_target);

        void setAbsoluteTarget(const gafro::Motor<double> &absolute_target);

        void setRelativeTarget(const gafro::Motor<double> &relative_target);

        const gafro::Motor<double> &getAbsoluteTarget() const;

        const gafro::Motor<double> &getRelativeTarget() const;

      private:
        gafro::Motor<double> absolute_target_;
        gafro::Motor<double> relative_target_;
    };

    template <int dof>
    class DualArmMotorAdmittanceController : public orwell::torque::AdmittanceController<dof, 12, DualArmMotorReference>
    {
      public:
        DualArmMotorAdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);
    };

    // template <int dof>
    // using DualArmMotorAdmittanceControllerTorque = DualArmMotorAdmittanceController<dof, orwell::AdmittanceControllerType::TORQUE>;

    // template <int dof>
    // using DualArmMotorAdmittanceControllerPosition = DualArmMotorAdmittanceController<dof, orwell::AdmittanceControllerType::POSITION>;

}  // namespace gafro

#include <gafro/control/DualArmMotorAdmittanceController.hxx>

// REGISTER_CLASS(orwell::TorqueController<14>, gafro_control::DualArmMotorAdmittanceControllerTorque<7>, "dual_arm_motor_admittance_controller")
// REGISTER_CLASS(orwell::PositionController<14>, gafro_control::DualArmMotorAdmittanceControllerPosition<7>, "dual_arm_motor_admittance_controller")