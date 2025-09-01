// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/gafro.hpp>
#include <gafro_control/AdmittanceController.hpp>

namespace gafro_control
{

    template <int dof, orwell::AdmittanceControllerType type>
    class MotorAdmittanceController : public AdmittanceController<dof, gafro::Motor<double>, type>
    {
      public:
        MotorAdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

      private:
        void computeResiduals();

        gafro::Motor<double> getReferenceFrame();
    };

    template <int dof>
    using MotorAdmittanceTorqueController = MotorAdmittanceController<dof, orwell::AdmittanceControllerType::TORQUE>;

    template <int dof>
    using MotorAdmittancePositionController = MotorAdmittanceController<dof, orwell::AdmittanceControllerType::POSITION>;

}  // namespace gafro_control

#include <gafro_control/MotorAdmittanceController.hxx>

REGISTER_CLASS(orwell::TorqueController<7>, gafro_control::MotorAdmittanceTorqueController<7>, "motor_admittance_controller")
REGISTER_CLASS(orwell::PositionController<7>, gafro_control::MotorAdmittancePositionController<7>, "motor_admittance_controller")