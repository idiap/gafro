// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/control/LineImpedanceController.hpp>
#include <gafro/control/RobotModel.hpp>

namespace gafro_control
{

    template <int dof>
    LineImpedanceController<dof>::LineImpedanceController(const sackmesser::Interface::Ptr &interface, const std::string &name)
      : orwell::CartesianImpedanceController<dof, gafro::Line<double>>(interface, name)
    {}

    template <int dof>
    LineImpedanceController<dof>::~LineImpedanceController() = default;

    template <int dof>
    void LineImpedanceController<dof>::computeStateError()
    {
        auto robot_model = std::dynamic_pointer_cast<gafro_control::RobotModel<dof>>(this->getRobotModel());

        gafro::Motor<double> ee_motor = robot_model->getEEMotor().reverse();

        gafro::Line<double> target_line = ee_motor.apply(this->getReference());

        gafro::Motor<double> residual_motor = gafro::Line<double>::Z().getMotor(target_line);

        this->setPositionError(residual_motor.log().vector());
        this->setVelocityError(-robot_model->getEETwist().vector());
        // this->setAccelerationError(gafro::Line<double>::Z().vector());
    }

}  // namespace gafro_control