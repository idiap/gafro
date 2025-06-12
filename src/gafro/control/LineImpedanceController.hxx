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

#include <gafro_control/LineImpedanceController.hpp>
#include <gafro_control/RobotModel.hpp>

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