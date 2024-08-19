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

#include <gafro_control/PlaneAdmittanceController.hpp>

namespace gafro_control
{

    template <int dof>
    PlaneAdmittanceController<dof>::PlaneAdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name)
      : AdmittanceController<dof, gafro::Plane<double>>(interface, name)
    {}

    template <int dof>
    void PlaneAdmittanceController<dof>::computeResiduals()
    {
        auto robot_model = std::dynamic_pointer_cast<gafro_control::RobotModel<dof>>(this->getRobotModel());

        gafro::Motor<double> ee_motor = robot_model->getEEMotor().reverse();

        gafro::Plane<double> target_plane = ee_motor.apply(this->getReference());

        gafro::Motor<double> residual_motor = target_plane.computeMotor(gafro::Plane<double>::XZ());

        this->setResidualBivector(residual_motor.log());
        this->setResidualTwist(robot_model->getEETwist());

        reference_frame_ = robot_model->getEEMotor();
    }

    template <int dof>
    gafro::Motor<double> PlaneAdmittanceController<dof>::getReferenceFrame()
    {
        return reference_frame_;
    }

}  // namespace gafro_control