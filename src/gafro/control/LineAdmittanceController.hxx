// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/control/LineAdmittanceController.hpp>

namespace gafro_control
{

    template <int dof, orwell::AdmittanceControllerType type>
    LineAdmittanceController<dof, type>::LineAdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name)
      : AdmittanceController<dof, gafro::Line<double>, type>(interface, name)
    {}

    template <int dof, orwell::AdmittanceControllerType type>
    void LineAdmittanceController<dof, type>::computeResiduals()
    {
        auto robot_model = std::dynamic_pointer_cast<gafro_control::RobotModel<dof>>(this->getRobotModel());

        gafro::Motor<double> ee_motor_reverse = robot_model->getEEMotor().reverse();

        gafro::Motor<double> residual_motor = reference_line_.getMotor(ee_motor_reverse.apply(this->getReference()));

        this->setResidualBivector(-residual_motor.log());
        this->setResidualTwist(robot_model->getEETwist());

        reference_frame_ = robot_model->getEEMotor();
    }

    template <int dof, orwell::AdmittanceControllerType type>
    void LineAdmittanceController<dof, type>::setReferenceLine(const gafro::Line<double> &reference_line)
    {
        reference_line_ = reference_line;
    }

    template <int dof, orwell::AdmittanceControllerType type>
    gafro::Motor<double> LineAdmittanceController<dof, type>::getReferenceFrame()
    {
        return reference_frame_;
    }

}  // namespace gafro_control