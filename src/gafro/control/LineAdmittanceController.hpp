// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/gafro.hpp>
#include <gafro/control/AdmittanceController.hpp>

namespace gafro_control
{

    template <int dof, orwell::AdmittanceControllerType type>
    class LineAdmittanceController : public AdmittanceController<dof, gafro::Line<double>, type>
    {
      public:
        LineAdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        void setReferenceLine(const gafro::Line<double> &reference_line);

      private:
        void computeResiduals();

        gafro::Motor<double> getReferenceFrame();

        gafro::Motor<double> reference_frame_;

        gafro::Line<double> reference_line_;
    };

    template <int dof>
    using LineAdmittancePositionController = gafro_control::LineAdmittanceController<dof, orwell::AdmittanceControllerType::POSITION>;

}  // namespace gafro_control

#include <gafro/control/LineAdmittanceController.hxx>

// REGISTER_CLASS(orwell::TorqueController<7>, gafro_control::LineAdmittanceController<7, orwell::AdmittanceControllerType::TORQUE>,
//                "line_admittance_controller")
// REGISTER_CLASS(orwell::PositionController<7>, gafro_control::LineAdmittanceController<7, orwell::AdmittanceControllerType::POSITION>,
//                "line_admittance_controller")