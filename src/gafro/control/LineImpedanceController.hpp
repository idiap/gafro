// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/gafro.hpp>
//
#include <orwell/torque/CartesianImpedanceController.hpp>

namespace gafro_control
{

    template <int dof>
    class LineImpedanceController : public orwell::CartesianImpedanceController<dof, gafro::Line<double>>
    {
      public:
        LineImpedanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        virtual ~LineImpedanceController();

      private:
        void computeStateError();
    };

}  // namespace gafro_control

#include <gafro/control/LineImpedanceController.hxx>

REGISTER_CLASS(orwell::TorqueController<7>, gafro_control::LineImpedanceController<7>, "line_impedance_controller")