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

#include <gafro_control/LineImpedanceController.hxx>

REGISTER_CLASS(orwell::TorqueController<7>, gafro_control::LineImpedanceController<7>, "line_impedance_controller")