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
    class PlaneAdmittanceController : public AdmittanceController<dof, gafro::Plane<double>, type>
    {
      public:
        PlaneAdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

      private:
        void computeResiduals();

        gafro::Motor<double> getReferenceFrame();

        gafro::Motor<double> reference_frame_;
    };

    template <int dof>
    using PlaneAdmittanceTorqueController = gafro_control::PlaneAdmittanceController<dof, orwell::AdmittanceControllerType::TORQUE>;

    template <int dof>
    using PlaneAdmittancePositionController = gafro_control::PlaneAdmittanceController<dof, orwell::AdmittanceControllerType::POSITION>;

}  // namespace gafro_control

#include <gafro_control/PlaneAdmittanceController.hxx>

REGISTER_CLASS(orwell::TorqueController<7>, gafro_control::PlaneAdmittanceTorqueController<7>, "plane_admittance_controller")
REGISTER_CLASS(orwell::PositionController<7>, gafro_control::PlaneAdmittancePositionController<7>, "plane_admittance_controller")