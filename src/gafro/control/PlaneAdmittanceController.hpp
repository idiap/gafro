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
    class PlaneAdmittanceController : public AdmittanceController<dof, gafro::Plane<double>, type>
    {
      public:
        PlaneAdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        void setReferencePlane(const gafro::Plane<double> &reference_plane);

      private:
        void computeResiduals();

        gafro::Motor<double> getReferenceFrame();

        gafro::Motor<double> reference_frame_;

        gafro::Plane<double> reference_plane_;
    };

    template <int dof>
    using PlaneAdmittanceTorqueController = gafro_control::PlaneAdmittanceController<dof, orwell::AdmittanceControllerType::TORQUE>;

    template <int dof>
    using PlaneAdmittancePositionController = gafro_control::PlaneAdmittanceController<dof, orwell::AdmittanceControllerType::POSITION>;

}  // namespace gafro_control

#include <gafro_control/PlaneAdmittanceController.hxx>

REGISTER_CLASS(orwell::TorqueController<7>, gafro_control::PlaneAdmittanceTorqueController<7>, "plane_admittance_controller")
REGISTER_CLASS(orwell::PositionController<7>, gafro_control::PlaneAdmittancePositionController<7>, "plane_admittance_controller")