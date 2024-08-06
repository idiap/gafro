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