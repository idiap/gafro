#pragma once

#include <gafro/gafro.hpp>
#include <gafro_control/AdmittanceController.hpp>

namespace gafro_control
{

    template <int dof>
    class MotorAdmittanceController : public AdmittanceController<dof, gafro::Motor<double>>
    {
      public:
        MotorAdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

      private:
        void computeResiduals();

        gafro::Motor<double> getReferenceFrame();
    };

}  // namespace gafro_control

#include <gafro_control/MotorAdmittanceController.hxx>

REGISTER_CLASS(orwell::TorqueController<7>, gafro_control::MotorAdmittanceController<7>, "motor_admittance_controller")