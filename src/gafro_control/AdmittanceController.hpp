#pragma once

#include <gafro/gafro.hpp>
//
#include <orwell/torque/AdmittanceController.hpp>

namespace gafro_control
{

    template <int dof, class Reference>
    class AdmittanceController : public orwell::AdmittanceController<dof, Reference>
    {
      public:
        AdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        virtual ~AdmittanceController();

      protected:
        typename orwell::RobotState<dof>::Vector computeDesiredJointAcceleration();

        void setResidualBivector(const gafro::Motor<double>::Generator &residual_bivector);

        void setResidualTwist(const gafro::Twist<double> &residual_twist);

        void setExternalWrench(const gafro::Wrench<double> &external_wrench);

      protected:
        virtual void computeResiduals() = 0;

        virtual gafro::Motor<double> getReferenceFrame() = 0;

      private:
        gafro::Inertia<double> inertia_;
        gafro::Inertia<double> damping_;
        gafro::Inertia<double> stiffness_;

        gafro::Motor<double>::Generator residual_bivector_;
        gafro::Twist<double> residual_twist_;
        gafro::Wrench<double> external_wrench_;
    };

}  // namespace gafro_control

#include <gafro_control/AdmittanceController.hxx>