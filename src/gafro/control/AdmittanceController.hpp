// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/gafro.hpp>
//
#include <orwell/AdmittanceController.hpp>

namespace gafro_control
{

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    class AdmittanceController : public orwell::AdmittanceController<dof, Reference, type>
    {
      public:
        AdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        virtual ~AdmittanceController();

        void setDesiredWrench(const gafro::Wrench<double> &desired_wrench);

        void setExternalWrench(const gafro::Wrench<double> &external_wrench);

        void setInertia(const gafro::Inertia<double> &inertia);

        void setDamping(const gafro::Inertia<double> &damping);

        void setStiffness(const gafro::Inertia<double> &stiffness);

        const gafro::Inertia<double> &getInertia() const;

        const gafro::Inertia<double> &getDamping() const;

        const gafro::Inertia<double> &getStiffness() const;

      protected:
        typename orwell::RobotState<dof>::Vector computeDesiredJointAcceleration();

        void setResidualBivector(const gafro::Motor<double>::Generator &residual_bivector);

        void setResidualTwist(const gafro::Twist<double> &residual_twist);

      protected:
        virtual void computeResiduals() = 0;

        virtual gafro::Motor<double> getReferenceFrame() = 0;

      private:
        gafro::Inertia<double> inertia_;
        gafro::Inertia<double> damping_;
        gafro::Inertia<double> stiffness_;

        gafro::Motor<double>::Generator residual_bivector_;
        gafro::Twist<double> residual_twist_;

        gafro::Wrench<double> desired_wrench_;
        gafro::Wrench<double> external_wrench_;
    };

}  // namespace gafro_control

#include <gafro_control/AdmittanceController.hxx>