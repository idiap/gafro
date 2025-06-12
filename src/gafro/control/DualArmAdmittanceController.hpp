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
#include <orwell/AdmittanceController.hpp>

namespace gafro_control
{

    template <int dof, class Reference, orwell::AdmittanceControllerType type>
    class DualArmAdmittanceController : public orwell::AdmittanceController<2 * dof, Reference, type>
    {
      public:
        DualArmAdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        virtual ~DualArmAdmittanceController();

        void setDesiredWrench(const gafro::Wrench<double> &w1, const gafro::Wrench<double> &w2);

        void setExternalWrench(const gafro::Wrench<double> &w1, const gafro::Wrench<double> &w2);

      protected:
        typename orwell::RobotState<2 * dof>::Vector computeDesiredJointAcceleration();

        gafro::Twist<double> computeDesiredAbsoluteEEAcceleration() const;

        gafro::Twist<double> computeDesiredRelativeEEAcceleration() const;

        virtual void computeResiduals() = 0;

        void setAbsoluteResidual(const gafro::Motor<double>::Generator &absolute_residual);

        void setAbsoluteResidualDt(const gafro::Twist<double> &absolute_residual_dt);

        void setRelativeResidual(const gafro::Motor<double>::Generator &relative_residual);

        void setRelativeResidualDt(const gafro::Twist<double> &relative_residual_dt);

        void convertWrenchesToDualTaskSpace(const gafro::Wrench<double> &w1, const gafro::Wrench<double> &w2, gafro::Wrench<double> &absolute,
                                            gafro::Wrench<double> &relative);

      private:
        gafro::Inertia<double> absolute_inertia_;
        gafro::Inertia<double> absolute_damping_;
        gafro::Inertia<double> absolute_stiffness_;

        gafro::Inertia<double> relative_inertia_;
        gafro::Inertia<double> relative_damping_;
        gafro::Inertia<double> relative_stiffness_;

        gafro::Wrench<double> desired_absolute_wrench_;
        gafro::Wrench<double> external_absolute_wrench_;

        gafro::Wrench<double> desired_relative_wrench_;
        gafro::Wrench<double> external_relative_wrench_;

        gafro::Twist<double> absolute_residual_;
        gafro::Twist<double> absolute_residual_dt_;
        gafro::Twist<double> relative_residual_;
        gafro::Twist<double> relative_residual_dt_;
    };

}  // namespace gafro_control

#include <gafro_control/DualArmAdmittanceController.hxx>