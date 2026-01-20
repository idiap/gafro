// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/gafro.hpp>
//
#include <orwell/TaskSpaceObjective.hpp>

namespace gafro
{

    class DualArmMotorReference
    {
      public:
        DualArmMotorReference() = default;

        DualArmMotorReference(const gafro::Motor<double> &absolute_target, const gafro::Motor<double> &relative_target);

        void setAbsoluteTarget(const gafro::Motor<double> &absolute_target);

        void setRelativeTarget(const gafro::Motor<double> &relative_target);

        const gafro::Motor<double> &getAbsoluteTarget() const;

        const gafro::Motor<double> &getRelativeTarget() const;

      private:
        gafro::Motor<double> absolute_target_;
        gafro::Motor<double> relative_target_;
    };

    template <int dof>
    class DualArmMotorObjective : public orwell::TaskSpaceObjective<dof, 12, DualArmMotorReference>
    {
      public:
        using State = orwell::TaskSpaceState<dof, 12, DualArmMotorReference>;

        DualArmMotorObjective(const orwell::TaskSpace<dof>::Ptr &task_space);

        State computeState(const orwell::RobotState<dof> &robot_state) const;

        Eigen::Vector<double, 12> computeError(const orwell::RobotState<dof> &robot_state) const;

        bool isReached() const;

        mutable bool is_reached_;
    };

}  // namespace gafro

#include <gafro/control/DualArmMotorObjective.hxx>