// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/gafro.hpp>
//
#include <orwell/TaskSpaceObjective.hpp>

namespace gafro_control
{

    template <int dof>
    class SingleArmMotorObjective : public orwell::TaskSpaceObjective<dof, 6, gafro::Motor<double>>
    {
      public:
        using State = orwell::TaskSpaceState<dof, 6, gafro::Motor<double>>;

        SingleArmMotorObjective();

        State computeState(const orwell::TaskSpace<dof>::Ptr &task_space, const orwell::RobotState<dof> &robot_state) const;
    };

}  // namespace gafro_control

#include <gafro/control/SingleArmMotorObjective.hxx>
