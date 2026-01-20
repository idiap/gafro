#pragma once

#include <gafro/control/task_space/SingleArmTaskSpace.hpp>

namespace gafro
{

    template <class T, int dof>
    SingleArmTaskSpace<T, dof>::SingleArmTaskSpace(Manipulator<T, dof> *system)
      : system_(system)
    {}

    template <class T, int dof>
    SingleArmTaskSpace<T, dof>::~SingleArmTaskSpace() = default;

    template <class T, int dof>
    Manipulator<T, dof> *SingleArmTaskSpace<T, dof>::getSystem()
    {
        return system_;
    }

    template <class T, int dof>
    const Manipulator<T, dof> *SingleArmTaskSpace<T, dof>::getSystem() const
    {
        return system_;
    }

    template <class T, int dof>
    typename orwell::RobotState<dof>::Vector SingleArmTaskSpace<T, dof>::computeInverseDynamics(
      const typename orwell::RobotState<dof>::Vector &position,
      const typename orwell::RobotState<dof>::Vector &velocity,
      const typename orwell::RobotState<dof>::Vector &acceleration) const
    {
        return system_->getJointTorques(position, velocity, acceleration, 0.0);
    }

}  // namespace gafro