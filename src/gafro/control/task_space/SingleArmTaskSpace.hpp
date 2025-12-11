#pragma once

#include <gafro/robot/Manipulator.hpp>
#include <orwell/TaskSpace.hpp>

namespace gafro
{

    template <class T, int dof>
    class SingleArmTaskSpace : public orwell::TaskSpace<dof>
    {
      public:
        SingleArmTaskSpace(Manipulator<T, dof> *system);

        virtual ~SingleArmTaskSpace();

        Manipulator<T, dof> *getSystem();

        const Manipulator<T, dof> *getSystem() const;

        typename orwell::RobotState<dof>::Vector computeInverseDynamics(const typename orwell::RobotState<dof>::Vector &position,
                                                                        const typename orwell::RobotState<dof>::Vector &velocity,
                                                                        const typename orwell::RobotState<dof>::Vector &acceleration) const;

      protected:
      private:
        Manipulator<T, dof> *system_;
    };

}  // namespace gafro

#include <gafro/control/task_space/SingleArmTaskSpace.hxx>