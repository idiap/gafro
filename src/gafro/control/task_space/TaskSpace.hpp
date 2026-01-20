#pragma once

#include <gafro/robot/System.hpp>

namespace gafro
{

    template <class T>
    class TaskSpace
    {
      public:
        TaskSpace(System<T> *system);

        virtual ~TaskSpace();

        System<T> *getSystem();

        const System<T> *getSystem() const;

      protected:
      private:
        System<T> *system_;
    };

}  // namespace gafro

#include <gafro/control/task_space/TaskSpace.hxx>