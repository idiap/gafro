#pragma once

#include <gafro/control/task_space/TaskSpace.hpp>

namespace gafro
{

    template <class T>
    TaskSpace<T>::TaskSpace(System<T> *system)
      : system_(system)
    {}

    template <class T>
    TaskSpace<T>::~TaskSpace() = default;

    template <class T>
    System<T> *TaskSpace<T>::getSystem()
    {
        return system_;
    }

    template <class T>
    const System<T> *TaskSpace<T>::getSystem() const
    {
        return system_;
    }

}  // namespace gafro