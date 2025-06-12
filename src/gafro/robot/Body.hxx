// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/Body.hpp>

namespace gafro
{

    template <class T>
    Body<T>::Body()
    {}

    template <class T>
    Body<T>::Body(Body &&other)
    {
        *this = std::move(other);
    }

    template <class T>
    void Body<T>::setCenterOfMass(const Translator<T> &center_of_mass)
    {
        center_of_mass_ = center_of_mass;
    }

    template <class T>
    void Body<T>::setInertia(const Inertia<T> &inertia)
    {
        inertia_ = inertia;
    }

    template <class T>
    void Body<T>::setAxis(const typename Motor<T>::Generator &axis)
    {
        axis_ = axis;
    }

    template <class T>
    void Body<T>::setMass(const T &mass)
    {
        mass_ = mass;
    }

    template <class T>
    const Translator<T> &Body<T>::getCenterOfMass() const
    {
        return center_of_mass_;
    }

    template <class T>
    const Inertia<T> &Body<T>::getInertia() const
    {
        return inertia_;
    }

    template <class T>
    const typename Motor<T>::Generator &Body<T>::getAxis() const
    {
        return axis_;
    }

    template <class T>
    const T &Body<T>::getMass() const
    {
        return mass_;
    }

}  // namespace gafro
