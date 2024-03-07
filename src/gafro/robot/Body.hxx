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

}  // namespace gafro
