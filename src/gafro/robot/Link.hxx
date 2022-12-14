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

#include <gafro/robot/Link.hpp>

namespace gafro
{

    template <class T>
    Link<T>::Link() : Link<T>(T(0.0), Translator<T>())
    {}

    template <class T>
    Link<T>::Link(const T &mass, const Translator<T> &center_of_mass, const Inertia<T> &inertia)
      : mass_(mass), center_of_mass_(center_of_mass), inertia_(inertia)
    {}

    template <class T>
    void Link<T>::setMass(const T &mass)
    {
        mass_ = mass;
    }

    template <class T>
    void Link<T>::setCenterOfMass(const Translator<T> &center_of_mass)
    {
        center_of_mass_ = center_of_mass;
    }

    template <class T>
    void Link<T>::setInertia(const Inertia<T> &inertia)
    {
        inertia_ = inertia;
    }

    template <class T>
    const T &Link<T>::getMass() const
    {
        return mass_;
    }

    template <class T>
    const Translator<T> &Link<T>::getCenterOfMass() const
    {
        return center_of_mass_;
    }

    template <class T>
    const Inertia<T> &Link<T>::getInertia() const
    {
        return inertia_;
    }

}  // namespace gafro