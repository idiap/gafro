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

#include <gafro/algebra/cga/Vector.hpp>

namespace gafro
{

    template <class T>
    Vector<T>::Vector() : Vector<T>(0.0, 0.0, 0.0)
    {}

    template <class T>
    Vector<T>::Vector(const T &x, const T &y, const T &z)
    {
        this->template set<blades::e1>(x);
        this->template set<blades::e2>(y);
        this->template set<blades::e3>(z);
    }

    template <class T>
    Vector<T>::Vector(const Base &other) : Base(other)
    {}

    template <class T>
    Rotor<T> Vector<T>::getRotor(const Vector &other) const
    {
        return Rotor<T>(Scalar<T>(TypeTraits<T>::Value(1.0)) + ((*this) | other) - ((*this) ^ other)).normalized();
    }

    template <class T>
    Vector<T> Vector<T>::Z(const T &z)
    {
        return Vector(0.0, 0.0, z);
    }

    template <class T>
    Vector<T> Vector<T>::UnitZ()
    {
        return Z(1.0);
    }

}  // namespace gafro