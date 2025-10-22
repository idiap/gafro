// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Vector.hpp>

namespace gafro
{

    template <class T>
    Vector<T>::Vector()
      : Vector<T>(0.0, 0.0, 0.0)
    {}

    template <class T>
    Vector<T>::Vector(const T &x, const T &y, const T &z)
    {
        this->template set<blades::e1>(x);
        this->template set<blades::e2>(y);
        this->template set<blades::e3>(z);
    }

    template <class T>
    Vector<T>::Vector(const Base &other)
      : Base(other)
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