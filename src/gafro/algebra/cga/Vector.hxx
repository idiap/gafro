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

}  // namespace gafro