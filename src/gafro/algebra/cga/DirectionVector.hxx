// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/DirectionVector.hpp>

namespace gafro
{

    template <class T>
    DirectionVector<T>::DirectionVector() : DirectionVector<T>(0.0, 0.0, 0.0)
    {}

    template <class T>
    DirectionVector<T>::DirectionVector(const T &x, const T &y, const T &z)
    {
        this->template set<blades::e1i>(x);
        this->template set<blades::e2i>(y);
        this->template set<blades::e3i>(z);
    }

    template <class T>
    DirectionVector<T>::DirectionVector(const Base &other) : Base(other)
    {}

}  // namespace gafro