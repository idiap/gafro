// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/TangentVector.hpp>

namespace gafro
{

    template <class T>
    TangentVector<T>::TangentVector() : TangentVector<T>(0.0, 0.0, 0.0)
    {}

    template <class T>
    TangentVector<T>::TangentVector(const T &x, const T &y, const T &z)
    {
        this->template set<blades::e01>(x);
        this->template set<blades::e02>(y);
        this->template set<blades::e03>(z);
    }

    template <class T>
    TangentVector<T>::TangentVector(const Base &other) : Base(other)
    {}

}  // namespace gafro