// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/AbstractMultivector.hpp>

namespace gafro
{

    template <class Derived>
    AbstractMultivector<Derived>::AbstractMultivector() = default;

    template <class Derived>
    AbstractMultivector<Derived>::~AbstractMultivector() = default;

    template <class Derived>
    Derived &AbstractMultivector<Derived>::derived()
    {
        return *static_cast<Derived *>(this);
    }

    template <class Derived>
    const Derived &AbstractMultivector<Derived>::derived() const
    {
        return *static_cast<const Derived *>(this);
    }

}  // namespace gafro
