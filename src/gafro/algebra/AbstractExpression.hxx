// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/AbstractExpression.hpp>

namespace gafro
{

    template <class Derived>
    AbstractExpression<Derived>::AbstractExpression() = default;

    template <class Derived>
    AbstractExpression<Derived>::~AbstractExpression() = default;

    template <class Derived>
    Derived &AbstractExpression<Derived>::derived()
    {
        return *static_cast<Derived *>(this);
    }

    template <class Derived>
    const Derived &AbstractExpression<Derived>::derived() const
    {
        return *static_cast<const Derived *>(this);
    }

}  // namespace gafro
