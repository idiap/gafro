// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/ConformalTransformation.hpp>

namespace gafro
{

    template <class T>
    class ConformalTransformation<T>::Exponential
      : public UnaryExpression<ConformalTransformation<T>::Exponential, typename ConformalTransformation<T>::Generator, ConformalTransformation<T>>
    {
        Exponential(const typename ConformalTransformation<T>::Generator &generator);
        virtual ~Exponential() = default;

        template <int blade>
            requires(ConformalTransformation<T>::has(blade))  //
        T get() const;
    };

}  // namespace gafro