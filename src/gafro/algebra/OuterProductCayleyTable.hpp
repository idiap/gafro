// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Algebra.hpp>

namespace gafro
{
    template <class Metric>
    template <class T, int i1, int i2>
    class Algebra<Metric>::OuterProduct
    {
      public:
        constexpr static int resulting_grade = Algebra::BladeBitmap::template getGrade<i1 & i2>();

        using Type = std::conditional_t<resulting_grade != 0,                               //
                                        typename Algebra<Metric>::template Multivector<T>,  //
                                        typename Algebra<Metric>::template Multivector<T, (i1 ^ i2)>>;

        constexpr static std::array<double, Type::size> signs = [] {
            if constexpr (resulting_grade != 0)
            {
                return std::array<double, 0>();
            }
            else
            {
                return std::array<double, 1>({ Algebra::BladeBitmap::template reorderSign<i1, i2>() });
            }
        }();
    };

}  // namespace gafro