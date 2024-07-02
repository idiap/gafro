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

#include <gafro/algebra/Algebra.hpp>

namespace gafro
{
    template <class Metric>
    template <class T, int i1, int i2>
    class Algebra<Metric>::OuterProduct
    {
      public:
        constexpr static int resulting_grade = Algebra::BladeBitmap::template getGrade<i1 & i2>();

        using Type = std::conditional_t<resulting_grade,                                    //
                                        typename Algebra<Metric>::template Multivector<T>,  //
                                        typename Algebra<Metric>::template Multivector<T, (i1 ^ i2)>>;

        constexpr static std::array<double, Type::size> signs = [] {
            if constexpr (resulting_grade)
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