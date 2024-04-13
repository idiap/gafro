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
    template <class T, int b1, int b2>
    class Algebra<Metric>::GeometricProduct
    {
      public:
        template <int k, int... ks>
        struct OuterRecursion
        {
            constexpr static bool value()
            {
                if ((Metric::template get<k, k>() == 0.0) && (((b1 & (1 << k)) != 0) && ((b2 & (1 << k)) != 0)))
                {
                    return false;
                }

                return true && (OuterRecursion<ks>::value() && ...);
            }
        };

        template <int i, int j, int lhs, int rhs, int sign>
        constexpr static std::pair<int, int> next()
        {
            if constexpr (j == Metric::dim - 1 && i < Metric::dim - 1)
            {
                return recurse<i + 1, 0, lhs, rhs, sign>();
            }
            else if constexpr (j == Metric::dim - 1 && i == Metric::dim - 1)
            {
                if constexpr ((lhs & rhs) != 0)
                {
                    return std::make_pair(lhs ^ rhs, 0);
                }

                if constexpr ((Algebra<Metric>::BladeBitmap::template getGrade<lhs ^ rhs>() !=
                               Algebra<Metric>::BladeBitmap::template getGrade<b1, b2>()))
                {
                    constexpr int s = sign * Algebra<Metric>::BladeBitmap::template reorderSign<lhs, rhs>();

                    return std::make_pair(lhs ^ rhs, s);
                }

                return std::make_pair(lhs ^ rhs, sign);
            }
            else
            {
                return recurse<i, j + 1, lhs, rhs, sign>();
            }
        }

        template <int i, int j, int lhs, int rhs, int sign>
        constexpr static std::pair<int, int> recurse()
        {
            if constexpr ((Metric::template get<i, j>() != 0.0) && (((b1 & (1 << i)) != 0) && ((b2 & (1 << j)) != 0)))
            {
                constexpr int s = sign * Metric::template get<i, j>() *
                                  math::pown<math::positive<Algebra<Metric>::BladeBitmap::template getRightShifts<(1 << i), lhs>()>() +
                                             math::positive<Algebra<Metric>::BladeBitmap::template getLeftShifts<(1 << j), rhs>()>()>();

                constexpr int lhs2 = lhs ^ (1 << i);
                constexpr int rhs2 = rhs ^ (1 << j);

                return next<i, j, lhs2, rhs2, s>();
            }

            return next<i, j, lhs, rhs, sign>();
        }

        template <std::size_t... k>
        constexpr static bool outer_recurse(std::index_sequence<k...>)
        {
            return true && (OuterRecursion<k>::value() && ...);
        }

        constexpr static std::pair<int, int> result_inner = recurse<0, 0, b1, b2, 1>();

        constexpr static int value_inner()
        {
            return result_inner.first;
        }

        constexpr static double sign_inner()
        {
            return result_inner.second;
        }

        constexpr static double sign_outer()
        {
            return valid_outer() ? Algebra<Metric>::BladeBitmap::template reorderSign<b1, b2>() : 0.0;
        }

        constexpr static bool valid_inner()
        {
            return sign_inner() != 0.0;
        }

        constexpr static bool valid_outer()
        {
            return outer_recurse(std::make_index_sequence<Metric::dim>()) && (value_inner() != (b1 ^ b2));
        }

        constexpr static int value_outer()
        {
            return (b1 ^ b2);
        }

        using InnerType = std::conditional_t<valid_inner(),                                               //
                                             typename Algebra::template Multivector<T, (value_inner())>,  //
                                             typename Algebra::template Multivector<T>>;

        using OuterType = std::conditional_t<valid_outer(),                                               //
                                             typename Algebra::template Multivector<T, (value_outer())>,  //
                                             typename Algebra::template Multivector<T>>;

        using Type = typename Sum<InnerType, OuterType, detail::AdditionOperator>::Type;

        constexpr static std::array<double, Type::size> signs = [] {
            if constexpr (!valid_inner() && !valid_outer())
            {
                return std::array<double, 0>();
            }
            else if constexpr (valid_inner() && !valid_outer())
            {
                return std::array<double, 1>({ sign_inner() });
            }
            else if constexpr (!valid_inner() && valid_outer())
            {
                return std::array<double, 1>({ sign_outer() });
            }
            else
            {
                return std::array<double, 2>({ sign_inner(), sign_outer() });
            }
        }();
    };

}  // namespace gafro