// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Expression.hpp>
#include <gafro/algebra/Multivector.hpp>

namespace gafro
{

    template <class Derived, class Result, class T, class M, int... index>
    class Assign
    {
      public:
        static void run(typename Algebra<M>::template Multivector<T, index...> &multivector, const Expression<Derived, Result> &expression)
        {
            [&multivector, &expression]<std::size_t... i>(std::index_sequence<i...>) {
                (
                  [&multivector, &expression] {
                      constexpr int blade = Algebra<M>::template Multivector<T, index...>::blades()[i];

                      multivector.template set<blade>(expression.template get<blade>());
                  }(),
                  ...);
            }(std::make_index_sequence<Algebra<M>::template Multivector<T, index...>::size>());
        }
    };

}  // namespace gafro