#pragma once

#include <gafro/algebra/Multivector.hpp>
#include <gafro/algebra/expressions/Expression.hpp>

namespace gafro
{

    template <class Derived, class Result, class T, int... index>
    class Assign
    {
      public:
        static void run(Multivector<T, index...> &multivector, const Expression<Derived, Result> &expression)
        {
            [&multivector, &expression ]<std::size_t... i>(std::index_sequence<i...>)
            {
                (
                  [&multivector, &expression] {
                      constexpr int blade = Multivector<T, index...>::blades()[i];

                      multivector.template set<blade>(expression.template get<blade>());
                  }(),
                  ...);
            }
            (std::make_index_sequence<Multivector<T, index...>::size>());
        }
    };

}  // namespace gafro