#pragma once

#include <gafro/algebra/Algebra.hpp>

namespace gafro
{

    template <class M>
    template <class T, int grade>
    class Algebra<M>::GradeConstructor
    {
        template <int blade>
        constexpr static auto multivector()
        {
            if constexpr (Algebra<M>::BladeBitmap::template getGrade<blade>() == grade)
            {
                return typename Algebra<M>::Multivector<T, blade>();
            }
            else
            {
                return typename Algebra<M>::Multivector<T>();
            }
        }

        template <std::size_t... blades>
        constexpr static auto makeGrade(std::index_sequence<blades...>)
        {
            return typename Algebra<M>::Multivector<T>() + (multivector<blades>() + ...);
        }

      public:
        using Type = typename decltype(makeGrade(std::make_index_sequence<Algebra<M>::dim>{}))::Type;
    };

}  // namespace gafro