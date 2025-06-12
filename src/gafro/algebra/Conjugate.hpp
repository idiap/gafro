// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/UnaryExpression.hpp>

namespace gafro
{

    template <class M>
    class Conjugate : public UnaryExpression<Conjugate<M>, M, typename M::Type>
    {
      public:
        using Type = typename M::Type;
        using Vtype = typename M::Vtype;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits = Type::bits;
        // constexpr static auto map = Type::map;
        constexpr static auto has = Type::has;

        Conjugate(const M &m1)  //
          : UnaryExpression<Conjugate<M>, M, typename M::Type>(m1)
        {}

        Conjugate(M &&m1)  //
          : UnaryExpression<Conjugate<M>, M, typename M::Type>(std::move(m1))
        {}

        virtual ~Conjugate() = default;

        template <int blade>
            requires(has(blade))  //
        Vtype get() const
        {
            constexpr int grade = M::MAlgebra::BladeBitmap::template getGrade<blade>();
            constexpr int ngrade = M::MAlgebra::BladeBitmap::template getNegativeGrade<blade>();
            constexpr int e = grade * (grade - 1) / 2;
            constexpr double sign = math::pown<ngrade>() * math::pown<e>();

            return sign * this->operand().template get<blade>();
        }

      protected:
      private:
    };

}  // namespace gafro