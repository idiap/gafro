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
    class Inverse : public UnaryExpression<Inverse<M>, M, M>
    {
      public:
        using Type = typename M::Type;
        using Vtype = typename M::Vtype;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits = Type::bits;
        // constexpr static auto map = Type::map;
        constexpr static auto has = Type::has;

        Inverse(const M &m1) : UnaryExpression<Inverse<M>, M, M>(m1)
        {
            norm_ = m1.squaredNorm();
        }

        Inverse(M &&m1) : UnaryExpression<Inverse<M>, M, M>(std::move(m1))
        {
            norm_ = m1.squaredNorm();
        }

        virtual ~Inverse() = default;

        template <int blade>
            requires(has(blade))  //
        Vtype get() const
        {
            constexpr int grade = M::MAlgebra::BladeBitmap::template getGrade<blade>();
            constexpr int e = grade * (grade - 1) / 2;
            constexpr double sign = math::pown<e>();

            return sign * this->operand().template get<blade>() / norm_;
        }

      protected:
      private:
        Vtype norm_;
    };

}  // namespace gafro